function metrics=validate_filter_remedies
%VALIDATE_FILTER_REMEDIES Compare remedies only in the HIL-free full cascade.
p=setup_project; c=cascade_parameters;
offline='aero2_hardware_cascade_offline'; direct='aero2_cascade_control_experiment';
load_system(fullfile(p.experiments,[offline '.slx']));
load_system(fullfile(p.experiments,[direct '.slx']));
cases={'Original',offline,c.kp,c.kd,10; 'Filter 30',offline,c.kp,c.kd,30; ...
    'Gains 2.25',offline,2.25,2.25,10; 'Gains 2.25 direct',direct,2.25,2.25,NaN};
metrics=table; runs=cell(4,2);
for k=1:4
  for direction=1:2
    target=(3-2*direction)*[.2;.1];
    input=Simulink.SimulationInput(cases{k,2});
    input=input.setVariable('Kq',cases{k,3},'Workspace',cases{k,2});
    input=input.setVariable('Dq',cases{k,4},'Workspace',cases{k,2});
    input=input.setVariable('Kff',0,'Workspace',cases{k,2});
    input=input.setVariable('meas_on',0,'Workspace',cases{k,2});
    input=input.setModelParameter('StopTime','12');
    if k<4
        uv=timeseries([0;target(1);target(1)],[0;.2;12]);
        uh=timeseries([0;target(2);target(2)],[0;.2;12]);
        input=input.setVariable('U_v',uv,'Workspace',offline);
        input=input.setVariable('U_h',uh,'Workspace',offline);
        for axis={'yaw','pitch'}
            name=[upper(axis{1}(1)) axis{1}(2:end)];
            input=input.setBlockParameter([offline '/Desired orientation/Course ' axis{1} ' reference'], ...
                'Interpolate','off');
            for suffix={' rate',' filtered'}
                wn=cases{k,5}; num=num2str(wn^2);
                if strcmp(suffix{1},' rate'), num=mat2str([wn^2 0]); end
                path=[offline '/Offline AERO 2 surrogate/' name suffix{1}];
                input=input.setBlockParameter(path,'Numerator',num);
                input=input.setBlockParameter(path,'Denominator',mat2str([1 1.6*wn wn^2]));
            end
        end
    else
        ref=[0 0 0;.198 0 0;.2 target';12 target'];
        input=input.setVariable('Reference',ref,'Workspace',direct);
        input=input.setBlockParameter([direct '/Desired orientation'],'Interpolate','off');
    end
    output=sim(input);
    if k<4, signal=output.hardware_offline_data; width=14; voltageColumns=13:14;
    else, signal=output.cascade_data; width=24; voltageColumns=15:16; end
    r.time=signal.Time;
    if signal.IsTimeFirst, r.data=reshape(signal.Data,numel(r.time),width);
    else, r.data=reshape(signal.Data,width,[])'; end
    runs{k,direction}=r;
    for axis=1:2
        y=r.data(:,axis+2); selected=r.time>=.2;
        t=r.time(selected)-.2; q=y(selected); last=find(abs(q-target(axis))>.02*abs(target(axis)),1,'last');
        ts=NaN; if ~isempty(last) && last<numel(t), ts=t(last+1); end
        os=100*max(0,max(sign(target(axis))*q)-abs(target(axis)))/abs(target(axis));
        row=table(cases(k,1),direction,axis,ts,os,max(abs(r.data(:,voltageColumns)),[],'all'), ...
            'VariableNames',{'Case','Direction','Axis','SettlingTime','OvershootPercent','PeakVoltage'});
        metrics=[metrics;row]; %#ok<AGROW>
    end
  end
end
folder=fullfile(p.results,'filter_remedies'); if ~isfolder(folder),mkdir(folder);end
save(fullfile(folder,'validation.mat'),'cases','runs','metrics');
writetable(metrics,fullfile(folder,'steps.csv')); disp(metrics);
assert(all(isfinite(metrics.SettlingTime)),'A remedy test did not settle.');
assert(all(metrics.PeakVoltage<=24+1e-9),'A remedy exceeds the voltage limit.');
end
