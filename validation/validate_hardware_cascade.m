function metrics = validate_hardware_cascade
%VALIDATE_HARDWARE_CASCADE Run ONLY the HIL-free offline surrogate.
project=setup_project; p=cascade_parameters;
hardware='aero2_hardware_cascade'; baseline='aero2_cascade_control';
load_system(fullfile(project.simulink,[hardware '.slx']));
load_system(fullfile(project.simulink,[baseline '.slx']));
for variable={'Kq','Dq','Kp','Ki','Kaw'}
    wh=get_param(hardware,'ModelWorkspace'); wb=get_param(baseline,'ModelWorkspace');
    assert(isequal(getVariable(wh,variable{1}),getVariable(wb,variable{1})), ...
        'Hardware gains differ from the Question 5 controller.');
end
root=sfroot;
ch=root.find('-isa','Stateflow.EMChart','Path',[hardware '/Mechanical inverse']);
cb=root.find('-isa','Stateflow.EMChart','Path',[baseline '/Mechanical inverse']);
assert(strcmp(ch.Script,cb.Script),'Mechanical inverse differs from Question 5.');
assert(strcmp(get_param(hardware,'FixedStep'),'0.002'),'Hardware step must be 2 ms.');

model='aero2_hardware_cascade_offline';
load_system(fullfile(project.experiments,[model '.slx']));
% Refuse simulation if a hardware reference survived the replacement.
refs=find_system(model,'FollowLinks','off','LookUnderMasks','all','BlockType','Reference');
for k=1:numel(refs)
    assert(~contains(get_param(refs{k},'SourceBlock'),'quarc_library'), ...
        'Offline model still contains a QUARC hardware reference.');
end
assert(strcmp(get_param(model,'SystemTargetFile'),'grt.tlc'),'Offline target must be grt.');
commands=[0 0; .1 0;-.1 0;0 .1;0 -.1;.2 .1;-.2 -.1];
metrics=table; runs=cell(size(commands,1),1);
for k=1:size(commands,1)
    [uv,uh]=steps(commands(k,:),8);
    r=runOffline(model,uv,uh,8,0,p.step,false); runs{k}=r;
    if k==1
        assert(max(abs(r.data(:)))<1e-10,'Zero-reference equilibrium failed.');
    end
    for axis=find(commands(k,:)~=0)
        y=r.data(:,2+axis); target=commands(k,axis);
        ts=settling(r.time,y,target);
        overshoot=max(0,max(sign(target)*(y-target)))/abs(target)*100;
        row=table(k,axis,target,ts,overshoot,y(end)-target, ...
            max(abs(r.data(:,13:14)),[],'all'), ...
            p.step*sum(any(abs(r.raw)>24+1e-8,2)), ...
            max(abs(r.integral),[],'all'), ...
            'VariableNames',{'Test','Axis','Command','SettlingTime','OvershootPercent', ...
            'FinalError','PeakVoltage','SaturationDuration','PeakIntegral'});
        row.TimingMet=ts>=2.4 && ts<=3.6;
        metrics=[metrics;row]; %#ok<AGROW>
        assert(isfinite(ts),'Filtered cascade did not settle within the 8 s test.');
        assert(abs(y(end)-target)<.02*abs(target),'Step final error exceeds 2%%.');
    end
end
[uv,uh]=aero2_course_reference;
course=runOffline(model,uv,uh,60,0,p.step,true);
noisy=runOffline(model,uv,uh,60,1,p.step,true);
tracking=table;
for mode=1:2
    rlist={course,noisy}; r=rlist{mode}; e=r.data(:,1:2)-r.data(:,3:4);
    for axis=1:2
        tracking=[tracking;table(mode-1,axis,sqrt(mean(e(:,axis).^2)), ...
            max(abs(r.data(:,13:14)),[],'all'),max(abs(r.data(:,4))), ...
            'VariableNames',{'MeasurementEffects','Axis','RMS','PeakVoltage','PeakPitch'})]; %#ok<AGROW>
    end
end
% Keep sensor and controller sample times at 2 ms while refining integration.
[uv,uh]=steps([.2 .1],8);
fine=runOffline(model,uv,uh,8,0,p.step/2,false);
coarse=runs{6};
difference=max(abs(coarse.data(:,3:4)-interp1(fine.time,fine.data(:,3:4),coarse.time)),[],'all');
assert(difference<.002,'Solver refinement changes the response materially.');
refinement=table;
for axis=1:2
    target=[.2 .1]; target=target(axis);
    a=settling(coarse.time,coarse.data(:,axis+2),target);
    b=settling(fine.time,fine.data(:,axis+2),target);
    refinement=[refinement;table(axis,a,b,difference, ...
        'VariableNames',{'Axis','CoarseSettling','FineSettling','MaxAngleDifference'})]; %#ok<AGROW>
    assert(abs(a-b)<.03,'Solver refinement changes the settling conclusion.');
end
% Run the exported offline model in R2024b, without claiming R2021a execution.
export='aero2_hardware_cascade_offline_R2021a';
load_system(fullfile(project.experiments,[export '.slx']));
exportRun=runOffline(export,uv,uh,8,0,p.step,false);
assert(max(abs(exportRun.data(:)-coarse.data(:)))<1e-10,'Offline export differs.');
folder=fullfile(project.results,'hardware_cascade');
if ~isfolder(folder), mkdir(folder); end
save(fullfile(folder,'validation.mat'),'metrics','tracking','refinement','runs','course','noisy','fine');
writetable(metrics,fullfile(folder,'steps.csv'));
writetable(tracking,fullfile(folder,'tracking.csv'));
writetable(refinement,fullfile(folder,'refinement.csv'));
disp(metrics); disp(tracking); disp(refinement);
fprintf('Offline integrity/convergence checks passed. NO hardware model was run.\n');
fprintf('3 s timing gate: %d/%d passed; see TimingMet (retained gains + hardware filters).\n', ...
    sum(metrics.TimingMet),height(metrics));
end

function [uv,uh]=steps(q,stop)
t=[0;.2;stop];
uv=timeseries([0;q(1);q(1)],t); uh=timeseries([0;q(2);q(2)],t);
end

function r=runOffline(model,uv,uh,stop,noise,step,interpolate)
in=Simulink.SimulationInput(model);
in=in.setVariable('Kff',double(interpolate),'Workspace',model);
in=in.setVariable('U_v',uv,'Workspace',model);
in=in.setVariable('U_h',uh,'Workspace',model);
in=in.setVariable('meas_on',noise,'Workspace',model);
in=in.setModelParameter('StopTime',num2str(stop),'FixedStep',num2str(step));
setting='off'; if interpolate, setting='on'; end
for n={'Course yaw reference','Course pitch reference'}
    in=in.setBlockParameter([model '/Desired orientation/' n{1}],'Interpolate',setting);
end
out=sim(in); r.time=out.hardware_offline_data.Time;
r.data=squeeze(out.hardware_offline_data.Data);
if size(r.data,1)~=numel(r.time), r.data=r.data'; end
r.integral=squeeze(out.hardware_integral_data.Data);
r.raw=squeeze(out.hardware_raw_voltage_data.Data);
assert(all(isfinite(r.data(:))) && all(isfinite(r.integral(:))) && all(isfinite(r.raw(:))), ...
    'Nonfinite controller/plant signal.');
assert(max(abs(r.data(:,4)))<=pi/3,'Pitch envelope exceeded.');
assert(max(abs(r.data(:,13:14)),[],'all')<=24+1e-9,'Voltage limit exceeded.');
assert(max(abs(r.integral),[],'all')<100,'Excessive integral contribution.');
end

function ts=settling(t,y,target)
last=find(abs(y-target)>.02*abs(target),1,'last');
if isempty(last), ts=0;
elseif last==numel(t), ts=NaN;
else, ts=t(last+1)-.2;
end
end
