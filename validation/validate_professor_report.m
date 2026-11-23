function validate_professor_report
%VALIDATE_PROFESSOR_REPORT Matched report comparisons with template filters.
project=setup_project;[~,~,reference,rate]=aero2_course_reference;
models={'aero2_cascade_professor_format','aero2_integral_professor_format'};
runs=cell(3,1);steps=cell(2,6);metrics=table;stepMetrics=table;
labels={'IP cascade, without FF','IP cascade, with FF','Integral cascade, with FF'};
for design=1:2
    model=models{design};load_system(fullfile(project.simulink,[model '.slx']));
    experiment=[model '_report_experiment'];
    if bdIsLoaded(experiment),close_system(experiment,0);end
    save_system(model,fullfile(project.experiments,[experiment '.slx']));
    w=get_param(experiment,'ModelWorkspace');assignin(w,'Kff',1);
    loops={'Subsystem3','Subsystem2'};
    for axis=1:2
        loop=[experiment '/' loops{axis}];
        add_block('simulink/Math Operations/Gain',[loop '/Velocity feedforward switch'], ...
            'Gain','Kff','Position',[210 150 245 180]);
        connect(loop,'Derivative',1,'Velocity feedforward switch',1);
        connect(loop,'Velocity feedforward switch',1,'de',1);
    end
    plant=[experiment '/Twin rotor (simplified model)'];
    record(plant,'x',1,'true_state');
    record(experiment,'+//- 24V1',1,'main_voltage');
    record(experiment,'+//- 24V',1,'tail_voltage');
    if design==2
        record(experiment,'Outer integration gate',1,'outer_gate');
        record([experiment '/Subsystem3'],'Integrator',1,'yaw_integral');
        record([experiment '/Subsystem2'],'Integrator',1,'pitch_integral');
    end
    save_system(experiment);
    cases=0:1;if design==2,cases=1;end
    for feedforward=cases
        input=Simulink.SimulationInput(experiment);
        input=input.setVariable('Kff',feedforward,'Workspace',experiment);
        out=sim(input);r=plainRun(out,reference);
        index=feedforward+1;if design==2,index=3;end
        runs{index}=r;
        for axis=1:2
            error=r.data(:,axis)-r.data(:,axis+2);
            rmsError=sqrt(trapz(r.time,error.^2)/(r.time(end)-r.time(1)));
            row=table(labels(index),axis,rmsError,max(abs(error)),max(abs(r.data(:,5:6)),[],'all'), ...
                'VariableNames',{'Case','Axis','TrackingRMS','PeakError','PeakVoltage'});
            metrics=[metrics;row]; %#ok<AGROW>
        end
        if feedforward
            % The comparison instrumentation/switch must preserve the answer.
            load_system(fullfile(project.simulink,[model '.slx']));
            record([model '/Twin rotor (simplified model)'],'x',1,'true_state');
            simple=sim(model);
            assert(max(abs(matrix(simple.true_state)-matrix(out.true_state)),[],'all')<1e-10);
            close_system(model,0);
        end
    end
    % Independently compare the weighted-integral law against the old model
    % when its reference and tachometer conventions are matched.
    if design==2
        for name={'Transfer Fcn1','Transfer Fcn3'}
            set_param([experiment '/' name{1}],'Denominator','1');
        end
        for axis=1:2
            loop=[experiment '/' loops{axis}];
            replaceDerivative(loop,'From Workspace');
            variable=sprintf('MatchedRate%d',axis);assignin(w,variable,[rate(:,1) rate(:,axis+1)]);
            set_param([loop '/Derivative'],'VariableName',variable,'Interpolate','on', ...
                'OutputAfterFinalValue','Holding final value');
        end
        input=Simulink.SimulationInput(experiment);
        input=input.setVariable('meas_on',0,'Workspace',experiment);matched=sim(input);
        old='aero2_position_integral';load_system(fullfile(project.simulink,[old '.slx']));
        record([old '/Course nonlinear plant'],'x',1,'true_state');
        input=Simulink.SimulationInput(old);input=input.setVariable('meas_on',0,'Workspace',old);
        previous=sim(input);integralDifference=max(abs(matrix(matched.true_state)-matrix(previous.true_state)),[],'all');
        assert(integralDifference<1e-7,'Weighted integral law differs from the original design.');
        close_system(old,0);
        for name={'Transfer Fcn1','Transfer Fcn3'}
            set_param([experiment '/' name{1}],'Denominator','[1/180 1]');
        end
    end
    % Sharp steps do not enter a differentiator: replace only the reference
    % derivative source in memory; keep damping and all motor filters active.
    for axis=1:2
        loop=[experiment '/' loops{axis}];replaceDerivative(loop,'Constant');
        set_param([loop '/Derivative'],'Value','0');
    end
    commands=[0.1 0;-0.1 0;0 0.1;0 -0.1;0.2 0.1;-0.2 -0.1];
    for test=1:size(commands,1)
        target=commands(test,:);times=[0;0.198;0.2;8];
        values=[0 0;0 0;target;target];stepReference=[times values];
        input=Simulink.SimulationInput(experiment);
        input=input.setModelParameter('StopTime','8');
        input=input.setVariable('meas_on',0,'Workspace',experiment);
        input=input.setVariable('Kff',0,'Workspace',experiment);
        for axis=1:2
            names={'U_v','U_h'};
            input=input.setVariable(names{axis},timeseries(values(:,axis),times),'Workspace',experiment);
        end
        sources=find_system(experiment,'SearchDepth',1,'BlockType','FromWorkspace');
        for k=1:numel(sources)
            input=input.setBlockParameter(sources{k},'Interpolate','off');
            input=input.setBlockParameter(sources{k},'OutputAfterFinalValue','Holding final value');
        end
        out=sim(input);r=plainRun(out,stepReference,'previous');steps{design,test}=r;
        if design==2
            limited=any(abs(r.data(:,5:6))>=24-1e-8,2);
            gate=out.outer_gate.Data(:);assert(all(gate(limited)==0));
        end
        for axis=1:2
            if target(axis)==0
                assert(max(abs(r.data(:,axis+2)))<1e-6);continue;
            end
            selected=r.time>=0.2;t=r.time(selected)-0.2;y=r.data(selected,axis+2);
            last=find(abs(y-target(axis))>0.02*abs(target(axis)),1,'last');
            assert(~isempty(last) && last<numel(t),'Cascade did not settle.');
            settling=t(last+1);overshoot=max(0,max(sign(target(axis))*y)/abs(target(axis))-1)*100;
            row=table(models(design),test,axis,settling,overshoot,max(abs(r.data(:,5:6)),[],'all'), ...
                'VariableNames',{'Model','Test','Axis','SettlingTime','OvershootPercent','PeakVoltage'});
            stepMetrics=[stepMetrics;row]; %#ok<AGROW>
        end
    end
    if design==2
        % Excite actual voltage limiting to verify the optional outer gate.
        target=[0.5 0.2];values=[0 0;0 0;target;target];
        for axis=1:2
            names={'U_v','U_h'};
            input=input.setVariable(names{axis},timeseries(values(:,axis),times),'Workspace',experiment);
        end
        out=sim(input);integralStress=plainRun(out,[times values],'previous');
        gate=out.outer_gate.Data(:);frozen=gate(1:end-1)==0;assert(any(frozen));
        delta=diff([out.yaw_integral.Data(:) out.pitch_integral.Data(:)]);
        assert(max(abs(delta(frozen,:)),[],'all')<1e-10,'Outer integrals changed while gated off.');
    end
    close_system(experiment,0);
    fprintf('Report design %d: course comparisons, equivalence and signed step checks passed.\n',design);
end
folder=fullfile(project.results,'professor_report');if ~isfolder(folder),mkdir(folder);end
save(fullfile(folder,'comparisons.mat'),'runs','steps','integralDifference','integralStress');
writetable(metrics,fullfile(folder,'metrics.csv'));writetable(stepMetrics,fullfile(folder,'steps.csv'));
disp(metrics);disp(stepMetrics);fprintf('Matched integral difference: %.3g\n',integralDifference);
end

function r=plainRun(out,reference,method)
if nargin<3,method='linear';end
r.time=out.true_state.Time;x=matrix(out.true_state);
q=interp1(reference(:,1),reference(:,2:3),r.time,method);
r.data=[q x(:,1:2) out.main_voltage.Data(:) out.tail_voltage.Data(:)];
assert(all(isfinite(r.data),'all') && max(abs(r.data(:,5:6)),[],'all')<=24+1e-10);
assert(max(abs(x(:,2)))<=pi/3,'Report run left the pitch envelope.');
end

function x=matrix(signal)
x=reshape(signal.Data,numel(signal.Time),[]);
end

function record(system,source,port,variable)
name=['Record ' variable];add_block('simulink/Sinks/To Workspace',[system '/' name], ...
    'VariableName',variable,'SaveFormat','Timeseries','MaxDataPoints','inf');
connect(system,source,port,name,1);
end

function connect(system,source,sourcePort,destination,destinationPort)
from=get_param([system '/' source],'PortHandles');to=get_param([system '/' destination],'PortHandles');
line=get_param(to.Inport(destinationPort),'Line');if line~=-1,delete_line(line);end
add_line(system,from.Outport(sourcePort),to.Inport(destinationPort),'autorouting','on');
end

function replaceDerivative(loop,kind)
block=[loop '/Derivative'];position=get_param(block,'Position');ports=get_param(block,'PortHandles');
line=get_param(ports.Outport,'Line');to=get_param(line,'DstPortHandle');delete_line(line);
if ~isempty(ports.Inport)
    line=get_param(ports.Inport(1),'Line');if line~=-1,delete_line(line);end
end
delete_block(block);add_block(['simulink/Sources/' kind],block,'Position',position);
from=get_param(block,'PortHandles');add_line(loop,from.Outport(1),to);
end
