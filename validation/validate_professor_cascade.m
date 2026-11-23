function metrics = validate_professor_cascade
%VALIDATE_PROFESSOR_CASCADE Course tracking, equivalent dynamics and saturation.
project=setup_project; p=cascade_parameters;
model='aero2_cascade_professor_format';
load_system(fullfile(project.simulink,[model '.slx']));
experiment=[model '_experiment'];
if bdIsLoaded(experiment), close_system(experiment,0); end
save_system(model,fullfile(project.experiments,[experiment '.slx']));
plant=[experiment '/Twin rotor (simplified model)'];
record(plant,'x','true_state');
record(experiment,'+//- 24V1','main_voltage');
record(experiment,'+//- 24V','tail_voltage');
record(experiment,'IP_m','main_raw'); record(experiment,'IP_t','tail_raw');
% Every limiter output must reach the plant and its own anti-windup port.
for axis=1:2
    limits={'+//- 24V1','+//- 24V'}; loops={'IP_m','IP_t'};
    ports=get_param([experiment '/' limits{axis}],'PortHandles');
    destination=get_param(get_param(ports.Outport,'Line'),'DstPortHandle');
    expected=get_param([experiment '/' loops{axis}],'PortHandles');
    assert(any(destination==expected.Inport(1)),'Missing anti-windup feedback.');
    expected=get_param(plant,'PortHandles');
    assert(any(destination==expected.Inport(axis)),'Plant bypasses the voltage limit.');
end
save_system(experiment);
metrics=table; runs=cell(2,1);
[~,~,reference,referenceRate]=aero2_course_reference;
for mode=0:1
    input=Simulink.SimulationInput(experiment);
    input=input.setVariable('meas_on',mode,'Workspace',experiment);
    out=sim(input); runs{mode+1}=out;
    t=out.true_state.Time; x=squeeze(out.true_state.Data);
    volts=[out.main_voltage.Data(:) out.tail_voltage.Data(:)];
    assert(all(isfinite(x),'all') && max(abs(volts),[],'all')<=24+1e-12);
    assert(max(abs(x(:,2)))<pi/3,'Pitch leaves the validated simulation envelope.');
    target=interp1(reference(:,1),reference(:,2:3),t);
    error=x(:,1:2)-target;
    rmsError=sqrt(trapz(t,error.^2)/(t(end)-t(1)));
    metrics=[metrics;table(mode,rmsError(1),rmsError(2),max(abs(volts),[],'all'), ...
        'VariableNames',{'SensorEffects','YawRMS','PitchRMS','PeakVoltage'})]; %#ok<AGROW>
    fprintf('60 s course run, sensor effects %d: RMS %.4f / %.4f rad\n', ...
        mode,rmsError(1),rmsError(2));
end
% The diagnostic copy must preserve the simple model's default dynamics.
load_system(fullfile(project.simulink,[model '.slx']));
record([model '/Twin rotor (simplified model)'],'x','true_state');
input=Simulink.SimulationInput(model);
input=input.setVariable('meas_on',0,'Workspace',model); simple=sim(input);
assert(max(abs(squeeze(simple.true_state.Data)-squeeze(runs{1}.true_state.Data)),[],'all')<1e-10);
close_system(model,0);
% Exercise the R2021a export in the installed release, without claiming an
% actual R2021a runtime test.
exported=[model '_R2021a'];
load_system(fullfile(project.simulink,[exported '.slx']));
record([exported '/Twin rotor (simplified model)'],'x','true_state');
input=Simulink.SimulationInput(exported);
input=input.setVariable('meas_on',0,'Workspace',exported); legacy=sim(input);
assert(max(abs(squeeze(legacy.true_state.Data)-squeeze(runs{1}.true_state.Data)),[],'all')<1e-10);
close_system(exported,0);
% Match the older Lab 2 cascade's sensor/derivative conventions in memory.
% This ablation verifies the adaptation without pretending the retained
% professor tachometer filters and numerical derivatives are identical.
for block={'Transfer Fcn1','Transfer Fcn3'}
    set_param([experiment '/' block{1}],'Denominator','1');
end
workspace=get_param(experiment,'ModelWorkspace');
for axis=1:2
    loops={'Subsystem3','Subsystem2'};
    loop=[experiment '/' loops{axis}];
    ports=get_param([loop '/Derivative'],'PortHandles');
    line=get_param(ports.Outport,'Line'); destinations=get_param(line,'DstPortHandle');
    destination=destinations(1); parent=get_param(destination,'Parent');
    port=get_param(destination,'PortNumber');
    delete_line(line);
    delete_line(get_param(ports.Inport,'Line'));
    delete_block([loop '/Derivative']);
    variable=sprintf('ReferenceRate%d',axis);
    assignin(workspace,variable,[referenceRate(:,1) referenceRate(:,axis+1)]);
    add_block('simulink/Sources/From Workspace',[loop '/Derivative'], ...
        'VariableName',variable,'Interpolate','on');
    add_line(loop,'Derivative/1',[get_param(parent,'Name') '/' num2str(port)]);
end
input=Simulink.SimulationInput(experiment);
input=input.setVariable('meas_on',0,'Workspace',experiment); matched=sim(input);
old='aero2_cascade_control'; load_system(fullfile(project.simulink,[old '.slx']));
record([old '/Course nonlinear plant'],'x','true_state');
input=Simulink.SimulationInput(old);
input=input.setVariable('meas_on',0,'Workspace',old); original=sim(input);
delta=max(abs(squeeze(matched.true_state.Data)-squeeze(original.true_state.Data)),[],'all');
assert(delta<1e-7,'Matched professor/original cascade dynamics differ.');
fprintf('Matched state comparison passed: %.3g\n',delta);
close_system(old,0); close_system(experiment,0);
% Reopen the saved experiment to discard temporary comparison changes.
load_system(fullfile(project.experiments,[experiment '.slx']));
load_system(fullfile(project.simulink,[model '.slx']));
stress=saturationTest(project,model,p);
folder=fullfile(project.results,'professor_format'); if ~isfolder(folder), mkdir(folder); end
save(fullfile(folder,'validation.mat'),'metrics','runs','delta','stress');
writetable(metrics,fullfile(folder,'metrics.csv'));
disp(metrics); fprintf('Matched state maximum difference: %.3g\n',delta);
fprintf('Stress peak voltage %.4f V, final speed %.4f rad/s\n',stress.peak,stress.finalSpeed);
export_professor_cascade_figures;
end

function record(system,source,variable)
name=['Record ' variable];
add_block('simulink/Sinks/To Workspace',[system '/' name], ...
    'VariableName',variable,'SaveFormat','Timeseries','MaxDataPoints','inf');
from=get_param([system '/' source],'PortHandles');
to=get_param([system '/' name],'PortHandles');
add_line(system,from.Outport(1),to.Inport(1),'autorouting','on');
end

function stress = saturationTest(project,source,p)
% Isolate a copied IP loop for unreachable-speed and release tests.
m='aero2_professor_motor_saturation_experiment';
if bdIsLoaded(m),close_system(m,0);end
new_system(m);
set_param(m,'SolverType','Fixed-step','Solver','ode1','FixedStep','0.002', ...
    'StopTime','5','ReturnWorkspaceOutputs','on');
w=get_param(m,'ModelWorkspace');
assignin(w,'kp_PI_motor',p.motor.kp); assignin(w,'ki_PI_motor',p.motor.ki);
assignin(w,'kw_PI_motor',p.motor.awGain/p.motor.ki);
assignin(w,'Command',[0 0;0.1 1000;2 1000;2.002 0;5 0]);
add_block([source '/IP_m'],[m '/IP_m'],'Position',[300 180 450 280]);
add_block([source '/+//- 24V1'],[m '/Voltage limit'],'Position',[510 190 560 230]);
add_block('simulink/Sources/From Workspace',[m '/Reference'], ...
    'VariableName','Command','Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value','Position',[30 230 130 260]);
add_block('simulink/Continuous/Transfer Fcn',[m '/Motor'], ...
    'Numerator',num2str(p.motor.inputGain,17), ...
    'Denominator',mat2str([1 p.motor.decay],17),'Position',[620 180 730 230]);
add_block('simulink/Continuous/Transfer Fcn',[m '/Tachometer'], ...
    'Numerator','1','Denominator','[1/180 1]','Position',[580 320 700 360]);
add_line(m,'Reference/1','IP_m/3'); add_line(m,'IP_m/1','Voltage limit/1');
add_line(m,'Voltage limit/1','Motor/1'); add_line(m,'Voltage limit/1','IP_m/1');
add_line(m,'Motor/1','Tachometer/1'); add_line(m,'Tachometer/1','IP_m/2');
record(m,'Voltage limit','voltage'); record(m,'Motor','speed');
save_system(m,fullfile(project.experiments,[m '.slx']));
out=sim(m); stress.time=out.voltage.Time; stress.voltage=out.voltage.Data(:);
stress.speed=out.speed.Data(:); stress.peak=max(abs(stress.voltage));
stress.finalSpeed=stress.speed(end);
assert(stress.peak<=24+1e-12 && any(stress.voltage>23.999));
assert(abs(stress.finalSpeed)<0.01,'IP loop failed to recover after saturation.');
end
