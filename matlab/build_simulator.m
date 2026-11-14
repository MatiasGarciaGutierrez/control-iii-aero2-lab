function model = build_simulator
%BUILD_SIMULATOR Create an open-loop working model from the course plant.
project = setup_project;
load_system('simulink');
sourceFile = fullfile(project.course, 'model_AERO2_simul_R2021a.slx');
source = 'model_AERO2_simul_R2021a';
model = 'aero2_simulator';
load_system(sourceFile);
if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);
% Defaults live in the model, so a clean workspace needs no missing callback.
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'meas_on', 0);
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode1', ...
    'FixedStep', '0.002', 'StopTime', '1', ...
    'ReturnWorkspaceOutputs', 'on', 'SaveOutput', 'off', ...
    'SaveTime', 'off', 'SignalLogging', 'off', ...
    'Location', [30 80 1380 850]);
add_block([source '/Twin rotor (simplified model)'], ...
    [model '/Course nonlinear plant'], 'Position', [470 160 790 420]);

names = {'Main voltage input', 'Tail voltage input'};
amplitudes = {'1', '-1'};
limits = {'Main voltage limit', 'Tail voltage limit'};
for k = 1:2
    y = 160 + (k-1)*160;
    add_block('simulink/Sources/Pulse Generator', [model '/' names{k}], ...
        'PulseType', 'Time based', 'Amplitude', amplitudes{k}, ...
        'Period', '10', 'PulseWidth', '2', 'PhaseDelay', '0.1', ...
        'SampleTime', '0', 'Position', [65 y 145 y+55]);
    add_block('simulink/Discontinuities/Saturation', [model '/' limits{k}], ...
        'UpperLimit', '24', 'LowerLimit', '-24', ...
        'Position', [270 y 330 y+55]);
    add_line(model, [names{k} '/1'], [limits{k} '/1'], 'autorouting', 'on');
    line = add_line(model, [limits{k} '/1'], ...
        sprintf('Course nonlinear plant/%d', k), 'autorouting', 'on');
    labels = {'main voltage [V]', 'tail voltage [V]'};
    set_param(line, 'Name', labels{k});
end

scopeNames = {'Voltage scope', 'Rotor speed scope', 'Body angle scope'};
units = {'Voltage [V]', 'Speed [rad/s]', 'Angle [rad]'};
positions = {[270 520 365 580], [960 320 1060 390], [960 155 1060 265]};
for k = 1:3
    add_block('simulink/Sinks/Scope', [model '/' scopeNames{k}], ...
        'NumInputPorts', '2', 'Position', positions{k});
    config = get_param([model '/' scopeNames{k}], 'ScopeConfiguration');
    config.ShowLegend = true;
    config.ShowGrid = true;
    config.TimeSpan = '1';
    config.YLabel = units{k};
end
add_line(model, 'Main voltage limit/1', 'Voltage scope/1', 'autorouting', 'on');
add_line(model, 'Tail voltage limit/1', 'Voltage scope/2', 'autorouting', 'on');
ports = [5 6 1 3 2 4];
labels = {'main speed [rad/s]', 'tail speed [rad/s]', ...
    'yaw [rad]', 'pitch [rad]', 'yaw rate [rad/s]', 'pitch rate [rad/s]'};
add_block('simulink/Signal Routing/Mux', [model '/Log signals'], ...
    'Inputs', '8', 'Position', [1190 450 1195 650]);
add_block('simulink/Sinks/To Workspace', [model '/Recorded signals'], ...
    'VariableName', 'simulator_data', 'SaveFormat', 'Timeseries', ...
    'SampleTime', '0.002', 'MaxDataPoints', 'inf', ...
    'Position', [1250 535 1380 575]);
add_line(model, 'Main voltage limit/1', 'Log signals/1', 'autorouting', 'on');
add_line(model, 'Tail voltage limit/1', 'Log signals/2', 'autorouting', 'on');
for k = 1:6
    line = add_line(model, sprintf('Course nonlinear plant/%d', ports(k)), ...
        sprintf('Log signals/%d', k+2), 'autorouting', 'on');
    set_param(line, 'Name', labels{k});
end
add_line(model, 'Course nonlinear plant/5', 'Rotor speed scope/1', 'autorouting', 'on');
add_line(model, 'Course nonlinear plant/6', 'Rotor speed scope/2', 'autorouting', 'on');
add_line(model, 'Course nonlinear plant/1', 'Body angle scope/1', 'autorouting', 'on');
add_line(model, 'Course nonlinear plant/3', 'Body angle scope/2', 'autorouting', 'on');
add_line(model, 'Log signals/1', 'Recorded signals/1', 'autorouting', 'on');
add_block('simulink/Sinks/Terminator', [model '/Unused current'], ...
    'Position', [840 435 860 455]);
add_line(model, 'Course nonlinear plant/7', 'Unused current/1', 'autorouting', 'on');
note = Simulink.Annotation(model, sprintf([ ...
    'Open-loop simulator - no controller\n' ...
    'Main +1 V, tail -1 V from 0.1 to 0.3 s; stop at 1 s\n' ...
    'Euler integration: 0.002 s. Ideal sensors by default.\n' ...
    'The course plant and its internal amplifier/sensors are copied unchanged.']));
note.Position = [60 40 1050 120];
if ~isfolder(project.simulink)
    mkdir(project.simulink);
end
save_system(model, fullfile(project.simulink, [model '.slx']));
Simulink.exportToVersion(model, ...
    fullfile(project.simulink, [model '_R2021a.slx']), 'R2021a');
end
