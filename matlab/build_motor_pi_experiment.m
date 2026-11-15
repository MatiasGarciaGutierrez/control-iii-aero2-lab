function model = build_motor_pi_experiment
%BUILD_MOTOR_PI_EXPERIMENT Create Question 1 speed loops around the unchanged course plant.
project = setup_project;
if ~isfolder(project.experiments), mkdir(project.experiments); end
p = motor_pi_parameters;
load_system('simulink');
source = 'model_AERO2_simul_R2021a';
load_system(fullfile(project.course, [source '.slx']));
model = 'aero2_motor_pi_experiment';
if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'meas_on', 0);
assignin(workspace, 'Kp', p.kp);
assignin(workspace, 'Ki', p.ki);
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode1', ...
    'FixedStep', num2str(p.sampleTime), 'StopTime', num2str(p.stopTime), ...
    'ReturnWorkspaceOutputs', 'on', 'SaveOutput', 'off', ...
    'SaveTime', 'off', 'SignalLogging', 'off');
add_block([source '/Twin rotor (simplified model)'], ...
    [model '/Course nonlinear plant'], 'Position', [850 210 1110 460]);
channels = {'Main', 'Tail'};
for k = 1:2
    n = channels{k}; y = 180+(k-1)*260;
    add_block('simulink/Sources/Step', [model '/' n ' speed reference'], ...
        'Time', num2str(p.stepTime), 'Before', '0', ...
        'After', num2str(p.reference(k)), 'SampleTime', '0', ...
        'Position', [40 y 90 y+40]);
    add_block('simulink/Math Operations/Sum', [model '/' n ' speed error'], ...
        'Inputs', '+-', 'Position', [175 y 200 y+40]);
    add_block('simulink/Math Operations/Gain', [model '/' n ' proportional'], ...
        'Gain', 'Kp', 'Position', [280 y-30 345 y+10]);
    add_block('simulink/Math Operations/Gain', [model '/' n ' integral gain'], ...
        'Gain', 'Ki', 'Position', [280 y+60 345 y+100]);
    add_block('simulink/Continuous/Integrator', [model '/' n ' integral state'], ...
        'InitialCondition', '0', 'Position', [410 y+60 450 y+100]);
    add_block('simulink/Math Operations/Sum', [model '/' n ' raw voltage'], ...
        'Inputs', '++', 'Position', [530 y 555 y+40]);
    add_block('simulink/Discontinuities/Saturation', [model '/' n ' voltage limit'], ...
        'UpperLimit', num2str(p.voltageLimit), 'LowerLimit', num2str(-p.voltageLimit), ...
        'Position', [660 y 720 y+40]);
    wire(model, [n ' speed reference/1'], [n ' speed error/1']);
    wire(model, [n ' speed error/1'], [n ' proportional/1']);
    wire(model, [n ' speed error/1'], [n ' integral gain/1']);
    wire(model, [n ' integral gain/1'], [n ' integral state/1']);
    wire(model, [n ' proportional/1'], [n ' raw voltage/1']);
    wire(model, [n ' integral state/1'], [n ' raw voltage/2']);
    wire(model, [n ' raw voltage/1'], [n ' voltage limit/1']);
    wire(model, [n ' voltage limit/1'], sprintf('Course nonlinear plant/%d', k));
    wire(model, sprintf('Course nonlinear plant/%d', k+4), [n ' speed error/2']);
end
scopeNames = {'Speed scope', 'Error scope', 'Voltage scope'};
units = {'Speed [rad/s]', 'Error [rad/s]', 'Applied voltage [V]'};
for k = 1:3
    y = 100+130*(k-1);
    add_block('simulink/Sinks/Scope', [model '/' scopeNames{k}], ...
        'NumInputPorts', num2str(2+2*(k==1)), 'Position', [1240 y 1320 y+65]);
    config = get_param([model '/' scopeNames{k}], 'ScopeConfiguration');
    config.ShowLegend = true; config.ShowGrid = true;
    config.TimeSpan = '1'; config.YLabel = units{k};
end
% Log order: references, speeds, errors, raw voltages, applied voltages,
% integral contributions, then yaw/pitch. Each pair is main/tail except angles.
add_block('simulink/Signal Routing/Mux', [model '/Log signals'], ...
    'Inputs', '14', 'Position', [1440 140 1445 560]);
add_block('simulink/Sinks/To Workspace', [model '/Recorded signals'], ...
    'VariableName', 'motor_data', 'SaveFormat', 'Timeseries', ...
    'SampleTime', '-1', 'MaxDataPoints', 'inf', 'Position', [1500 320 1630 360]);
for k = 1:2
    n = channels{k};
    sources = {[n ' speed reference/1'], sprintf('Course nonlinear plant/%d', k+4), ...
        [n ' speed error/1'], [n ' raw voltage/1'], [n ' voltage limit/1'], ...
        [n ' integral state/1']};
    for j = 1:6
        add_line(model, sources{j}, sprintf('Log signals/%d', 2*(j-1)+k), 'autorouting', 'on');
    end
    wire(model, sources{1}, sprintf('Speed scope/%d', 2*k-1));
    wire(model, sources{2}, sprintf('Speed scope/%d', 2*k));
    wire(model, sources{3}, sprintf('Error scope/%d', k));
    wire(model, sources{5}, sprintf('Voltage scope/%d', k));
end
wire(model, 'Course nonlinear plant/1', 'Log signals/13');
wire(model, 'Course nonlinear plant/3', 'Log signals/14');
wire(model, 'Log signals/1', 'Recorded signals/1');
for k = [2 4 7]
    name = sprintf('Unused output %d', k);
    add_block('simulink/Sinks/Terminator', [model '/' name], ...
        'Position', [1150 500+25*k 1170 520+25*k]);
    wire(model, sprintf('Course nonlinear plant/%d', k), [name '/1']);
end
note = Simulink.Annotation(model, sprintf([ ...
    'Question 1: motor-speed PI control (no position controller)\n' ...
    'Kp = %.8f, Ki = %.8f; ideal sensors; Euler step 0.002 s\n' ...
    'Main +100 rad/s, tail -100 rad/s at 0.1 s; voltage limits +/-24 V\n' ...
    'Continuous PI: v_raw = Kp*(w_d-w) + integral(Ki*(w_d-w))'], p.kp, p.ki));
note.Position = [40 25 1100 120];
save_system(model, fullfile(project.experiments, [model '.slx']));
Simulink.exportToVersion(model, ...
    fullfile(project.experiments, [model '_R2021a.slx']), 'R2021a');
end

function wire(model, source, destination)
add_line(model, source, destination, 'autorouting', 'on');
end
