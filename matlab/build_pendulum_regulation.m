function model = build_pendulum_regulation
%BUILD_PENDULUM_REGULATION Part I, Question 1(c): critically damped regulation.
project = setup_project;
source = 'part1_pendulum_linearization';
model = 'part1_pendulum_regulation';
if bdIsLoaded(model), close_system(model, 0); end
load_system(fullfile(project.simulink, [source '.slx']));
save_system(source, fullfile(project.simulink, [model '.slx']));
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'Kp', 36);
assignin(workspace, 'Kd', 12);
% Replace the virtual acceleration source with state feedback; preserve plant
% and nonlinear cancellation subsystems from Question 1(b).
ports = get_param([model '/Virtual acceleration v'], 'PortHandles');
delete_line(get_param(ports.Outport(1), 'Line'));
delete_block([model '/Virtual acceleration v']);
feedback = [model '/Linear state feedback'];
add_block('built-in/Subsystem', feedback, 'Position', [55 170 195 285]);
add_block('simulink/Ports & Subsystems/In1', [feedback '/theta'], ...
    'Port', '1', 'Position', [35 65 65 85]);
add_block('simulink/Ports & Subsystems/In1', [feedback '/omega'], ...
    'Port', '2', 'Position', [35 165 65 185]);
add_block('simulink/Math Operations/Gain', [feedback '/Position feedback'], ...
    'Gain', '-Kp', 'Position', [140 55 220 95]);
add_block('simulink/Math Operations/Gain', [feedback '/Rate feedback'], ...
    'Gain', '-Kd', 'Position', [140 155 220 195]);
add_block('simulink/Math Operations/Sum', [feedback '/Sum'], ...
    'Inputs', '++', 'Position', [305 100 335 150]);
add_block('simulink/Ports & Subsystems/Out1', [feedback '/v'], ...
    'Port', '1', 'Position', [415 115 445 135]);
connections = {'theta/1','Position feedback/1'; 'omega/1','Rate feedback/1'; ...
    'Position feedback/1','Sum/1'; 'Rate feedback/1','Sum/2'; 'Sum/1','v/1'};
for index = 1:size(connections,1)
    add_line(feedback, connections{index,1}, connections{index,2}, 'autorouting', 'on');
end
connections = {'Nonlinear pendulum/1','Linear state feedback/1'; ...
    'Nonlinear pendulum/2','Linear state feedback/2'; ...
    'Linear state feedback/1','Linearizing controller/1'; ...
    'Linear state feedback/1','Acceleration scope/2'};
for index = 1:size(connections,1)
    add_line(model, connections{index,1}, connections{index,2}, 'autorouting', 'on');
end
ports = get_param(feedback, 'PortHandles');
set_param(get_param(ports.Outport(1), 'Line'), 'Name', 'v');
notes = find_system(model, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation');
delete(notes);
note = Simulink.Annotation(model, sprintf([ ...
    'Part I - Question 1(c): regulation to upright\n' ...
    'v = -36*theta - 12*omega; critically damped poles at -6 rad/s\n' ...
    'theta0 = 0.1 rad, omega0 = 0; expected 2%% settling time = 0.9723 s\n' ...
    'Exact cancellation requires cos(theta) nonzero and ideal input authority.']));
note.Position = [55 20 1100 105];
save_system(model);
Simulink.exportToVersion(model, ...
    fullfile(project.simulink, [model '_R2021a.slx']), 'R2021a');
end
