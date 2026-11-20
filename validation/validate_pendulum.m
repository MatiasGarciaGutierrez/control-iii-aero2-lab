function metrics = validate_pendulum
%VALIDATE_PENDULUM Compare the block model with independent ODE integration.
project = setup_project;
model = 'part1_pendulum';
load_system(fullfile(project.simulink, [model '.slx']));
% Instrument in memory only; the delivered answer model has basic scopes.
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
sources = {'Angle', 'Angular velocity', 'Angular acceleration'};
for index = 1:numel(sources)
    ports = get_param([model '/' sources{index}], 'PortHandles');
    set_param(ports.Outport(1), 'DataLogging', 'on');
end
cases = [0 0 0; 0.1 0 0; -0.1 0 0; 0 0 0.02; 0 0 -0.02];
metrics = table;
runs = cell(size(cases,1),1);
for index = 1:size(cases,1)
    input = Simulink.SimulationInput(model);
    input = input.setVariable('theta0', cases(index,1), 'Workspace', model);
    input = input.setVariable('omega0', cases(index,2), 'Workspace', model);
    input = input.setBlockParameter([model '/Control input u'], ...
        'Value', num2str(cases(index,3)));
    input = input.setModelParameter('SignalLogging', 'on', ...
        'SignalLoggingName', 'logsout');
    output = sim(input);
    angle = output.logsout.get('theta').Values;
    rate = output.logsout.get('omega').Values;
    acceleration = output.logsout.get('alpha').Values;
    time = angle.Time;
    state = [angle.Data(:), rate.Data(:)];
    assert(all(isfinite(state(:))), 'A state became nonfinite.');
    command = cases(index,3);
    % Literal physical constants make this independent of block parameters.
    rhs = @(~,x) [x(2); 19.62*sin(x(1))-x(2)+20*cos(x(1))*command];
    reference = ode45(rhs, [0 2], cases(index,1:2)', ...
        odeset('RelTol', 1e-11, 'AbsTol', 1e-13));
    expected = deval(reference, time)';
    odeError = max(max(abs(state-expected)));
    assert(odeError < 2e-6, 'Block model disagrees with independent ODE.');
    expectedAcceleration = 19.62*sin(state(:,1))-state(:,2) ...
        +20*cos(state(:,1))*command;
    accelerationError = max(abs(acceleration.Data(:)-expectedAcceleration));
    assert(accelerationError < 1e-10, 'Acceleration equation is inconsistent.');
    energy = 0.025*state(:,2).^2+0.981*cos(state(:,1));
    if command == 0
        assert(max(diff(energy)) < 1e-8, 'Unforced mechanical energy increased.');
    end
    if index == 1
        assert(max(abs(state(:))) < 1e-12, 'Upright equilibrium moved.');
    end
    runs{index} = struct('time', time, 'state', state, ...
        'acceleration', acceleration.Data(:), 'energy', energy);
    row = table(cases(index,1), command, odeError, accelerationError, ...
        state(end,1), state(end,2), 'VariableNames', ...
        {'InitialAngle','Input','MaxStateError','AccelerationError', ...
        'FinalAngle','FinalRate'});
    metrics = [metrics; row]; %#ok<AGROW>
end
assert(runs{2}.state(end,1) > 1, 'Small tilt did not depart from upright.');
for pair = [2 4]
    opposite = interp1(runs{pair+1}.time, runs{pair+1}.state, runs{pair}.time);
    assert(max(max(abs(runs{pair}.state+opposite))) < 2e-6, ...
        'Sign-reversed trajectories lost symmetry.');
end
% Execute the exported model in the installed release, not in R2021a.
exportModel = [model '_R2021a'];
load_system(fullfile(project.simulink, [exportModel '.slx']));
exportCleanup = onCleanup(@() close_system(exportModel, 0)); %#ok<NASGU>
for index = 1:2
    ports = get_param([exportModel '/' sources{index}], 'PortHandles');
    set_param(ports.Outport(1), 'DataLogging', 'on');
end
input = Simulink.SimulationInput(exportModel);
input = input.setModelParameter('SignalLogging', 'on', 'SignalLoggingName', 'logsout');
output = sim(input);
angle = output.logsout.get('theta').Values;
rate = output.logsout.get('omega').Values;
assert(isequal(angle.Time, runs{2}.time), 'Export changed solver output times.');
exportError = max(max(abs([angle.Data(:) rate.Data(:)]-runs{2}.state)));
assert(exportError < 1e-10, 'Export changed the default dynamics.');
if ~isfolder(project.results)
    mkdir(project.results);
end
save(fullfile(project.results, 'pendulum_validation.mat'), ...
    'metrics', 'runs', 'exportError');
writetable(metrics, fullfile(project.results, 'pendulum_metrics.csv'));
disp(metrics);
fprintf('PASS: ODE comparison, equilibrium, energy, symmetry and exported dynamics.\n');
end
