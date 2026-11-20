function metrics = validate_pendulum_linearization
%VALIDATE_PENDULUM_LINEARIZATION Verify exact double-integrator behavior.
project = setup_project;
model = 'part1_pendulum_linearization';
% Initial angle, initial rate, virtual acceleration after 0.2 seconds.
cases = [0.1 0 0.2; 0.1 0 -0.2; 0.1 0.05 0; 1.3 0 0; -1.3 0 0];
metrics = table; runs = cell(size(cases,1),1);
for index = 1:size(cases,1)
    run = simulate(project, model, cases(index,:));
    elapsed = max(run.time-0.2, 0);
    expectedAngle = cases(index,1)+cases(index,2)*run.time ...
        +0.5*cases(index,3)*elapsed.^2;
    expectedRate = cases(index,2)+cases(index,3)*elapsed;
    angleError = max(abs(run.theta-expectedAngle));
    rateError = max(abs(run.omega-expectedRate));
    accelerationError = max(abs(run.alpha-run.v));
    minimumCosine = min(abs(cos(run.theta)));
    assert(angleError < 2e-7 && rateError < 2e-7, ...
        'Plant does not follow analytical double-integrator motion.');
    assert(accelerationError < 1e-10, 'Nonlinear cancellation failed.');
    assert(minimumCosine > 0.2, 'Validation trajectory approached singularity.');
    runs{index} = run;
    row = table(cases(index,1), cases(index,2), cases(index,3), ...
        angleError, rateError, accelerationError, minimumCosine, max(abs(run.u)), ...
        'VariableNames', {'InitialAngle','InitialRate','VirtualAcceleration', ...
        'AngleError','RateError','AccelerationError','MinimumAbsCosine','PeakInput'});
    metrics = [metrics; row]; %#ok<AGROW>
end
exported = simulate(project, [model '_R2021a'], cases(1,:));
assert(isequal(exported.time, runs{1}.time), 'Export changed simulation times.');
assert(max(abs(exported.theta-runs{1}.theta)) < 1e-10, ...
    'Export changed the default trajectory.');
if ~isfolder(project.results), mkdir(project.results); end
save(fullfile(project.results, 'pendulum_linearization_validation.mat'), 'runs', 'metrics');
writetable(metrics, fullfile(project.results, 'pendulum_linearization_metrics.csv'));
disp(metrics);
fprintf('PASS: signed inputs, initial rate, finite-angle holds and exported trajectory.\n');
end

function run = simulate(project, model, settings)
load_system(fullfile(project.simulink, [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
sources = {'Nonlinear pendulum', 'Nonlinear pendulum', ...
    'Nonlinear pendulum', 'Linearizing controller', 'Virtual acceleration v'};
numbers = [1 2 3 1 1]; names = {'theta','omega','alpha','u','v'};
for index = 1:numel(sources)
    ports = get_param([model '/' sources{index}], 'PortHandles');
    set_param(ports.Outport(numbers(index)), 'DataLogging', 'on');
end
input = Simulink.SimulationInput(model);
input = input.setVariable('theta0', settings(1), 'Workspace', model);
input = input.setVariable('omega0', settings(2), 'Workspace', model);
input = input.setBlockParameter([model '/Virtual acceleration v'], ...
    'After', num2str(settings(3)));
input = input.setModelParameter('SignalLogging', 'on', 'SignalLoggingName', 'logsout');
output = sim(input);
for index = 1:numel(names)
    signal = output.logsout.get(names{index}).Values;
    run.(names{index}) = signal.Data(:);
    assert(all(isfinite(signal.Data(:))), 'Nonfinite simulation signal.');
end
run.time = signal.Time;
end
