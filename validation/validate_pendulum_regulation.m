function metrics = validate_pendulum_regulation
%VALIDATE_PENDULUM_REGULATION Check regulation and exact transient response.
project = setup_project; model = 'part1_pendulum_regulation';
cases = [0.1 0; -0.1 0; 0.5 0; 1.3 0; -1.3 0; 0.1 0.1; 0 0];
targetSettling = fzero(@(z) (1+z)*exp(-z)-0.02, [5 7])/6;
runs = cell(size(cases,1),1); metrics = table;
for index = 1:size(cases,1)
    run = simulate(project, model, cases(index,:));
    c = cases(index,2)+6*cases(index,1);
    expectedAngle = (cases(index,1)+c*run.time).*exp(-6*run.time);
    expectedRate = (cases(index,2)-6*c*run.time).*exp(-6*run.time);
    angleError = max(abs(run.theta-expectedAngle));
    rateError = max(abs(run.omega-expectedRate));
    accelerationError = max(abs(run.alpha-run.v));
    assert(angleError < 2e-7 && rateError < 2e-6, ...
        'Regulated plant disagrees with the analytical solution.');
    assert(accelerationError < 1e-10, 'Nonlinear cancellation failed.');
    assert(max(abs(run.v+36*run.theta+12*run.omega)) < 1e-10, ...
        'State feedback law is inconsistent.');
    assert(min(abs(cos(run.theta))) > 0.2, 'Trajectory approached singularity.');
    settling = NaN;
    if cases(index,1) ~= 0
        outside = find(abs(run.theta) > 0.02*abs(cases(index,1)), 1, 'last');
        assert(~isempty(outside) && outside < numel(run.time), 'Response did not settle.');
        settling = run.time(outside+1);
        if cases(index,2) == 0
            assert(abs(settling-targetSettling) < 0.006, ...
                '2%% settling time disagrees with the exact prediction.');
            assert(max(sign(cases(index,1))*run.theta) <= abs(cases(index,1))+1e-9 ...
                && min(sign(cases(index,1))*run.theta) >= -1e-9, ...
                'Zero-rate response overshot or crossed the origin.');
        end
    else
        assert(max(abs(run.theta))+max(abs(run.omega)) < 1e-12, 'Origin moved.');
    end
    runs{index} = run;
    row = table(cases(index,1), cases(index,2), angleError, rateError, ...
        accelerationError, settling, max(abs(run.u)), ...
        'VariableNames', {'InitialAngle','InitialRate','AngleError','RateError', ...
        'AccelerationError','SettlingTime2Percent','PeakInput'});
    metrics = [metrics; row]; %#ok<AGROW>
end
exported = simulate(project, [model '_R2021a'], cases(1,:));
assert(isequal(exported.time, runs{1}.time), 'Export changed solver output times.');
assert(max(abs(exported.theta-runs{1}.theta)) < 1e-10, 'Export changed trajectory.');
if ~isfolder(project.results), mkdir(project.results); end
save(fullfile(project.results, 'pendulum_regulation_validation.mat'), ...
    'runs', 'metrics', 'targetSettling');
writetable(metrics, fullfile(project.results, 'pendulum_regulation_metrics.csv'));
disp(metrics);
fprintf('PASS: exact response, 2%% settling, no overshoot, origin and export.\n');
end

function run = simulate(project, model, initial)
load_system(fullfile(project.simulink, [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
sources = {'Nonlinear pendulum','Nonlinear pendulum','Nonlinear pendulum', ...
    'Linearizing controller','Linear state feedback'};
numbers = [1 2 3 1 1]; names = {'theta','omega','alpha','u','v'};
for index = 1:numel(sources)
    ports = get_param([model '/' sources{index}], 'PortHandles');
    set_param(ports.Outport(numbers(index)), 'DataLogging', 'on');
end
input = Simulink.SimulationInput(model);
input = input.setVariable('theta0', initial(1), 'Workspace', model);
input = input.setVariable('omega0', initial(2), 'Workspace', model);
input = input.setModelParameter('SignalLogging', 'on', 'SignalLoggingName', 'logsout');
output = sim(input);
for index = 1:numel(names)
    signal = output.logsout.get(names{index}).Values;
    run.(names{index}) = signal.Data(:);
    assert(all(isfinite(signal.Data(:))), 'Nonfinite simulated signal.');
end
run.time = signal.Time;
end
