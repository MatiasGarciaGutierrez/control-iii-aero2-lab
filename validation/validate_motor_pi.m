function metrics = validate_motor_pi
%VALIDATE_MOTOR_PI Check each motor and both step signs with the course plant.
project = setup_project; p = motor_pi_parameters;
model = 'aero2_motor_pi_experiment';
load_system(fullfile(project.experiments, [model '.slx']));
% Verify the working copy retains the supplied dynamics and sensor settings.
load_system(fullfile(project.course, 'model_AERO2_simul_R2021a.slx'));
source = 'model_AERO2_simul_R2021a/Twin rotor (simplified model)';
target = [model '/Course nonlinear plant'];
root = sfroot;
original = root.find('-isa', 'Stateflow.EMChart', 'Path', [source '/dynamics']);
copied = root.find('-isa', 'Stateflow.EMChart', 'Path', [target '/dynamics']);
assert(strcmp(original.Script, copied.Script), 'The supplied dynamics changed.');
settings = {'x','InitialCondition'; 'Amplifier','UpperLimit'; ...
    'Amplifier','LowerLimit'; 'quantizer','QuantizationInterval'; ...
    'sensor noise','Variance'; 'sensor noise','Seed'; 'sensor noise','SampleTime'};
for k = 1:size(settings,1)
    assert(isequal(get_param([source '/' settings{k,1}], settings{k,2}), ...
        get_param([target '/' settings{k,1}], settings{k,2})), ...
        'A supplied plant or sensor setting changed.');
end
if ~isfolder(project.results)
    mkdir(project.results);
end
commands = [100 0; -100 0; 0 100; 0 -100; 100 -100; -100 100; 0 0];
metrics = table; runs = cell(size(commands,1),1);
for k = 1:size(commands,1)
    runs{k} = experiment(model, commands(k,:), p.sampleTime);
    for motor = 1:2
        if commands(k,motor) == 0
            assert(max(abs(runs{k}.data(:,motor+2))) < 1e-10, ...
                'An uncommanded motor moved: check channel wiring.');
        else
            row = measure(runs{k}, motor, commands(k,motor), p);
            metrics = [metrics; row]; %#ok<AGROW>
        end
    end
end
fine = experiment(model, [100 -100], p.sampleTime/2);
coarse = runs{5};
for motor = 1:2
    fineRow = measure(fine, motor, commands(5,motor), p);
    coarseRow = measure(coarse, motor, commands(5,motor), p);
    assert(abs(fineRow.SettlingTime-coarseRow.SettlingTime) < 0.01, ...
        'Step-size refinement changes the timing conclusion.');
end
assert(max(abs(runs{7}.data(:))) < 1e-10, 'Zero-reference equilibrium failed.');
writetable(metrics, fullfile(project.results, 'motor_pi_metrics.csv'));
save(fullfile(project.results, 'motor_pi_validation.mat'), 'runs', 'fine', 'metrics', 'p');
disp(metrics);
fprintf('PASS: independent channels, both signs, combined steps, zero equilibrium and half-step check.\n');
end

function r = experiment(model, commands, step)
input = Simulink.SimulationInput(model);
names = {'Main', 'Tail'};
for k = 1:2
    input = input.setBlockParameter([model '/' names{k} ' speed reference'], ...
        'After', num2str(commands(k)));
end
input = input.setModelParameter('FixedStep', num2str(step));
output = sim(input);
r.time = output.motor_data.Time;
if output.motor_data.IsTimeFirst
    r.data = reshape(output.motor_data.Data, numel(r.time), 14);
else
    r.data = reshape(output.motor_data.Data, 14, [])';
end
assert(all(isfinite(r.data(:))), 'A signal became nonfinite.');
end

function row = measure(r, motor, command, p)
selected = r.time >= p.stepTime;
t = r.time(selected)-p.stepTime;
y = r.data(selected, motor+2);
e = command-y;
lastOutside = find(abs(e) > 0.02*abs(command), 1, 'last');
assert(~isempty(lastOutside) && lastOutside < numel(t), 'Response did not settle.');
settling = t(lastOutside+1);
overshoot = max(0, max(sign(command)*y)-abs(command))/abs(command)*100;
finalError = e(end);
voltage = r.data(selected, motor+8);
raw = r.data(selected, motor+6);
peakVoltage = max(abs(voltage));
saturationTime = sum(abs(raw-voltage)>1e-9)*median(diff(t));
% Independent continuous-time PI step solution, including its numerator zero.
expected = command*(1-exp(-p.pole*t).*(1+(p.decay-p.pole)*t));
analyticError = max(abs(y-expected));
assert(settling >= 0.8*p.targetSettlingTime && settling <= 1.2*p.targetSettlingTime, ...
    'Settling time is outside the agreed +/-20%% planning tolerance.');
assert(abs(finalError) < 0.01*abs(command), 'Final error is too large.');
assert(peakVoltage <= p.voltageLimit+1e-9 && saturationTime == 0, ...
    'Nominal speed step saturates the actuator.');
assert(analyticError < 1.5, 'Response disagrees with the independent PI solution.');
row = table(motor, command, settling, overshoot, finalError, peakVoltage, ...
    saturationTime, analyticError, 'VariableNames', ...
    {'Motor','Command','SettlingTime','OvershootPercent','FinalError', ...
    'PeakVoltage','SaturationTime','AnalyticMaxError'});
end
