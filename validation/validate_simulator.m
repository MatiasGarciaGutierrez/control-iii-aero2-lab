function checks = validate_simulator
%VALIDATE_SIMULATOR Test initialization, input wiring and course motor dynamics.
project = setup_project;
model = 'aero2_simulator';
load_system(fullfile(project.simulink, [model '.slx']));
if ~isfolder(project.results)
    mkdir(project.results);
end
checks = table;
load_system(fullfile(project.course, 'model_AERO2_simul_R2021a.slx'));
original = 'model_AERO2_simul_R2021a/Twin rotor (simplified model)';
copied = [model '/Course nonlinear plant'];
stateflow = sfroot;
sourceChart = stateflow.find('-isa', 'Stateflow.EMChart', 'Path', [original '/dynamics']);
targetChart = stateflow.find('-isa', 'Stateflow.EMChart', 'Path', [copied '/dynamics']);
assert(strcmp(sourceChart.Script, targetChart.Script), 'The course dynamics were changed.');
parameters = {'x', 'InitialCondition'; 'Amplifier', 'UpperLimit'; ...
    'Amplifier', 'LowerLimit'; 'quantizer', 'QuantizationInterval'; ...
    'sensor noise', 'Variance'; 'sensor noise', 'Seed'; 'sensor noise', 'SampleTime'};
for k = 1:size(parameters,1)
    before = get_param([original '/' parameters{k,1}], parameters{k,2});
    after = get_param([copied '/' parameters{k,1}], parameters{k,2});
    assert(isequal(before, after), 'Course plant configuration changed.');
end
checks = record(checks, 'Unchanged dynamics and sensor configuration', 1);
connections = {'Voltage scope', 1, 'Main voltage limit', 1; ...
    'Voltage scope', 2, 'Tail voltage limit', 1; ...
    'Rotor speed scope', 1, 'Course nonlinear plant', 5; ...
    'Rotor speed scope', 2, 'Course nonlinear plant', 6; ...
    'Body angle scope', 1, 'Course nonlinear plant', 1; ...
    'Body angle scope', 2, 'Course nonlinear plant', 3};
for k = 1:size(connections,1)
    handles = get_param([model '/' connections{k,1}], 'PortHandles');
    line = get_param(handles.Inport(connections{k,2}), 'Line');
    sourcePort = get_param(line, 'SrcPortHandle');
    assert(strcmp(get_param(sourcePort, 'Parent'), [model '/' connections{k,3}]), ...
        'A scope is connected to the wrong block.');
    assert(get_param(sourcePort, 'PortNumber') == connections{k,4}, ...
        'A scope is connected to the wrong output channel.');
end
checks = record(checks, 'Verified scope input channels', size(connections,1));

zero = experiment(model, 0, 0, 0);
assert(max(abs(zero.data(:))) < 1e-10, 'Zero-voltage equilibrium failed.');
checks = record(checks, 'Zero-voltage equilibrium', max(abs(zero.data(:))));
main = experiment(model, 1, 0, 0);
tail = experiment(model, 0, -1, 0);
pulse = experiment(model, 1, -1, 0);
runs = {main, tail, pulse}; names = {'Main only', 'Tail only', 'Both motors'};
for k = 1:3
    r = runs{k};
    u = [double(k~=2), -double(k~=1)];
    expectedVoltage = (r.time>=0.1 & r.time<0.3).*u;
    inputError = max(max(abs(r.data(:,1:2)-expectedVoltage)));
    assert(inputError < 1e-10, 'Voltage pulse timing or sign is incorrect.');
    response = (96.532/5.78)*(1-exp(-5.78*max(r.time-0.1,0)));
    after = r.time>=0.3;
    response(after) = (96.532/5.78)*(1-exp(-5.78*0.2))*exp(-5.78*(r.time(after)-0.3));
    speedError = max(max(abs(r.data(:,3:4)-response.*u)));
    assert(speedError < 0.15, 'Motor response does not match the course equation.');
    checks = record(checks, [names{k} ' voltage error [V]'], inputError);
    checks = record(checks, [names{k} ' speed error [rad/s]'], speedError);
end
limited = experiment(model, 30, -30, 0);
peakVoltage = max(max(abs(limited.data(:,1:2))));
assert(abs(peakVoltage-24) < 1e-10, 'Amplifier limits are incorrect.');
checks = record(checks, 'Saturated peak voltage [V]', peakVoltage);
noisy = experiment(model, 0, 0, 1);
assert(any(abs(noisy.data(:,3:8)) > 0, 'all'), 'Sensor effects were not enabled.');
checks = record(checks, 'Noisy zero-input finite samples', numel(noisy.time));

writetable(checks, fullfile(project.results, 'simulator_checks.csv'));
save(fullfile(project.results, 'simulator_validation.mat'), ...
    'zero', 'main', 'tail', 'pulse', 'limited', 'noisy', 'checks');
disp(checks);
fprintf('PASS: simulator initialization, both motor channels, pulse timing, voltage limits and sensor toggle.\n');
end

function result = experiment(model, mainVoltage, tailVoltage, noise)
fprintf('Simulating main=%g V, tail=%g V, sensor effects=%g ...\n', ...
    mainVoltage, tailVoltage, noise);
input = Simulink.SimulationInput(model);
input = input.setBlockParameter([model '/Main voltage input'], 'Amplitude', num2str(mainVoltage));
input = input.setBlockParameter([model '/Tail voltage input'], 'Amplitude', num2str(tailVoltage));
input = input.setVariable('meas_on', noise, 'Workspace', model);
output = sim(input);
result.time = output.simulator_data.Time;
if output.simulator_data.IsTimeFirst
    result.data = reshape(output.simulator_data.Data, numel(result.time), 8);
else
    result.data = reshape(output.simulator_data.Data, 8, [])';
end
assert(all(isfinite(result.data(:))), 'Simulation produced nonfinite values.');
end

function checks = record(checks, name, value)
row = table({name}, value, 'VariableNames', {'Check', 'Observed'});
checks = [checks; row];
end
