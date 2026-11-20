function metrics = validate_dc_motor
%VALIDATE_DC_MOTOR Check the physical plant and its output linearization.
project = setup_project;
plant = 'part1_dc_motor'; controlled = 'part1_dc_motor_linearization';
% Both models must retain the same physical equations and integrator settings.
load_system(fullfile(project.simulink,[plant '.slx']));
load_system(fullfile(project.simulink,[controlled '.slx']));
for name = {'Mechanical equation','Electrical equation','Jerk equation'}
    block = ['/Nonlinear DC motor/' name{1}];
    assert(strcmp(get_param([plant block],'Expr'),get_param([controlled block],'Expr')), ...
        'Linearization changed a physical plant equation.');
end
close_system(plant,0); close_system(controlled,0);
plantCases = [0 0 0 0; 0 0 0 1; 0 0 0 -1; 0.1 -0.2 0.1 0];
plantRuns = cell(size(plantCases,1),1); plantErrors = zeros(size(plantCases,1),1);
for index = 1:size(plantCases,1)
    initial = plantCases(index,1:3); command = plantCases(index,4);
    run = simulate(project,plant,initial,'Voltage input u',command);
    rhs = @(t,x) [x(2);20*x(3)-x(2)-0.5*x(2)*abs(x(2)); ...
        -4*x(2)-40*x(3)+20*command*(t>=0.2)];
    reference = ode45(rhs,[0 2],initial', ...
        odeset('RelTol',1e-11,'AbsTol',1e-13,'MaxStep',0.001));
    expected = deval(reference,run.time)';
    plantErrors(index) = max(max(abs([run.theta run.omega run.ia]-expected)));
    assert(plantErrors(index)<2e-6,'Physical plant differs from independent ODE.');
    plantRuns{index} = run;
end
assert(max(abs(plantRuns{1}.theta))+max(abs(plantRuns{1}.ia))<1e-12, ...
    'Rest equilibrium moved.');
% Initial angle, speed, current and virtual jerk after 0.2 seconds.
cases = [0 0 0 1; 0 0 0 -1; 0.1 0.2 0.011 0; ...
    0.1 -0.2 -0.009 1; -0.1 0.2 0.009 -1; 0.1 0 0.05 0; 0 0 0 0];
runs = cell(size(cases,1),1); metrics = table;
for index = 1:size(cases,1)
    initial = cases(index,1:3); command = cases(index,4);
    run = simulate(project,controlled,initial,'Virtual jerk v',command);
    alpha0 = 20*initial(3)-initial(2)-0.5*initial(2)*abs(initial(2));
    elapsed = max(run.time-0.2,0);
    theta = initial(1)+initial(2)*run.time+alpha0/2*run.time.^2+command/6*elapsed.^3;
    omega = initial(2)+alpha0*run.time+command/2*elapsed.^2;
    alpha = alpha0+command*elapsed;
    current = (alpha+omega+0.5*omega.*abs(omega))/20;
    errors = max(abs([run.theta-theta,run.omega-omega,run.alpha-alpha,run.ia-current]),[],1);
    jerkError = max(abs(run.jerk-run.v));
    assert(max(errors)<2e-7,'Plant does not follow analytical triple-integrator motion.');
    assert(jerkError<1e-9,'Actual jerk does not match the virtual input.');
    if index == 4 || index == 5
        assert(min(run.omega)<0 && max(run.omega)>0,'Test did not cross zero speed.');
    end
    runs{index} = run;
    row = table(initial(1),initial(2),initial(3),command,errors(1),errors(2), ...
        errors(3),errors(4),jerkError,max(abs(run.u)), ...
        'VariableNames',{'InitialAngle','InitialSpeed','InitialCurrent','VirtualJerk', ...
        'AngleError','SpeedError','AccelerationError','CurrentError','JerkError','PeakVoltage'});
    metrics = [metrics;row]; %#ok<AGROW>
end
exportedPlant = simulate(project,[plant '_R2021a'],plantCases(2,1:3),'Voltage input u',1);
compare(exportedPlant,plantRuns{2});
exportedLinearization = simulate(project,[controlled '_R2021a'],cases(1,1:3),'Virtual jerk v',1);
compare(exportedLinearization,runs{1});
if ~isfolder(project.results),mkdir(project.results);end
save(fullfile(project.results,'dc_motor_validation.mat'), ...
    'runs','metrics','plantRuns','plantErrors');
writetable(metrics,fullfile(project.results,'dc_motor_linearization_metrics.csv'));
disp(plantErrors); disp(metrics);
fprintf('PASS: physical ODE, triple-integrator motion, zero-speed crossings and exports.\n');
end

function run = simulate(project,model,initial,source,command)
load_system(fullfile(project.simulink,[model '.slx']));
cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
names = {'theta','omega','ia','alpha','jerk'};
ports = get_param([model '/Nonlinear DC motor'],'PortHandles');
for index = 1:5,set_param(ports.Outport(index),'DataLogging','on');end
if contains(model,'linearization')
    names = [names {'u','v'}];
    ports = get_param([model '/Linearizing controller'],'PortHandles');
    set_param(ports.Outport(1),'DataLogging','on');
end
ports = get_param([model '/' source],'PortHandles');
set_param(ports.Outport(1),'DataLogging','on');
if ~contains(model,'linearization'),names = [names {'u'}];end
input = Simulink.SimulationInput(model);
variables = {'theta0','omega0','ia0'};
for index = 1:3
    input = input.setVariable(variables{index},initial(index),'Workspace',model);
end
input = input.setBlockParameter([model '/' source],'After',num2str(command));
input = input.setModelParameter('SignalLogging','on','SignalLoggingName','logsout');
output = sim(input);
for index = 1:numel(names)
    signal = output.logsout.get(names{index}).Values;
    run.(names{index}) = signal.Data(:);
    assert(all(isfinite(signal.Data(:))),'Nonfinite simulated signal.');
end
run.time = signal.Time;
end

function compare(exported,original)
assert(isequal(exported.time,original.time),'Export changed output times.');
assert(max(max(abs([exported.theta exported.omega exported.ia] ...
    -[original.theta original.omega original.ia])))<1e-10,'Export changed dynamics.');
end
