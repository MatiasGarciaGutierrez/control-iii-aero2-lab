function metrics = validate_cstr
%VALIDATE_CSTR Check minute-to-second conversion, cancellation and zero dynamics.
project = setup_project;reaction = @(T)7.2e10*exp(-8750./T);
C310 = 1/(1+reaction(310));C330 = 1/(1+reaction(330));K = log(50)/5;
U310 = 270-200*reaction(310)*C310;
plantErrors = zeros(2,1);plantRuns = cell(2,1);
for index = 1:2
    command = U310+5*(3-2*index);
    run = simulate(project,'part1_cstr',310,C310,'Coolant temperature',U310,command);
    rhs = @(t,x)[(1-x(1)-reaction(x(2))*x(1))/60; ...
        (350-2*x(2)+200*reaction(x(2))*x(1)+U310+(command-U310)*(t>=1))/60];
    reference = ode45(rhs,[0 180],[C310;310],odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',0.02));
    expected = deval(reference,run.time)';
    plantErrors(index) = max(max(abs([run.CA run.T]-expected)));
    assert(plantErrors(index)<2e-5,'Open-loop plant disagrees with independent ODE.');
    plantRuns{index} = run;
end
linearRuns = cell(2,1);
for index = 1:2
    command = 0.1*(3-2*index);
    run = simulate(project,'part1_cstr_linearization',310,C310,'Virtual temperature rate',0,command);
    assert(max(abs(run.T-(310+command*max(run.time-1,0))))<2e-7, ...
        'Linearized temperature does not integrate the virtual input.');
    assert(max(abs(run.Tdot-run.v))<1e-10,'Rate cancellation failed.');
    linearRuns{index} = run;
end
% Initial temperature, initial concentration, desired temperature.
cases = [310 C310 330;310 C310 320;310 0.5 330;310 C310 310;330 0.5 330];
metrics = table;runs = cell(size(cases,1),1);
for index = 1:size(cases,1)
    initialT = cases(index,1);initialC = cases(index,2);desired = cases(index,3);
    run = simulate(project,'part1_cstr_tracking',initialT,initialC,'Temperature reference',initialT,desired);
    temperature = @(t)desired+(initialT-desired)*exp(-K*max(t-1,0));
    temperatureError = max(abs(run.T-temperature(run.time)));
    assert(temperatureError<2e-6,'Thermal response differs from first-order prediction.');
    assert(max(abs(run.Tdot-run.v))<1e-10,'Temperature rate differs from virtual input.');
    concentration = ode45(@(t,c)(1-(1+reaction(temperature(t)))*c)/60, ...
        [0 180],initialC,odeset('RelTol',1e-11,'AbsTol',1e-13,'MaxStep',0.02));
    concentrationError = max(abs(run.CA-deval(concentration,run.time)'));
    assert(concentrationError<1e-7,'Concentration differs from the internal-dynamics ODE.');
    assert(all(run.CA>=-1e-10 & run.CA<=1+1e-10),'Concentration left its physical interval.');
    assert(all(run.T>0) && all(run.Tc>0),'Absolute temperature became nonphysical.');
    settling = NaN;
    if desired~=initialT
        selected = run.time>=1;t = run.time(selected)-1;
        outside = find(abs(run.T(selected)-desired)>0.02*abs(desired-initialT),1,'last');
        assert(~isempty(outside)&&outside<numel(t),'Temperature did not settle.');
        settling = t(outside+1);
        assert(abs(settling-5)<0.025,'Temperature settling misses the 5-second target.');
    end
    if index==5
        zeroDynamics = C330+(initialC-C330)*exp(-(1+reaction(330))*run.time/60);
        assert(max(abs(run.CA-zeroDynamics))<1e-8,'Shifted zero dynamics disagree with their solution.');
    end
    runs{index} = run;
    row = table(initialT,initialC,desired,temperatureError,concentrationError,settling, ...
        max(run.Tc),min(run.Tc),run.CA(end),'VariableNames', ...
        {'InitialTemperature','InitialConcentration','Reference','TemperatureError', ...
        'ConcentrationError','SettlingTime2Percent','PeakCoolant','MinimumCoolant','FinalConcentration'});
    metrics = [metrics;row]; %#ok<AGROW>
end
for model = {'part1_cstr','part1_cstr_linearization','part1_cstr_tracking'}
    name = model{1};
    if strcmp(name,'part1_cstr')
        exported = simulate(project,[name '_R2021a'],310,C310,'Coolant temperature',U310,U310+5);original = plantRuns{1};
    elseif strcmp(name,'part1_cstr_linearization')
        exported = simulate(project,[name '_R2021a'],310,C310,'Virtual temperature rate',0,0.1);original = linearRuns{1};
    else
        exported = simulate(project,[name '_R2021a'],310,C310,'Temperature reference',310,330);original = runs{1};
    end
    assert(isequal(exported.time,original.time),'Export changed solver times.');
    assert(max(abs(exported.T-original.T))<1e-10,'Export changed dynamics.');
end
if ~isfolder(project.results),mkdir(project.results);end
save(fullfile(project.results,'cstr_validation.mat'),'runs','metrics','linearRuns','plantRuns','plantErrors','C310','C330','K');
writetable(metrics,fullfile(project.results,'cstr_metrics.csv'));
disp(metrics);fprintf('PASS: physical plant, cancellation, 5 s tracking, zero dynamics and all exports.\n');
end

function run = simulate(project,model,T0,C0,source,before,after)
load_system(fullfile(project.simulink,[model '.slx']));
cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
names = {'T','CA','Tdot'};ports = get_param([model '/Nonlinear CSTR'],'PortHandles');
for index = 1:3,set_param(ports.Outport(index),'DataLogging','on');end
if contains(model,'tracking')
    sources = {'Linearizing controller','Linear temperature control','Temperature reference'};
    names = [names {'Tc','v','reference'}];
elseif contains(model,'linearization')
    sources = {'Linearizing controller','Virtual temperature rate'};names = [names {'Tc','v'}];
else
    sources = {'Coolant temperature'};names = [names {'Tc'}];
end
for index = 1:numel(sources)
    ports = get_param([model '/' sources{index}],'PortHandles');set_param(ports.Outport(1),'DataLogging','on');
end
input = Simulink.SimulationInput(model);
input = input.setVariable('T0',T0,'Workspace',model);
input = input.setVariable('C0',C0,'Workspace',model);
input = input.setBlockParameter([model '/' source],'Before',num2str(before,17));
input = input.setBlockParameter([model '/' source],'After',num2str(after,17));
input = input.setModelParameter('SignalLogging','on','SignalLoggingName','logsout');
output = sim(input);
for index = 1:numel(names)
    signal = output.logsout.get(names{index}).Values;run.(names{index}) = signal.Data(:);
    assert(all(isfinite(signal.Data(:))),'Nonfinite signal.');
end
run.time = signal.Time;
end
