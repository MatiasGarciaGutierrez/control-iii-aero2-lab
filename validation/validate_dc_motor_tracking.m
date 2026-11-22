function metrics = validate_dc_motor_tracking
%VALIDATE_DC_MOTOR_TRACKING Compare nonlinear closed loop with linear dynamics.
project = setup_project; model = 'part1_dc_motor_tracking';
% Initial theta, omega, current, and reference after 0.05 seconds.
cases = [0 0 0 0.1;0 0 0 -0.1;0 0 0 0.2;0.1 0 0 0; ...
    0.1 -0.2 -0.009 0.1;0 0 0 0];
A = [0 1 0;0 0 1;-38^3 -3*38^2 -3*38];
predictedSettling = fzero(@(q)exp(-q)*(1+q+q^2/2)-0.02,[6 9])/38;
metrics = table;runs = cell(size(cases,1),1);
for index = 1:size(cases,1)
    run = simulate(project,model,cases(index,:));
    x0 = cases(index,1:3)';r = cases(index,4);
    z0 = [x0(1);x0(2);20*x0(3)-x0(2)-0.5*x0(2)*abs(x0(2))];
    transition = expm(A*0.05)*z0;expected = zeros(numel(run.time),3);
    for sample = 1:numel(run.time)
        t = run.time(sample);
        if t<0.05,z = expm(A*t)*z0;
        else,z = [r;0;0]+expm(A*(t-0.05))*(transition-[r;0;0]);end
        expected(sample,:) = z';
    end
    expectedCurrent = (expected(:,3)+expected(:,2)+0.5*expected(:,2).*abs(expected(:,2)))/20;
    stateError = max(max(abs([run.theta run.omega run.alpha]-expected)));
    currentError = max(abs(run.ia-expectedCurrent));
    jerkError = max(abs(run.jerk-run.v));
    assert(stateError<2e-5 && currentError<2e-6,'Nonlinear tracking differs from linear solution.');
    assert(jerkError<1e-8,'Jerk cancellation failed.');
    assert(abs(run.theta(end)-r)<1e-4,'Final tracking error is too large.');
    settling = NaN;overshoot = NaN;
    if all(x0==0) && r~=0
        selected = run.time>=0.05;t = run.time(selected)-0.05;y = run.theta(selected);
        outside = find(abs(y-r)>0.02*abs(r),1,'last');
        assert(~isempty(outside) && outside<numel(t),'Step did not settle.');
        settling = t(outside+1);overshoot = max(0,max(sign(r)*y)-abs(r))/abs(r)*100;
        assert(abs(settling-predictedSettling)<0.001,'2% settling differs from prediction.');
        assert(overshoot<1e-4,'Step response overshot.');
    end
    runs{index} = run;
    row = table(r,x0(1),stateError,currentError,jerkError,settling,overshoot,max(abs(run.u)), ...
        'VariableNames',{'Reference','InitialAngle','StateError','CurrentError','JerkError', ...
        'SettlingTime2Percent','OvershootPercent','PeakVoltage'});
    metrics = [metrics;row]; %#ok<AGROW>
end
exported = simulate(project,[model '_R2021a'],cases(1,:));
assert(isequal(exported.time,runs{1}.time),'Export changed solver output times.');
assert(max(abs(exported.theta-runs{1}.theta))<1e-10,'Export changed dynamics.');
save(fullfile(project.results,'dc_motor_tracking_validation.mat'),'runs','metrics','predictedSettling');
writetable(metrics,fullfile(project.results,'dc_motor_tracking_metrics.csv'));
disp(metrics);fprintf('PASS: tracking, signed references, initial states, exact transient and export.\n');
end

function run = simulate(project,model,settings)
load_system(fullfile(project.simulink,[model '.slx']));
cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
sources = {'Nonlinear DC motor','Nonlinear DC motor','Nonlinear DC motor', ...
    'Nonlinear DC motor','Nonlinear DC motor','Linearizing controller','Linear tracking control','Angle reference'};
numbers = [1 2 3 4 5 1 1 1];names = {'theta','omega','ia','alpha','jerk','u','v','reference'};
for index = 1:numel(sources)
    ports = get_param([model '/' sources{index}],'PortHandles');
    set_param(ports.Outport(numbers(index)),'DataLogging','on');
end
input = Simulink.SimulationInput(model);variables = {'theta0','omega0','ia0'};
for index = 1:3,input = input.setVariable(variables{index},settings(index),'Workspace',model);end
input = input.setBlockParameter([model '/Angle reference'],'After',num2str(settings(4)));
input = input.setModelParameter('SignalLogging','on','SignalLoggingName','logsout');
output = sim(input);
for index = 1:numel(names)
    signal = output.logsout.get(names{index}).Values;run.(names{index}) = signal.Data(:);
    assert(all(isfinite(signal.Data(:))),'Nonfinite simulated signal.');
end
run.time = signal.Time;
end
