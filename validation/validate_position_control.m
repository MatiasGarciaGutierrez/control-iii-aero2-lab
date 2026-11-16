function metrics = validate_position_control
%VALIDATE_POSITION_CONTROL Ideal AERO 2 position steps, decoupling and timing.
project=setup_project; p=position_control_parameters; m=aero2_mechanical_parameters;
model='aero2_position_control_experiment';
load_system(fullfile(project.experiments,[model '.slx']));
commands=[0.1 0;-0.1 0;0 0.1;0 -0.1;0.2 0.1;-0.2 -0.1];
names={'Yaw positive','Yaw negative','Pitch positive','Pitch negative', ...
    'Simultaneous positive','Simultaneous negative'};
runs=cell(size(commands,1),1); metrics=table;
for k=1:size(commands,1)
    r=experiment(model,commands(k,:)',zeros(4,1),p.step); runs{k}=r;
    for axis=1:2
        target=commands(k,axis);
        if target==0
            assert(max(abs(r.data(:,axis+2)))<1e-10,'Uncommanded angle moved: cancellation or routing failed.');
        else
            row=measure(r,axis,target,p);
            row.Test=names(k); metrics=[metrics;row]; %#ok<AGROW>
        end
    end
    tau=max(r.time-p.stepTime,0);
    expected=(1-(1+p.pole*tau).*exp(-p.pole*tau))*commands(k,:);
    assert(max(abs(r.data(:,3:4)-expected),[],'all')<5e-4, ...
        'Position response disagrees with the independent double-integrator solution.');
    assert(max(abs(r.data(:,15:16)),[],'all')<1e-10,'Mechanical acceleration cancellation failed.');
    assert(max(abs(r.data(:,4)))<=m.pitchLimit,'Position test left the pitch envelope.');
end
fine=experiment(model,commands(5,:)',zeros(4,1),p.step/2);
coarseMetric=measure(runs{5},1,commands(5,1),p);
fineMetric=measure(fine,1,commands(5,1),p);
assert(abs(fineMetric.SettlingTime-coarseMetric.SettlingTime)<0.01, ...
    'Smaller solver step changes the settling conclusion.');
zero=experiment(model,[0;0],zeros(4,1),p.step);
assert(max(abs(zero.data(:)))<1e-10,'Zero-reference equilibrium failed.');
initial=[0.1;-0.1;0.05;-0.04];
regulation=experiment(model,[0;0],initial,p.step);
t=regulation.time;
expected=(initial(1:2)'+t*(initial(3:4)+p.pole*initial(1:2))').*exp(-p.pole*t);
assert(max(abs(regulation.data(:,3:4)-expected),[],'all')<5e-4, ...
    'Nonzero-state regulation disagrees with the analytical solution.');
assert(max(abs(regulation.data(end,3:6)))<1e-5,'Nonzero initial state did not converge.');
% A reference beyond the operating envelope must stop before crossing the singularity.
stopped=false;
try
    experiment(model,[0;1.2],zeros(4,1),p.step);
catch exception
    stopped=contains(exception.message,'Pitch envelope');
end
assert(stopped,'Out-of-envelope position tracking did not stop.');
if ~isfolder(project.results), mkdir(project.results); end
writetable(metrics,fullfile(project.results,'position_control_metrics.csv'));
save(fullfile(project.results,'position_control_validation.mat'), ...
    'metrics','runs','fine','fineMetric','zero','regulation','p');
disp(metrics); disp(fineMetric);
fprintf('PASS: signed position steps, cross-axis cancellation, equilibrium, regulation, envelope stop and refinement.\n');
end

function r=experiment(model,command,state,step)
input=Simulink.SimulationInput(model);
p=position_control_parameters;
reference=[0 0 0;p.stepTime-p.step 0 0;p.stepTime command';p.stopTime command'];
input=input.setVariable('Reference',reference,'Workspace',model);
input=input.setBlockParameter([model '/Desired orientation'],'Interpolate','off');
input=input.setVariable('X0',state,'Workspace',model);
input=input.setModelParameter('FixedStep',num2str(step),'StopTime',num2str(p.stopTime));
output=sim(input); r.time=output.position_data.Time;
if output.position_data.IsTimeFirst
    r.data=reshape(output.position_data.Data,numel(r.time),16);
else
    r.data=reshape(output.position_data.Data,16,[])';
end
assert(all(isfinite(r.data(:))),'Position simulation produced nonfinite signals.');
end

function row=measure(r,axis,target,p)
selected=r.time>=p.stepTime; t=r.time(selected)-p.stepTime;
y=r.data(selected,axis+2); e=target-y;
last=find(abs(e)>p.tolerance*abs(target),1,'last');
assert(~isempty(last) && last<numel(t),'Position did not settle during the run.');
settling=t(last+1);
normalized=sign(target)*y;
t10=find(normalized>=0.1*abs(target),1);
t90=find(normalized>=0.9*abs(target),1);
rise=t(t90)-t(t10);
overshoot=max(0,max(normalized)-abs(target))/abs(target)*100;
finalError=e(end);
assert(abs(settling-p.targetSettlingTime)<0.02,'Position settling differs from the exact design target.');
assert(overshoot<1e-6,'Ideal critically damped position response overshot.');
assert(abs(finalError)<1e-5,'Final position error is excessive.');
row=table(axis,target,settling,rise,overshoot,finalError, ...
    max(abs(r.data(:,9))),max(abs(r.data(:,10))), ...
    'VariableNames',{'Axis','Target','SettlingTime','RiseTime','OvershootPercent', ...
    'FinalError','PeakMainSpeed','PeakTailSpeed'});
end
