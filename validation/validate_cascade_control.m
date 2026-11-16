function metrics = validate_cascade_control
%VALIDATE_CASCADE_CONTROL Question 5: timing, limits, sensors and tracking.
project=setup_project; p=cascade_parameters; m=aero2_mechanical_parameters;
model='aero2_cascade_control_experiment';
load_system(fullfile(project.experiments,[model '.slx']));
commands=[0.1 0;-0.1 0;0 0.1;0 -0.1;0.2 0.1;-0.2 -0.1];
names={'Yaw positive','Yaw negative','Pitch positive','Pitch negative', ...
    'Simultaneous positive','Simultaneous negative'};
runs=cell(6,1); noisyRuns=cell(6,1); metrics=table; noiseMetrics=table;
idealRuns=cell(6,1);
idealModel='aero2_position_control_experiment';
load_system(fullfile(project.experiments,[idealModel '.slx']));
for k=1:6
    reference=stepReference(commands(k,:)',p);
    r=experiment(model,reference,zeros(6,1),p.step,0,p.stopTime); runs{k}=r;
    noisy=experiment(model,reference,zeros(6,1),p.step,1,p.stopTime); noisyRuns{k}=noisy;
    idealInput=Simulink.SimulationInput(idealModel);
    idealInput=idealInput.setBlockParameter([idealModel '/Desired orientation'],'After',mat2str(commands(k,:)'));
    idealOutput=sim(idealInput); idealRuns{k}=readSignal(idealOutput.position_data,16);
    for axis=1:2
        target=commands(k,axis);
        if target~=0
            row=measure(r.time,r.data(:,18+axis),target,p);
            row.Test=names(k); row.Axis=axis;
            idealRow=measure(idealOutput.position_data.Time,idealRuns{k}(:,2+axis),target,p);
            row.IdealSettlingTime=idealRow.SettlingTime;
            row.Degradation=row.SettlingTime-row.IdealSettlingTime;
            row.PeakVoltage=max(abs(r.data(:,15:16)),[],'all');
            row.SaturationDuration=p.step*sum(any(abs(r.data(:,13:14))>24,2));
            row.PeakIntegrator=max(abs(r.data(:,17:18)),[],'all');
            row.SpecMet=abs(row.SettlingTime-p.targetSettlingTime)<=p.planningTolerance*p.targetSettlingTime;
            metrics=[metrics;row]; %#ok<AGROW>
        end
        window=noisy.time>=p.stopTime-p.finalWindow;
        e=noisy.data(:,axis)-noisy.data(:,18+axis);
        measuredError=noisy.data(window,axis)-noisy.data(window,axis+2);
        row=table(names(k),axis,sqrt(mean(e(noisy.time>=p.stepTime).^2)), ...
            sqrt(mean(e(window).^2)),mean(measuredError),std(measuredError), ...
            max(abs(noisy.data(:,15:16)),[],'all'), ...
            'VariableNames',{'Test','Axis','TrackingRMS','FinalTrueRMS', ...
            'FinalMeasuredErrorMean','FinalMeasuredErrorStd','PeakVoltage'});
        noiseMetrics=[noiseMetrics;row]; %#ok<AGROW>
    end
    assert(max(abs(r.data(:,20)))<=m.pitchLimit,'Pitch envelope exceeded.');
    assert(max(abs(noisy.data(:,20)))<=m.pitchLimit,'Noisy test exceeded pitch envelope.');
    assert(max(abs(r.data(:,17:18)),[],'all')<100,'Nominal integrator contribution is excessive.');
    assert(max(abs(noisy.data(:,17:18)),[],'all')<100,'Noisy integrator contribution is excessive.');
end
% Refine all three representative motion types, retaining the sensor sample period.
fineRuns=cell(3,1); refinement=table;
for j=1:3
    k=[1 3 5]; k=k(j);
    fine=experiment(model,stepReference(commands(k,:)',p),zeros(6,1),p.step/2,0,p.stopTime);
    fineRuns{j}=fine;
    coarse=runs{k};
    angleDifference=max(abs(coarse.data(:,19:20)-interp1(fine.time,fine.data(:,19:20),coarse.time)),[],'all');
    assert(angleDifference<5e-4,'Solver refinement changes the angle response materially.');
    for axis=find(commands(k,:)~=0)
        a=measure(coarse.time,coarse.data(:,18+axis),commands(k,axis),p);
        b=measure(fine.time,fine.data(:,18+axis),commands(k,axis),p);
        assert(abs(a.SettlingTime-b.SettlingTime)<0.03,'Solver refinement changes the settling conclusion.');
        row=table(names(k),axis,a.SettlingTime,b.SettlingTime,angleDifference, ...
            'VariableNames',{'Test','Axis','CoarseSettling','FineSettling','MaxAngleDifference'});
        refinement=[refinement;row]; %#ok<AGROW>
    end
end
zero=experiment(model,stepReference([0;0],p),zeros(6,1),p.step,0,p.stopTime);
assert(max(abs(zero.data(:)))<1e-10,'Zero-reference equilibrium failed.');
regulation=experiment(model,stepReference([0;0],p),[0.1;-0.1;0.05;-0.04;0;0],p.step,0,12);
assert(max(abs(regulation.data(end,19:22)))<1e-4,'Cascade did not regulate the initial state.');
% Recreate the exact angle references from the original initialization script.
reference=courseReference(p.step);
smooth=experiment(model,reference,zeros(6,1),p.step,0,60);
smoothNoisy=experiment(model,reference,zeros(6,1),p.step,1,60);
smoothMetrics=table;
for mode=1:2
    list={smooth,smoothNoisy}; r=list{mode};
    assert(max(abs(r.data(:,17:18)),[],'all')<100,'Smooth-test integrator contribution is excessive.');
    e=r.data(:,1:2)-r.data(:,19:20);
    for axis=1:2
        row=table(mode-1,axis,sqrt(mean(e(:,axis).^2)),max(abs(e(:,axis))), ...
            max(abs(r.data(:,15:16)),[],'all'),max(abs(r.data(:,20))), ...
            'VariableNames',{'MeasurementEffects','Axis','TrackingRMS','PeakTrackingError','PeakVoltage','PeakPitch'});
        smoothMetrics=[smoothMetrics;row]; %#ok<AGROW>
    end
end
% The inverse must refuse operation outside the chosen simulation envelope.
stopped=false;
try
    experiment(model,stepReference([0;1.2],p),zeros(6,1),p.step,0,8);
catch exception
    stopped=contains(exception.message,'Pitch envelope');
end
assert(stopped,'Out-of-envelope cascade did not stop.');
if ~isfolder(project.results), mkdir(project.results); end
writetable(metrics,fullfile(project.results,'cascade_control_metrics.csv'));
writetable(noiseMetrics,fullfile(project.results,'cascade_noise_metrics.csv'));
writetable(refinement,fullfile(project.results,'cascade_refinement_metrics.csv'));
writetable(smoothMetrics,fullfile(project.results,'cascade_tracking_metrics.csv'));
save(fullfile(project.results,'cascade_control_validation.mat'), ...
    'p','commands','names','runs','noisyRuns','idealRuns','metrics','noiseMetrics', ...
    'fineRuns','refinement','zero','regulation','smooth','smoothNoisy','smoothMetrics');
disp(metrics); disp(noiseMetrics); disp(refinement); disp(smoothMetrics);
assert(all(metrics.SpecMet),'Full cascade does not meet the 3 s +/-20% timing criterion.');
assert(all(abs(metrics.FinalError)<1e-4),'Cascade final error is excessive.');
assert(all(metrics.SaturationDuration<0.05),'Nominal tests have sustained saturation.');
fprintf('PASS: cascade timing, voltage limits, bounded integrators, noise, trajectory and refinement.\n');
end

function reference=stepReference(command,p)
reference=[0 0 0;p.stepTime-p.step 0 0;p.stepTime command';p.stopTime command'];
end

function reference=courseReference(step)
t=(0:step:60)';
sstep=@(t,t0,A,T) A*(1-(1+(t-t0)/T).*exp(-(t-t0)/T)).*(t>=t0);
yaw=sstep(t,5,0.5,2.5)+sstep(t,25,-1,2.5)+sstep(t,40,0.5,2.5);
pitch=sstep(t,2,0.3,2.5)+sstep(t,15,-0.5,2.5)+sstep(t,32,0.4,2.5) ...
    +0.1*sin(2*pi*0.05*(t-45)).*sstep(t,45,1,2.5);
reference=[t 2*yaw 2*pitch];
end

function r=experiment(model,reference,state,step,noise,stopTime)
input=Simulink.SimulationInput(model);
input=input.setVariable('Reference',reference,'Workspace',model);
input=input.setVariable('PlantX0',state,'Workspace',model);
input=input.setVariable('meas_on',noise,'Workspace',model);
input=input.setModelParameter('FixedStep',num2str(step),'StopTime',num2str(stopTime));
output=sim(input); r.time=output.cascade_data.Time; r.data=readSignal(output.cascade_data,24);
assert(all(isfinite(r.data(:))),'Cascade produced nonfinite signals.');
assert(max(abs(r.data(:,15:16)),[],'all')<=24,'Applied voltage exceeds amplifier limits.');
assert(max(abs(r.data(:,15:16)-min(24,max(-24,r.data(:,13:14)))),[],'all')<1e-10, ...
    'Applied voltage does not match the raw voltage limiter.');
assert(max(abs(r.data(:,20)))<=pi/3,'True pitch exceeded simulation envelope.');
end

function data=readSignal(signal,width)
if signal.IsTimeFirst
    data=reshape(signal.Data,numel(signal.Time),width);
else
    data=reshape(signal.Data,width,[])';
end
end

function row=measure(time,angle,target,p)
selected=time>=p.stepTime; t=time(selected)-p.stepTime; y=angle(selected);
e=target-y; last=find(abs(e)>p.tolerance*abs(target),1,'last');
settling=NaN;
if ~isempty(last) && last<numel(t), settling=t(last+1); end
normalized=sign(target)*y;
t10=find(normalized>=0.1*abs(target),1); t90=find(normalized>=0.9*abs(target),1);
rise=NaN; if ~isempty(t10) && ~isempty(t90), rise=t(t90)-t(t10); end
overshoot=100*max(0,max(normalized)-abs(target))/abs(target);
row=table(settling,rise,overshoot,e(end), ...
    'VariableNames',{'SettlingTime','RiseTime','OvershootPercent','FinalError'});
end
