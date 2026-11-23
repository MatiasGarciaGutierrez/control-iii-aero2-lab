function metrics = validate_position_integral
%VALIDATE_POSITION_INTEGRAL Optional controller timing and disturbance rejection.
project=setup_project; p=position_integral_parameters;
model='aero2_position_integral_experiment';
load_system(fullfile(project.experiments,[model '.slx']));
% Independent ideal augmented-state response: the reference weight cancels a stable pole.
A=[0 1 0;-p.positionGain -p.rateGain 1;-p.integralGain 0 0];
B=[0;p.referenceGain;p.integralGain]; equilibrium=-A\B;
for time=0:0.2:8
    response=equilibrium-expm(A*time)*equilibrium;
    expected=1-(1+p.pole*time)*exp(-p.pole*time);
    assert(abs(response(1)-expected)<1e-12,'Reference weighting does not preserve the ideal step response.');
end
commands=[0.1 0;-0.1 0;0 0.1;0 -0.1;0.2 0.1;-0.2 -0.1];
runs=cell(6,1); metrics=table;
for k=1:6
    test=defaults(p); test.reference=stepReference(commands(k,:)',p);
    r=runTest(model,test,p); runs{k}=r;
    for axis=find(commands(k,:)~=0)
        row=measure(r,axis,commands(k,axis),p.stepTime,p);
        row.Test=k; row.Axis=axis;
        row.PeakVoltage=max(abs(r.data(:,15:16)),[],'all');
        row.SaturationDuration=p.step*sum(any(abs(r.data(:,13:14))>24,2));
        metrics=[metrics;row]; %#ok<AGROW>
    end
end
% Preserve the original Question 5 behavior when the optional action is disabled.
test=defaults(p); test.integral=false;
pd=runTest(model,test,p);
baseline='aero2_cascade_control_experiment';
load_system(fullfile(project.experiments,[baseline '.slx']));
input=Simulink.SimulationInput(baseline);
input=input.setVariable('Reference',test.reference,'Workspace',baseline);
input=input.setVariable('Kff',0,'Workspace',baseline);
input=input.setBlockParameter([baseline '/Desired orientation'],'Interpolate','off');
input=input.setModelParameter('StopTime',num2str(p.stopTime));
output=sim(input); reference=matrix(output.cascade_data,24);
pdDifference=max(abs(pd.data(:,1:24)-reference),[],'all');
assert(pdDifference<1e-7,'Optional experiment changed the PD baseline dynamics.');
assert(max(abs(pd.data(:,19:22)-reference(:,19:22)),[],'all')<1e-10, ...
    'Optional experiment changed the baseline physical state.');
% Add external physical torques, not acceleration-command offsets.
test=defaults(p); test.stop=p.disturbanceStop;
test.torque=[0 0 0;p.disturbanceTime p.disturbance';test.stop p.disturbance'];
test.integral=false; disturbancePD=runTest(model,test,p);
test.integral=true; disturbanceIntegral=runTest(model,test,p);
disturbanceMetrics=table;
for axis=1:2
    window=disturbancePD.time>=test.stop-2;
    ePD=disturbancePD.data(window,axis)-disturbancePD.data(window,18+axis);
    eI=disturbanceIntegral.data(window,axis)-disturbanceIntegral.data(window,18+axis);
    row=table(axis,p.disturbance(axis),mean(ePD),mean(eI),sqrt(mean(ePD.^2)),sqrt(mean(eI.^2)), ...
        'VariableNames',{'Axis','Torque','PDMeanError','IntegralMeanError','PDRMS','IntegralRMS'});
    disturbanceMetrics=[disturbanceMetrics;row]; %#ok<AGROW>
end
q=disturbancePD.data(end,19:20)';
[M,~,~,~]=aero2_mechanical_terms(q,[0;0]);
expectedBalance=M*(p.kp*(p.reference-q))+p.disturbance;
assert(max(abs(expectedBalance))<2e-5,'PD disturbance offset disagrees with physical torque balance.');
% Disturbance sign must reverse the regulation offset, then be rejected again.
test.torque(:,2:3)=-test.torque(:,2:3);
negativeDisturbance=runTest(model,test,p);
assert(max(abs(negativeDisturbance.data(end,19:20)-p.reference'))<2e-5, ...
    'Integral action did not reject the reversed disturbance.');
% Noise/quantization is a separate statistical test, not exact convergence.
test=defaults(p); test.noise=1; noisy=runTest(model,test,p);
window=noisy.time>=p.stopTime-2;
e=noisy.data(window,1:2)-noisy.data(window,19:20);
noiseRMS=sqrt(mean(e.^2));
assert(max(noiseRMS)<0.005,'Sensor effects produce excessive final-window error.');
% Solver refinement on the simultaneous command and disturbed regulation.
test=defaults(p); test.step=p.step/2; fine=runTest(model,test,p);
fineMetrics=table;
for axis=1:2
    row=measure(fine,axis,p.reference(axis),p.stepTime,p); row.Axis=axis;
    fineMetrics=[fineMetrics;row]; %#ok<AGROW>
end
refinementDifference=max(abs(runs{5}.data(:,19:20)-interp1(fine.time,fine.data(:,19:20),runs{5}.time)),[],'all');
assert(refinementDifference<5e-4,'Solver refinement changes nominal angle motion materially.');
test=defaults(p); test.stop=p.disturbanceStop; test.step=p.step/2;
test.torque=[0 0 0;p.disturbanceTime p.disturbance';test.stop p.disturbance'];
fineDisturbance=runTest(model,test,p);
assert(max(abs(fineDisturbance.data(end,19:20)-p.reference'))<2e-5, ...
    'Disturbance rejection fails with the smaller solver step.');
% Temporarily reduce voltage authority, then restore the real +/-24 V limits.
% Both cases retain the motor-loop anti-windup; only outer anti-windup differs.
test=defaults(p); test.stop=14;
test.reference=[0 0 0;p.stepTime 0.4 0;4 0.05 0;test.stop 0.05 0];
test.authority=[0 2;4 24;test.stop 24];
test.outerAW=false; recoveryWithout=runTest(model,test,p);
test.outerAW=true; recoveryWith=runTest(model,test,p);
recoveryMetrics=table;
for enabled=0:1
    list={recoveryWithout,recoveryWith}; r=list{enabled+1};
    row=measure(r,1,0.05,4,p); row.OvershootPercent=[]; row.OuterAW=logical(enabled);
    row.PeakIntegral=max(abs(r.data(:,25:26)),[],'all');
    row.PeakAngle=max(abs(r.data(:,19)));
    recoveryMetrics=[recoveryMetrics;row]; %#ok<AGROW>
end
test.step=p.step/2; fineRecovery=runTest(model,test,p);
fineRecoveryMetric=measure(fineRecovery,1,0.05,4,p);
assert(abs(fineRecoveryMetric.SettlingTime-recoveryMetrics.SettlingTime(2))<0.05, ...
    'Outer anti-windup recovery depends materially on the solver step.');
saturated=recoveryWith.time<4 & any(abs(recoveryWith.data(:,13:14)-recoveryWith.data(:,15:16))>1e-8,2);
assert(any(saturated),'Authority-loss experiment did not exercise outer anti-windup.');
% When saturated at an Euler sample, J must not increase over the next step.
indices=find(saturated); indices=indices(indices<size(recoveryWith.data,1));
freezeError=max(abs(recoveryWith.data(indices+1,25:26)-recoveryWith.data(indices,25:26)),[],'all');
assert(freezeError<1e-10,'Outer integrator accumulated while voltage was saturated.');
% Zero-reference rest and envelope stop remain required behavior.
test=defaults(p); test.reference=stepReference([0;0],p);
zero=runTest(model,test,p); assert(max(abs(zero.data(:)))<1e-10,'Zero-reference rest failed.');
stopped=false; test.reference=stepReference([0;1.2],p);
try
    runTest(model,test,p);
catch exception
    stopped=contains(exception.message,'Pitch envelope');
end
assert(stopped,'The optional controller did not stop outside the pitch envelope.');
if ~isfolder(project.results), mkdir(project.results); end
writetable(metrics,fullfile(project.results,'position_integral_metrics.csv'));
writetable(disturbanceMetrics,fullfile(project.results,'position_integral_disturbance.csv'));
writetable(recoveryMetrics,fullfile(project.results,'position_integral_recovery.csv'));
save(fullfile(project.results,'position_integral_validation.mat'), ...
    'p','runs','metrics','pd','pdDifference','disturbancePD','disturbanceIntegral', ...
    'disturbanceMetrics','negativeDisturbance','noisy','noiseRMS','fine','fineMetrics', ...
    'refinementDifference','fineDisturbance','recoveryWithout','recoveryWith','recoveryMetrics','freezeError','fineRecovery','fineRecoveryMetric');
disp(metrics); disp(disturbanceMetrics); disp(fineMetrics); disp(recoveryMetrics); fprintf('Final-window noise RMS: yaw %.8g, pitch %.8g rad\n',noiseRMS);
fprintf('Fine recovery settling: %.4f s\n',fineRecoveryMetric.SettlingTime);
assert(all(abs(metrics.SettlingTime-3)<=0.6),'Optional position settling is outside 3 s +/-20%.');
assert(all(metrics.SaturationDuration<0.05),'Nominal optional control causes sustained saturation.');
assert(all(abs(metrics.FinalError)<1e-4),'Nominal optional position error is excessive.');
assert(all(disturbanceMetrics.IntegralRMS<2e-5),'Position integrator did not reject the constant torque.');
assert(all(disturbanceMetrics.IntegralRMS<disturbanceMetrics.PDRMS/20), ...
    'Position integral action did not substantially improve disturbance rejection.');
assert(recoveryMetrics.PeakIntegral(2)<recoveryMetrics.PeakIntegral(1), ...
    'Outer anti-windup did not reduce stored integral action.');
assert(recoveryMetrics.SettlingTime(2)<recoveryMetrics.SettlingTime(1), ...
    'Outer anti-windup did not improve recovery.');
fprintf('PASS: optional timing, physical disturbance rejection, sensor effects, refinement and outer anti-windup.\n');
end

function t=defaults(p)
t.reference=stepReference(p.reference,p); t.torque=[0 0 0;p.stopTime 0 0];
t.authority=[0 24;p.stopTime 24]; t.stop=p.stopTime;
t.step=p.step; t.noise=0; t.integral=true; t.outerAW=true;
end

function reference=stepReference(command,p)
reference=[0 0 0;p.stepTime-p.step 0 0;p.stepTime command';p.stopTime command'];
end

function r=runTest(model,t,p)
input=Simulink.SimulationInput(model);
input=input.setBlockParameter([model '/Desired orientation'],'Interpolate','off');
variables={'Reference',t.reference;'Disturbance',t.torque;'VoltageLimits',t.authority; ...
    'meas_on',t.noise;'OuterAW',t.outerAW;'Kff',0};
if t.integral
    gains=[p.referenceGain p.positionGain p.rateGain p.integralGain];
else
    gains=[p.kp p.kp p.kd 0];
end
variables=[variables;{'Kref',gains(1);'Kpos',gains(2);'Dpos',gains(3);'Kint',gains(4)}];
for k=1:size(variables,1), input=input.setVariable(variables{k,1},variables{k,2},'Workspace',model); end
input=input.setModelParameter('StopTime',num2str(t.stop),'FixedStep',num2str(t.step));
output=sim(input); r.time=output.integral_data.Time; r.data=matrix(output.integral_data,28);
assert(all(isfinite(r.data(:))),'Optional controller produced nonfinite signals.');
assert(max(abs(r.data(:,15:16)),[],'all')<=24,'Applied motor voltage exceeds 24 V.');
assert(max(abs(r.data(:,20)))<=pi/3,'True pitch exceeds the simulation envelope.');
end

function data=matrix(signal,width)
if signal.IsTimeFirst
    data=reshape(signal.Data,numel(signal.Time),width);
else
    data=reshape(signal.Data,width,[])';
end
end

function row=measure(r,axis,target,stepTime,p)
selected=r.time>=stepTime; t=r.time(selected)-stepTime; y=r.data(selected,18+axis);
e=target-y; last=find(abs(e)>p.tolerance*abs(target),1,'last');
settling=NaN; if ~isempty(last) && last<numel(t), settling=t(last+1); end
overshoot=100*max(0,max(sign(target)*y)-abs(target))/abs(target);
row=table(settling,overshoot,e(end),'VariableNames',{'SettlingTime','OvershootPercent','FinalError'});
end
