function [nominal, recovery] = validate_motor_ip
%VALIDATE_MOTOR_IP Compare shared-gain PI/IP and signed windup recovery.
project=setup_project; p=motor_ip_parameters;
model='aero2_motor_ip'; load_system(fullfile(project.simulink,[model '.slx']));
if ~isfolder(project.results), mkdir(project.results); end
nominal=table; runs=cell(3,2);
% Compare PI, IP without AW, IP with AW under the same signed commands.
settings=[1 0; 0 0; 0 p.awGain];
labels={'PI','IP','IP + anti-windup'};
for mode=1:3
    for signIndex=1:2
        commands=(-1)^(signIndex-1)*[100 -100];
        r=experiment(model,commands,[0 0],settings(mode,:),1,p.sampleTime);
        runs{mode,signIndex}=r;
        for motor=1:2
            row=metrics(r,motor,commands(motor),p.stepTime);
            row.Controller=labels(mode); nominal=[nominal;row]; %#ok<AGROW>
            if mode==1
                expected=commands(motor)*(1-exp(-p.pole*r.elapsed).* ...
                    (1+(p.decay-p.pole)*r.elapsed));
            else
                expected=commands(motor)*(1-(1+p.pole*r.elapsed).*exp(-p.pole*r.elapsed));
                assert(row.OvershootPercent<0.1,'IP should have a monotonic nominal response.');
                assert(abs(row.SettlingTime-0.43755)<0.015,'IP settling differs from shared-pole prediction.');
            end
            observed=r.data(r.time>=p.stepTime,motor+2);
            assert(max(abs(observed-expected))<1.5,'Nominal response disagrees with analytical solution.');
            assert(row.SaturationTime==0,'A nominal step saturated.');
        end
    end
end
assert(max(abs(runs{2,1}.data(:)-runs{3,1}.data(:)))<1e-9, ...
    'Anti-windup changed the unsaturated response.');
% Unreachable +/-800 rad/s for 1 s; then return to +/-100 rad/s.
recovery=table; recoveryRuns=cell(2,2);
for enabled=1:2
    for signIndex=1:2
        commands=(-1)^(signIndex-1)*[800 -800];
        target=commands/8;
        r=experiment(model,commands,target-commands,[0 (enabled-1)*p.awGain],5,p.sampleTime);
        recoveryRuns{enabled,signIndex}=r;
        for motor=1:2
            row=metrics(r,motor,target(motor),p.recoveryTime);
            row.OvershootPercent=[]; % Initial excess after a return is not step overshoot.
            row.AntiWindup=logical(enabled-1);
            row.PeakIntegral=max(abs(r.data(:,motor+10)));
            recovery=[recovery;row]; %#ok<AGROW>
            assert(max(abs(r.data(:,motor+8)))<=p.voltageLimit+1e-9,'Voltage limit failed.');
            assert(any(abs(r.data(:,motor+6)-r.data(:,motor+8))>1), ...
                'The stress test did not actually saturate.');
        end
    end
end
without=recovery(~recovery.AntiWindup,:); with=recovery(recovery.AntiWindup,:);
assert(all(with.SettlingTime<without.SettlingTime), 'Anti-windup did not improve recovery.');
assert(all(with.PeakIntegral<0.5*without.PeakIntegral), 'Anti-windup did not reduce integral accumulation.');
assert(all(with.PeakIntegral<250), 'Integral contribution exceeded the stress-test bound.');
for active=1:2
    command=[0 0]; command(active)=100;
    independent=experiment(model,command,[0 0],[0 p.awGain],1,p.sampleTime);
    assert(max(abs(independent.data(:,5-active)))<1e-10, ...
        'An uncommanded motor moved: check feedback channel routing.');
    metrics(independent,active,100,p.stepTime);
end
zero=experiment(model,[0 0],[0 0],[0 p.awGain],1,p.sampleTime);
assert(max(abs(zero.data(:)))<1e-10,'Zero-reference equilibrium failed.');
fine=experiment(model,[800 -800],[-700 700],[0 p.awGain],5,p.sampleTime/2);
fineMetric=metrics(fine,1,100,p.recoveryTime);
assert(abs(fineMetric.SettlingTime-with.SettlingTime(1))<0.02,'Recovery depends excessively on solver step.');
% Assert gain retention and original course dynamics directly.
ws=get_param(model,'ModelWorkspace');
assert(evalin(ws,'Kp')==p.kp && evalin(ws,'Ki')==p.ki,'Question 1 gains were changed.');
load_system(fullfile(project.course,'model_AERO2_simul_R2021a.slx'));
root=sfroot;
a=root.find('-isa','Stateflow.EMChart','Path','model_AERO2_simul_R2021a/Twin rotor (simplified model)/dynamics');
b=root.find('-isa','Stateflow.EMChart','Path',[model '/Course nonlinear plant/dynamics']);
assert(strcmp(a.Script,b.Script),'Course dynamics changed.');
writetable(nominal,fullfile(project.results,'motor_ip_nominal.csv'));
writetable(recovery,fullfile(project.results,'motor_ip_recovery.csv'));
save(fullfile(project.results,'motor_ip_validation.mat'), ...
    'runs','recoveryRuns','nominal','recovery','zero','fine','p');
disp(nominal); disp(recovery);
fprintf('PASS: PI/IP shared-gain comparison, signed saturation/recovery, integral bounds and half-step check.\n');
end

function r=experiment(model,commands,changes,settings,stopTime,step)
input=Simulink.SimulationInput(model); names={'Main','Tail'};
for k=1:2
    input=input.setBlockParameter([model '/' names{k} ' speed reference'],'After',num2str(commands(k)));
    input=input.setBlockParameter([model '/' names{k} ' recovery change'],'After',num2str(changes(k)));
end
input=input.setVariable('Beta',settings(1),'Workspace',model);
input=input.setVariable('Kaw',settings(2),'Workspace',model);
input=input.setModelParameter('StopTime',num2str(stopTime),'FixedStep',num2str(step));
output=sim(input); r.time=output.motor_data.Time;
if output.motor_data.IsTimeFirst
    r.data=reshape(output.motor_data.Data,numel(r.time),14);
else
    r.data=reshape(output.motor_data.Data,14,[])';
end
r.elapsed=r.time(r.time>=0.1)-0.1;
assert(all(isfinite(r.data(:))),'A recorded signal became nonfinite.');
end

function row=metrics(r,motor,target,start)
selected=r.time>=start; t=r.time(selected)-start;
y=r.data(selected,motor+2); e=target-y;
last=find(abs(e)>0.02*abs(target),1,'last');
assert(~isempty(last) && last<numel(t),'Speed did not settle during the test.');
settling=t(last+1);
overshoot=max(0,max(sign(target)*y)-abs(target))/abs(target)*100;
finalError=e(end); peakVoltage=max(abs(r.data(:,motor+8)));
saturationTime=sum(abs(r.data(selected,motor+6)-r.data(selected,motor+8))>1e-9)*median(diff(t));
assert(abs(finalError)<0.1,'Final speed error is excessive.');
row=table(motor,target,settling,overshoot,finalError,peakVoltage,saturationTime, ...
    'VariableNames',{'Motor','Target','SettlingTime','OvershootPercent','FinalError','PeakVoltage','SaturationTime'});
end
