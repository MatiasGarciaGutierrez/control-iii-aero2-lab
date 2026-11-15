function checks = validate_feedback_linearization
%VALIDATE_FEEDBACK_LINEARIZATION Compare the inverse with original course code.
project=setup_project; p=aero2_mechanical_parameters;
model='aero2_mechanical_linearization';
load_system(fullfile(project.simulink,[model '.slx']));
load_system(fullfile(project.course,'model_AERO2_simul_R2021a.slx'));
root=sfroot;
original=root.find('-isa','Stateflow.EMChart','Path', ...
    'model_AERO2_simul_R2021a/Twin rotor (simplified model)/dynamics');
plant=root.find('-isa','Stateflow.EMChart','Path',[model '/Ideal-speed course mechanics']);
assert(contains(plant.Script,strrep(original.Script, ...
    'function dx = fcn(u, x)','function dx = course_dynamics(u, x)')), ...
    'Ideal mechanics no longer contains the original course dynamics.');
inverse=root.find('-isa','Stateflow.EMChart','Path',[model '/Mechanical inverse']);
files={fullfile(project.matlab,'mechanics','aero2_feedback_linearization.m'), ...
    fullfile(project.matlab,'mechanics','aero2_mechanical_terms.m'), ...
    fullfile(project.matlab,'parameters','aero2_mechanical_parameters.m')};
for k=1:numel(files)
    assert(contains(inverse.Script,fileread(files{k})), ...
        'Saved inverse differs from the documented MATLAB implementation.');
end
% Temporary independent oracle: the original course implementation, not the inverse helper.
folder=tempname; mkdir(folder); cleaner=onCleanup(@() cleanOracle(folder));
file=fopen(fullfile(folder,'aero2_course_oracle.m'),'w');
fprintf(file,'%s',strrep(original.Script,'function dx = fcn(u, x)', ...
    'function dx = aero2_course_oracle(u, x)')); fclose(file); addpath(folder);
oracle=str2func('aero2_course_oracle');
maxError=0; maxReconstruction=0; count=0;
velocities=[0 0; -0.5 0.4; 0.7 -0.3];
accelerations=[0 0; 0.1 -0.2; -0.3 0.15; 0.5 0.4];
for yaw=[-0.4 0 0.3]
    for pitch=[-60 -30 0 30 60]*pi/180
        for j=1:size(velocities,1)
            for k=1:size(accelerations,1)
                q=[yaw;pitch]; dq=velocities(j,:)'; a=accelerations(k,:)';
                [omega,s]=aero2_feedback_linearization(q,dq,a);
                dx=oracle([0;0],[q;dq;omega]);
                maxError=max(maxError,max(abs(dx(3:4)-a)));
                maxReconstruction=max(maxReconstruction,max(abs(omega.*abs(omega)-s)));
                count=count+1;
            end
        end
    end
end
assert(maxError<1e-10,'Computed speeds do not recover requested acceleration.');
assert(maxReconstruction<1e-8,'Signed square root reconstruction failed.');
[w,s]=aero2_feedback_linearization([0;0],[0;0],[0;0]);
assert(all(w==0) && all(s==0),'Zero rotor input is not handled exactly.');
rejected=false;
try
    aero2_feedback_linearization([0;p.pitchLimit+0.001],[0;0],[0;0]);
catch exception
    rejected=contains(exception.message,'Pitch envelope');
end
assert(rejected,'The inverse accepted an out-of-envelope pitch.');
nominal=experiment(model,p.initialState,p.step);
fine=experiment(model,p.initialState,p.step/2);
assert(max(abs(nominal.data(:,13:14)),[],'all')<1e-10,'Dynamic acceleration cancellation failed.');
t=nominal.time;
expectedInput=(t>=0.2 & t<0.6)*p.acceleration';
assert(max(abs(nominal.data(:,1:2)-expectedInput),[],'all')<1e-10,'Virtual acceleration pulse settings differ.');
expectedQ=p.initialState(1:2)'+t*p.initialState(3:4)' + ...
    0.5*(max(t-0.2,0).^2-max(t-0.6,0).^2)*p.acceleration';
positionError=max(abs(nominal.data(:,5:6)-expectedQ),[],'all');
assert(positionError<0.001,'Integrated positions do not match two double integrators.');
% Fixed-step Euler is only first order; refinement must improve the position error.
t=fine.time;
expectedFine=p.initialState(1:2)'+t*p.initialState(3:4)' + ...
    0.5*(max(t-0.2,0).^2-max(t-0.6,0).^2)*p.acceleration';
fineError=max(abs(fine.data(:,5:6)-expectedFine),[],'all');
assert(fineError<positionError,'Smaller integration step did not improve agreement.');
assert(max(abs(nominal.data(:,6)))<=p.pitchLimit,'Nominal test left the envelope.');
% Start inside the envelope and cross it before the acceleration pulse begins.
input=Simulink.SimulationInput(model);
input=input.setVariable('X0',[0;p.pitchLimit-0.001;0;0.1],'Workspace',model);
input=input.setModelParameter('StopTime','0.05');
stopped=false;
try
    sim(input);
catch exception
    stopped=contains(exception.message,'Pitch envelope');
end
assert(stopped,'Simulation did not stop on leaving the pitch envelope.');
checks=table(count,maxError,maxReconstruction,positionError,fineError, ...
    max(abs(nominal.data(:,13:14)),[],'all'), ...
    'VariableNames',{'SubstitutionCases','MaxAccelerationError','SignedSquareError', ...
    'PositionError','HalfStepPositionError','DynamicResidual'});
if ~isfolder(project.results), mkdir(project.results); end
save(fullfile(project.results,'feedback_linearization_validation.mat'), ...
    'checks','nominal','fine','p');
writetable(checks,fullfile(project.results,'feedback_linearization_checks.csv'));
disp(checks);
fprintf('PASS: course-equation substitution, zero input, ideal double integrators and pitch-envelope stop.\n');
end

function r=experiment(model,state,step)
input=Simulink.SimulationInput(model);
input=input.setVariable('X0',state,'Workspace',model);
input=input.setModelParameter('FixedStep',num2str(step));
output=sim(input); r.time=output.linearization_data.Time;
if output.linearization_data.IsTimeFirst
    r.data=reshape(output.linearization_data.Data,numel(r.time),14);
else
    r.data=reshape(output.linearization_data.Data,14,[])';
end
assert(all(isfinite(r.data(:))),'Simulation produced nonfinite values.');
end

function cleanOracle(folder)
rmpath(folder);
rmdir(folder,'s');
end
