function model = build_professor_cascade(sourceFile)
%BUILD_PROFESSOR_CASCADE Apply Lab 2 control to the professor's block layout.
% The supplied Lab 3 source is read only; the saved Lab 2 model is standalone.
project=setup_project;
if nargin<1
    sourceFile=fullfile(fileparts(project.root),'lab3','simulink', ...
        'lin_feedback_simplified_R2024b.slx');
end
assert(isfile(sourceFile),'Supply the professor model path to rebuild this model.');
p=cascade_parameters; [U_v,U_h]=aero2_course_reference;
model='aero2_cascade_professor_format';
[~,source]=fileparts(sourceFile);
load_system(sourceFile);
if bdIsLoaded(model), close_system(model,0); end
save_system(source,fullfile(project.simulink,[model '.slx']));
% Remove the Lab 3 base-workspace initializer; embed all needed Lab 2 data.
set_param(model,'InitFcn','','StopTime','60','SolverType','Fixed-step', ...
    'Solver','ode1','FixedStep','0.002','ReturnWorkspaceOutputs','on');
w=get_param(model,'ModelWorkspace');
w.DataSource='Model File';
names={'Kp','Kd','Ki','kp_PI_motor','ki_PI_motor','kw_PI_motor', ...
    'meas_on','Ts','tmax','U_v','U_h'};
% The professor puts Ki AFTER the integrator, so Kaw/Ki multiplies the
% voltage mismatch before it: dI/dt=Ki*(r-y)+Kaw*(u_sat-u_raw).
values={p.kp,p.kd,0,p.motor.kp,p.motor.ki,p.motor.awGain/p.motor.ki, ...
    1,p.step,60,U_v,U_h};
for k=1:numel(names), assignin(w,names{k},values{k}); end
for name={'IP_m','IP_t'}
    % Retain the existing transfer-function block, using unity to reproduce
    % our unfiltered back-calculation law rather than adding a new AW state.
    set_param([model '/' name{1} '/Transfer Fcn2'], ...
        'Numerator','1','Denominator','1');
end
% Keep the professor's two 180 rad/s tachometer filters, signed square roots,
% nonlinear inverse, plant, reference derivative blocks and visible limits.
scopes=find_system(model,'BlockType','Scope');
for k=1:numel(scopes)
    config=get_param(scopes{k},'ScopeConfiguration');
    config.TimeSpan='60'; config.DataLoggingSaveFormat='Dataset';
end
% Keep the answer model simple: diagnostics live only in an experiment copy.
for name={'Clock log','From log psi','From log theta','From log w_main', ...
        'From log w_tail','Mux log','log','log time'}
    delete_block([model '/' name{1}]);
end
% Deleting diagnostic blocks leaves open branch stubs in the source layout.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
for k=numel(lines):-1:1
    if ishandle(lines(k)) && isempty(get_param(lines(k),'LineChildren')) && ...
            all(get_param(lines(k),'DstPortHandle')==-1)
        delete_line(lines(k));
    end
end
charts=find(sfroot,'-isa','Stateflow.EMChart');
for k=1:numel(charts)
    if startsWith(charts(k).Path,[model '/']) && isprop(charts(k),'VectorOutputs1D')
        charts(k).VectorOutputs1D=true;
    end
end
notes=find_system(model,'FindAll','on','Type','annotation');
for k=1:numel(notes), delete(get_param(notes(k),'Object')); end
note=Simulink.Annotation(model,sprintf([ ...
    'Lab 2: nominal PD + nonlinear inverse + IP motor loops\n' ...
    'Course references embedded; press Run, then open Position and Motor Speeds.\n' ...
    'Visible +/-24 V limits; Kaw=20 s^-1; sensor effects on; no QUARC hardware.']));
note.Position=[-800 700 600 790];
set_param(model,'Description',sprintf([ ...
    'Lab 2 nominal PD + exact inverse + IP cascade in the professor layout.\n' ...
    'Visible +/-24 V limits feed both the plant and IP anti-windup ports.\n' ...
    'Kp=%.4f, Kd=%.4f, Ki=0; motor Kp=%.4f, Ki=%.4f, Kaw=%.4f.\n' ...
    'Original smooth course references; sensor effects on; tachometer LPFs 180 rad/s.'], ...
    p.kp,p.kd,p.motor.kp,p.motor.ki,p.motor.awGain));
save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end
