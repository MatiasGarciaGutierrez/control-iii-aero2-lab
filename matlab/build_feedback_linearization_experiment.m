function model = build_feedback_linearization_experiment
%BUILD_FEEDBACK_LINEARIZATION_EXPERIMENT Question 3: ideal-speed mechanical test bench.
project = setup_project;
if ~isfolder(project.experiments), mkdir(project.experiments); end
p = aero2_mechanical_parameters;
load_system('simulink');
source = 'model_AERO2_simul_R2021a';
load_system(fullfile(project.course, [source '.slx']));
root = sfroot;
original = root.find('-isa', 'Stateflow.EMChart', ...
    'Path', [source '/Twin rotor (simplified model)/dynamics']);
model = 'aero2_mechanical_linearization_experiment';
if bdIsLoaded(model), close_system(model, 0); end
new_system(model);
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'X0', p.initialState);
set_param(model, 'SolverType','Fixed-step','Solver','ode1','FixedStep','0.002', ...
    'StopTime','2','ReturnWorkspaceOutputs','on','SaveOutput','off', ...
    'SaveTime','off','SignalLogging','off');
names = {'Yaw virtual acceleration', 'Pitch virtual acceleration'};
for k = 1:2
    y = 220+(k-1)*100;
    add_block('simulink/Sources/Pulse Generator',[model '/' names{k}], ...
        'PulseType','Time based','Amplitude',num2str(p.acceleration(k)), ...
        'Period','10','PulseWidth','4','PhaseDelay','0.2','SampleTime','0', ...
        'Position',[40 y 110 y+45]);
end
add_block('simulink/Signal Routing/Mux',[model '/Virtual acceleration'], ...
    'Inputs','2','Position',[210 230 215 365]);
wire(model,[names{1} '/1'],'Virtual acceleration/1');
wire(model,[names{2} '/1'],'Virtual acceleration/2');
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [model '/Mechanical inverse'], 'Position',[350 220 560 350]);
inverse = root.find('-isa','Stateflow.EMChart','Path',[model '/Mechanical inverse']);
if isprop(inverse,'VectorOutputs1D'), inverse.VectorOutputs1D=true; end
% Embed the pure functions so the saved model needs no external MATLAB path.
inverse.Script = sprintf('%s\n%s\n%s', ...
    fileread(fullfile(project.matlab,'mechanics','aero2_feedback_linearization.m')), ...
    fileread(fullfile(project.matlab,'mechanics','aero2_mechanical_terms.m')), ...
    fileread(fullfile(project.matlab,'parameters','aero2_mechanical_parameters.m')));
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [model '/Ideal-speed course mechanics'], 'Position',[690 230 940 350]);
plant = root.find('-isa','Stateflow.EMChart','Path',[model '/Ideal-speed course mechanics']);
if isprop(plant,'VectorOutputs1D'), plant.VectorOutputs1D=true; end
% Use the original dynamics verbatim apart from the local entry-point name.
% Supply algebraic rotor speeds and retain only the four mechanical derivatives.
plant.Script = sprintf([ ...
    'function [dx, acceleration] = ideal_mechanics(x, omega)\n' ...
    'fullDerivative = course_dynamics([0;0], [x;omega]);\n' ...
    'dx = fullDerivative(1:4);\n' ...
    'acceleration = fullDerivative(3:4);\nend\n\n%s'], ...
    strrep(original.Script,'function dx = fcn(u, x)','function dx = course_dynamics(u, x)'));
add_block('simulink/Continuous/Integrator',[model '/Mechanical state'], ...
    'InitialCondition','X0','Position',[1030 235 1080 285]);
add_block('simulink/Signal Routing/Demux',[model '/Angles and rates'], ...
    'Outputs','[2 2]','Position',[1035 440 1040 530]);
add_block('simulink/Math Operations/Sum',[model '/Acceleration residual'], ...
    'Inputs','+-','Position',[1050 330 1080 370]);
wire(model,'Virtual acceleration/1','Mechanical inverse/3');
wire(model,'Mechanical inverse/1','Ideal-speed course mechanics/2');
wire(model,'Ideal-speed course mechanics/1','Mechanical state/1');
wire(model,'Mechanical state/1','Ideal-speed course mechanics/1');
wire(model,'Mechanical state/1','Angles and rates/1');
wire(model,'Angles and rates/1','Mechanical inverse/1');
wire(model,'Angles and rates/2','Mechanical inverse/2');
wire(model,'Virtual acceleration/1','Acceleration residual/1');
wire(model,'Ideal-speed course mechanics/2','Acceleration residual/2');
scopes = {'Acceleration scope','Angle scope','Rate scope','Rotor command scope','Residual scope'};
units = {'Acceleration [rad/s^2]','Angle [rad]','Rate [rad/s]','Rotor speed [rad/s]','Residual [rad/s^2]'};
for k=1:5
    y=150+115*(k-1);
    add_block('simulink/Sinks/Scope',[model '/' scopes{k}], ...
        'NumInputPorts',num2str(1+(k==1)), 'Position',[1250 y 1340 y+65]);
    config=get_param([model '/' scopes{k}],'ScopeConfiguration');
    config.ShowLegend=true; config.ShowGrid=true; config.TimeSpan='2'; config.YLabel=units{k};
end
wire(model,'Virtual acceleration/1','Acceleration scope/1');
wire(model,'Ideal-speed course mechanics/2','Acceleration scope/2');
wire(model,'Angles and rates/1','Angle scope/1');
wire(model,'Angles and rates/2','Rate scope/1');
wire(model,'Mechanical inverse/1','Rotor command scope/1');
wire(model,'Acceleration residual/1','Residual scope/1');
add_block('simulink/Signal Routing/Mux',[model '/Log signals'], ...
    'Inputs','7','Position',[1450 200 1455 660]);
add_block('simulink/Sinks/To Workspace',[model '/Recorded signals'], ...
    'VariableName','linearization_data','SaveFormat','Timeseries', ...
    'SampleTime','-1','MaxDataPoints','inf','Position',[1520 400 1660 445]);
signals={'Virtual acceleration/1','Ideal-speed course mechanics/2', ...
    'Angles and rates/1','Angles and rates/2','Mechanical inverse/1', ...
    'Mechanical inverse/2','Acceleration residual/1'};
for k=1:numel(signals), wire(model,signals{k},sprintf('Log signals/%d',k)); end
wire(model,'Log signals/1','Recorded signals/1');
note=Simulink.Annotation(model,sprintf([ ...
    'Question 3: mechanical feedback linearization with ideal rotor-speed tracking\n' ...
    'a=[yaw acceleration; pitch acceleration]; omega=[main;tail]; x=[q;dq]\n' ...
    'Pulse: yaw +0.15, pitch -0.10 rad/s^2 from 0.2 to 0.6 s; stop 2 s\n' ...
    'Pitch envelope +/-60 deg: assertion stops the simulation outside it. No position controller or motor voltage loop.']));
note.Position=[40 30 1400 130];
save_system(model,fullfile(project.experiments,[model '.slx']));
Simulink.exportToVersion(model,fullfile(project.experiments,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
