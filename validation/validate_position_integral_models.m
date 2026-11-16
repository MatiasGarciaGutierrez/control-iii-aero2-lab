function difference = validate_position_integral_models
%VALIDATE_POSITION_INTEGRAL_MODELS Simple/experiment equivalence and export checks.
project=setup_project; model='aero2_position_integral';
load_system(fullfile(project.simulink,[model '.slx']));
cleanup=onCleanup(@() close_system(model,0));
assert(isempty(find_system(model,'SearchDepth',1,'BlockType','ToWorkspace')), ...
    'The optional answer contains saved experiment logging.');
% Temporarily expose the outer integral contribution; discard every change.
c=[model '/Position integral control'];
add_block('simulink/Sinks/Out1',[c '/Temporary integral output'],'Port','2');
add_line(c,'Integral contribution/1','Temporary integral output/1');
sources={'Desired orientation/1','Measured angles/1','Measured rates/1', ...
    'Position integral control/1','Mechanical inverse/1','Measured rotor speeds/1', ...
    'IP motor control/2','IP motor control/1','Position integral control/2'};
add_block('simulink/Signal Routing/Mux',[model '/Temporary log mux'],'Inputs',num2str(numel(sources)));
add_block('simulink/Sinks/To Workspace',[model '/Temporary log'], ...
    'VariableName','equivalence_data','SaveFormat','Timeseries','SampleTime','-1');
for k=1:numel(sources)
    add_line(model,sources{k},sprintf('Temporary log mux/%d',k),'autorouting','on');
end
add_line(model,'Temporary log mux/1','Temporary log/1');
output=sim(model); signal=output.equivalence_data;
simple=matrix(signal,18); time=signal.Time;
clear cleanup;
experiment=[model '_experiment'];
load_system(fullfile(project.experiments,[experiment '.slx'])); output=sim(experiment);
reference=matrix(output.integral_data,28); columns=[1:16 25:26];
assert(isequal(time,output.integral_data.Time),'Simple and experiment time vectors differ.');
difference=max(abs(simple-reference(:,columns)),[],'all');
assert(difference<1e-10,'Optional simple and experiment dynamics differ.');
% Run saved R2021a exports in the available release; this is not R2021a execution.
exported=[model '_R2021a']; load_system(fullfile(project.simulink,[exported '.slx'])); sim(exported);
exported=[experiment '_R2021a']; load_system(fullfile(project.experiments,[exported '.slx']));
output=sim(exported); exportedData=matrix(output.integral_data,28);
assert(max(abs(exportedData-reference),[],'all')<1e-10,'The R2021a export changes default dynamics.');
% Baseline/course source files are not modified by either new builder.
root=sfroot;
load_system(fullfile(project.simulink,'aero2_cascade_control.slx'));
load_system(fullfile(project.simulink,[model '.slx']));
a=root.find('-isa','Stateflow.EMChart','Path','aero2_cascade_control/Course nonlinear plant/dynamics');
b=root.find('-isa','Stateflow.EMChart','Path',[model '/Course nonlinear plant/dynamics']);
assert(strcmp(a.Script,b.Script),'The simple optional model changed the supplied dynamics.');
fprintf('PASS: optional simple/experiment difference %.3g; exports match in R2024b; nominal plant preserved.\n',difference);
end

function data=matrix(signal,width)
if signal.IsTimeFirst
    data=reshape(signal.Data,numel(signal.Time),width);
else
    data=reshape(signal.Data,width,[])';
end
end
