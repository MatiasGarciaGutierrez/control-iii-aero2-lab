function model = import_ballbeam_model(sourceFile)
%IMPORT_BALLBEAM_MODEL Import only Lars's optional exercise, preserving source.
project=setup_project;
if nargin<1
    sourceFile='/Users/matiasgarcia/Desktop/AC3_Lab2_Linearization/task1/ex4_ballbeam.slx';
end
assert(isfile(sourceFile),'Supply the optional ball-and-beam model path.');
[~,source]=fileparts(sourceFile);load_system(sourceFile);
model='part1_ballbeam_experiment';
if bdIsLoaded(model),close_system(model,0);end
save_system(source,fullfile(project.experiments,[model '.slx']));
% Store the model's own parameter callback in its model workspace so this
% selected handoff runs without modifying the caller's base workspace.
settings=get_param(model,'InitFcn');w=get_param(model,'ModelWorkspace');
w.DataSource='Model File';evalin(w,settings);
set_param(model,'InitFcn','','SolverType','Variable-step','Solver','ode45', ...
    'StopTime','10','MaxStep','0.005','RelTol','1e-8','AbsTol','1e-10', ...
    'ReturnWorkspaceOutputs','on');
scopes=find_system(model,'BlockType','Scope');
for k=1:numel(scopes)
    config=get_param(scopes{k},'ScopeConfiguration');config.DataLoggingSaveFormat='Dataset';
end
save_system(model);
simple='part1_ballbeam';
if bdIsLoaded(simple),close_system(simple,0);end
save_system(model,fullfile(project.simulink,[simple '.slx']));
% Keep the answer model's plant, coordinates, law, feedback and basic scopes.
blocks=find_system(simple,'SearchDepth',1,'BlockType','ToWorkspace');
for k=1:numel(blocks),delete_block(blocks{k});end
for name={'Part (a) open loop','Clock (a)','t <= t_stop_plant','t_stop_plant','Designed linear closed loop'}
    delete_block([simple '/' name{1}]);
end
set_param([simple '/Output'],'NumInputPorts','2');
lines=find_system(simple,'FindAll','on','SearchDepth',1,'Type','line');
for k=numel(lines):-1:1
    if ishandle(lines(k)) && isempty(get_param(lines(k),'LineChildren')) && ...
            (all(get_param(lines(k),'DstPortHandle')==-1) || get_param(lines(k),'SrcPortHandle')==-1)
        delete_line(lines(k));
    end
end
save_system(simple);
Simulink.exportToVersion(simple,fullfile(project.simulink,[simple '_R2021a.slx']),'R2021a');
end
