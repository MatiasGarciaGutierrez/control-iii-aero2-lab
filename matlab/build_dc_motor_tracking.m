function model = build_dc_motor_tracking
%BUILD_DC_MOTOR_TRACKING Part I, Question 2(c-d): constant-reference tracking.
project = setup_project; source = 'part1_dc_motor_linearization';
model = 'part1_dc_motor_tracking';
if bdIsLoaded(model),close_system(model,0);end
load_system(fullfile(project.simulink,[source '.slx']));
save_system(source,fullfile(project.simulink,[model '.slx']));
workspace = get_param(model,'ModelWorkspace');
assignin(workspace,'K1',38^3);assignin(workspace,'K2',3*38^2);assignin(workspace,'K3',3*38);
set_param(model,'StopTime','0.5','MaxStep','0.0005','AlgebraicLoopMsg','error');
ports = get_param([model '/Virtual jerk v'],'PortHandles');
delete_line(get_param(ports.Outport(1),'Line'));
delete_block([model '/Virtual jerk v']);
feedback = [model '/Linear tracking control'];
add_block('built-in/Subsystem',feedback,'Position',[60 175 225 340]);
names = {'reference','theta','omega','ia'};
for index = 1:4
    y = 65*index;
    add_block('simulink/Ports & Subsystems/In1',[feedback '/' names{index}], ...
        'Port',num2str(index),'Position',[35 y 65 y+20]);
end
add_block('simulink/Signal Routing/Mux',[feedback '/Tracking arguments'], ...
    'Inputs','4','Position',[145 65 150 280]);
add_block('simulink/User-Defined Functions/Fcn',[feedback '/State feedback'], ...
    'Expr','K1*(u(1)-u(2))-K2*u(3)-K3*(KT*u(4)-d1*u(3)-d2*u(3)*abs(u(3)))/J', ...
    'Position',[240 145 725 200]);
add_block('simulink/Ports & Subsystems/Out1',[feedback '/v'], ...
    'Port','1','Position',[790 165 820 185]);
for index = 1:4
    add_line(feedback,[names{index} '/1'],sprintf('Tracking arguments/%d',index),'autorouting','on');
end
add_line(feedback,'Tracking arguments/1','State feedback/1','autorouting','on');
add_line(feedback,'State feedback/1','v/1','autorouting','on');
add_block('simulink/Sources/Step',[model '/Angle reference'], ...
    'Time','0.05','Before','0','After','0.1','Position',[65 115 115 145]);
connections = {'Angle reference/1','Linear tracking control/1'; ...
    'Nonlinear DC motor/1','Linear tracking control/2'; ...
    'Nonlinear DC motor/2','Linear tracking control/3'; ...
    'Nonlinear DC motor/3','Linear tracking control/4'; ...
    'Linear tracking control/1','Linearizing controller/1'; ...
    'Linear tracking control/1','Jerk scope/2'};
for index = 1:size(connections,1)
    add_line(model,connections{index,1},connections{index,2},'autorouting','on');
end
set_param([model '/Angle scope'],'NumInputPorts','2');
add_line(model,'Angle reference/1','Angle scope/2','autorouting','on');
ports = get_param([model '/Angle reference'],'PortHandles');
set_param(get_param(ports.Outport(1),'Line'),'Name','reference');
ports = get_param(feedback,'PortHandles');set_param(get_param(ports.Outport(1),'Line'),'Name','v');
scopes = find_system(model,'SearchDepth',1,'BlockType','Scope');
for index = 1:numel(scopes)
    configuration = get_param(scopes{index},'ScopeConfiguration');configuration.TimeSpan = '0.5';
end
notes = find_system(model,'FindAll','on','SearchDepth',1,'Type','annotation');delete(notes);
note = Simulink.Annotation(model,sprintf([ ...
    'Part I - Question 2(c-d): constant-reference tracking\n' ...
    'v = 54872*(reference-theta) - 4332*omega - 114*alpha\n' ...
    'Triple pole at -38; predicted 2%% settling time = 0.1978 s.\n' ...
    'Reference: 0 to 0.1 rad at 0.05 s; zero initial states; ideal voltage authority.']));
note.Position = [60 10 1100 95];
save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end
