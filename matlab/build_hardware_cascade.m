function model = build_hardware_cascade
%BUILD_HARDWARE_CASCADE Adapt the professor's QUARC interface to Question 5.
% Creates a model only. Never builds, connects, downloads, or starts a target.
project=setup_project; p=cascade_parameters;
source='aero2_cascade_control'; template='lin_feedback_EMPTY';
model='aero2_hardware_cascade';
load_system(fullfile(project.simulink,[source '.slx']));
load_system(fullfile(project.root,'real_hardware',[template '.slx']));
if bdIsLoaded(model), close_system(model,0); end
save_system(source,fullfile(project.simulink,[model '.slx']));
% Reconnect the simple cascade around the supplied hardware interface.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
while ~isempty(lines)
    delete_line(lines(1));
    lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
end
delete_block([model '/Course nonlinear plant']);
notes=find_system(model,'FindAll','on','SearchDepth',1,'Type','annotation');
for k=1:numel(notes), delete(get_param(notes(k),'Object')); end
add_block([template '/System'],[model '/AERO 2 hardware'], ...
    'Position',[1250 165 1500 425]);
add_block([template '/HIL Initialize1'],[model '/HIL Initialize'], ...
    'Position',[1260 45 1420 115]);
add_block('simulink/Sources/Constant',[model '/LED color'], ...
    'Value','[0;1;0]','Position',[1130 350 1180 390]);
% The template routes tachometer channel 14000 to its MAIN controller and
% channel 14001 to TAIL, despite its swapped System output labels. Keep wires.
set_param([model '/AERO 2 hardware/w_tail'],'Name','Motor 0 speed - main');
set_param([model '/AERO 2 hardware/w_main'],'Name','Motor 1 speed - tail');
for k=1:2
    names={'Unused filtered yaw','Unused filtered pitch'};
    add_block('simulink/Sinks/Terminator',[model '/' names{k}], ...
        'Position',[1540 430+35*k 1560 450+35*k]);
end
% Use the professor's 180 rad/s tachometer filters; do not alter their gain.
for k=1:2
    names={'Main tachometer filter','Tail tachometer filter'};
    add_block('simulink/Continuous/Transfer Fcn',[model '/' names{k}], ...
        'Numerator','1','Denominator','[1/180 1]', ...
        'Position',[1510 620+65*k 1590 650+65*k]);
end
% Forward Euler matches our simulation integrator at Ts=0.002 s.
controller=[model '/IP motor control'];
pos=get_param([controller '/Integral state'],'Position');
ports=get_param([controller '/Integral state'],'PortHandles');
for h=[ports.Inport ports.Outport]
    line=get_param(h,'Line'); if line~=-1, delete_line(line); end
end
delete_block([controller '/Integral state']);
add_block('simulink/Discrete/Discrete-Time Integrator',[controller '/Integral state'], ...
    'IntegratorMethod','Integration: Forward Euler','SampleTime','Ts', ...
    'InitialCondition','[0;0]','Position',pos);
wire(controller,'Integral derivative/1','Integral state/1');
wire(controller,'Integral state/1','Raw voltage/1');
% Restore passive integral-state scope connection removed with the integrator.
if getSimulinkBlockHandle([controller '/Integral scope'])>0
    wire(controller,'Integral state/1','Integral scope/1');
end
% Diagnostic estimates only; no acceleration sensor is supplied by the model.
for k=1:2
    names={'Angular acceleration estimate','Motor acceleration estimate'};
    add_block('simulink/Discrete/Discrete Transfer Fcn',[model '/' names{k}], ...
        'Numerator','[1 -1]','Denominator','[Ts 0]','SampleTime','Ts', ...
        'Position',[1820 630+70*k 1930 665+70*k]);
end
% Keep the answer diagram readable: feedback collection, then diagnostic Scopes.
set_param([model '/IP motor control'],'ContentPreviewEnabled','off');
set_param([model '/AERO 2 hardware'],'ContentPreviewEnabled','off');
set_param([model '/Measured angles'],'Position',[1700 180 1705 240]);
set_param([model '/Measured rates'],'Position',[1700 340 1705 400]);
set_param([model '/Measured rotor speeds'],'Position',[1840 660 1845 720]);
set_param([model '/Main tachometer filter'],'Position',[1660 645 1770 680]);
set_param([model '/Tail tachometer filter'],'Position',[1660 730 1770 765]);
set_param([model '/Unused filtered yaw'],'Position',[1550 480 1570 500]);
set_param([model '/Unused filtered pitch'],'Position',[1550 530 1570 550]);
set_param([model '/Unused currents'],'Position',[1550 580 1570 600]);
set_param([model '/Angular acceleration estimate'],'Position',[1950 515 2070 550]);
set_param([model '/Motor acceleration estimate'],'Position',[1950 835 2070 870]);
names={'Angle scope','Rate scope','Acceleration scope','Rotor speed scope', ...
    'Motor acceleration scope','Voltage scope','Error scope'};
for k=1:numel(names)
    y=180+160*(k-1);
    set_param([model '/' names{k}],'Position',[2200 y 2280 y+65]);
end
connect_hardware_cascade(model,'AERO 2 hardware');
config=get_param([model '/Acceleration scope'],'ScopeConfiguration');
config.YLabel='Virtual / estimated angular acceleration [rad/s^2]';
% Preserve the template's QUARC target configuration, but fix its 0.1 s step.
config=copy(getActiveConfigSet(template));
old=getActiveConfigSet(model); oldName=old.Name;
config.Name='Hardware QUARC'; attachConfigSet(model,config);
setActiveConfigSet(model,config.Name); detachConfigSet(model,oldName);
set_param(model,'SolverType','Fixed-step','Solver','ode1', ...
    'FixedStep',num2str(p.step),'StopTime','60','SimulationMode','normal', ...
    'UnconnectedInputMsg','error','UnconnectedOutputMsg','error');
workspace=get_param(model,'ModelWorkspace'); assignin(workspace,'Ts',p.step);
% All gains and U_v/U_h are already embedded by the simple Question 5 model.
note=Simulink.Annotation(model,sprintf([ ...
    'Full position cascade: PD -> mechanical inverse -> IP + anti-windup -> AERO 2\n' ...
    'Professor QUARC interface retained; Euler Ts=0.002 s; course U_v/U_h, 60 s\n' ...
    'Motor 0 = main, motor 1 = tail follows template controller routing; confirm on rig.\n' ...
    'Acceleration scopes show finite-difference ESTIMATES, not direct sensor data.\n' ...
    'Hardware build/run and encoder calibration are not validated on this Mac. See docs/hardware.md.']));
note.Position=[30 5 1190 140];
save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
