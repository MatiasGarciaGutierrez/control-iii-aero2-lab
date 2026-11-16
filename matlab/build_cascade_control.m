function model = build_cascade_control
%BUILD_CASCADE_CONTROL Question 5: position control, IP loops and course plant.
project=setup_project; p=cascade_parameters;
source='aero2_position_control'; model='aero2_cascade_control';
load_system(fullfile(project.simulink,[source '.slx']));
if bdIsLoaded(model), close_system(model,0); end
save_system(source,fullfile(project.simulink,[model '.slx']));
% Keep Question 4's position feedback and embedded nonlinear inverse.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
while ~isempty(lines)
    delete_line(lines(1));
    lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
end
for name={'Ideal-speed course mechanics','Mechanical state','Angles and rates','Rotor command scope'}
    delete_block([model '/' name{1}]);
end
notes=find_system(model,'FindAll','on','Type','annotation');
for k=1:numel(notes), delete(get_param(notes(k),'Object')); end
workspace=get_param(model,'ModelWorkspace');
assignin(workspace,'meas_on',0);
assignin(workspace,'Kp',p.motor.kp); assignin(workspace,'Ki',p.motor.ki);
assignin(workspace,'Kaw',p.motor.awGain);
% Copy the supplied plant without changing its equations, sensors or amplifier.
course='model_AERO2_simul_R2021a';
load_system(fullfile(project.course,[course '.slx']));
add_block([course '/Twin rotor (simplified model)'],[model '/Course nonlinear plant'], ...
    'Position',[1250 165 1500 375]);
add_block('simulink/Ports & Subsystems/Subsystem',[model '/IP motor control'], ...
    'Position',[960 190 1120 280]);
controller=[model '/IP motor control']; Simulink.SubSystem.deleteContents(controller);
add_block('simulink/Sources/In1',[controller '/Desired speeds'],'Position',[30 65 60 85]);
add_block('simulink/Sources/In1',[controller '/Measured speeds'],'Port','2', ...
    'Position',[30 200 60 220]);
add_block('simulink/Math Operations/Sum',[controller '/Speed error'],'Inputs','+-', ...
    'Position',[130 65 155 100]);
add_block('simulink/Math Operations/Gain',[controller '/Integral gain'],'Gain','Ki', ...
    'Position',[200 55 260 95]);
add_block('simulink/Math Operations/Sum',[controller '/Integral derivative'],'Inputs','++', ...
    'Position',[315 60 340 95]);
add_block('simulink/Continuous/Integrator',[controller '/Integral state'], ...
    'InitialCondition','[0;0]','Position',[390 55 430 95]);
add_block('simulink/Math Operations/Gain',[controller '/Proportional feedback'],'Gain','Kp', ...
    'Position',[300 185 360 225]);
add_block('simulink/Math Operations/Sum',[controller '/Raw voltage'],'Inputs','+-', ...
    'Position',[490 65 515 100]);
add_block('simulink/Discontinuities/Saturation',[controller '/Voltage limit'], ...
    'UpperLimit','24','LowerLimit','-24','Position',[570 60 630 100]);
add_block('simulink/Sinks/Out1',[controller '/Applied voltage'], ...
    'Position',[700 65 730 85]);
add_block('simulink/Math Operations/Sum',[controller '/Voltage mismatch'],'Inputs','+-', ...
    'Position',[575 300 600 335]);
add_block('simulink/Math Operations/Gain',[controller '/Anti-windup gain'],'Gain','Kaw', ...
    'Orientation','left','Position',[390 295 450 335]);
wire(controller,'Desired speeds/1','Speed error/1');
wire(controller,'Measured speeds/1','Speed error/2');
wire(controller,'Speed error/1','Integral gain/1');
wire(controller,'Integral gain/1','Integral derivative/1');
wire(controller,'Integral derivative/1','Integral state/1');
wire(controller,'Integral state/1','Raw voltage/1');
wire(controller,'Measured speeds/1','Proportional feedback/1');
wire(controller,'Proportional feedback/1','Raw voltage/2');
wire(controller,'Raw voltage/1','Voltage limit/1');
wire(controller,'Voltage limit/1','Applied voltage/1');
wire(controller,'Voltage limit/1','Voltage mismatch/1');
wire(controller,'Raw voltage/1','Voltage mismatch/2');
wire(controller,'Voltage mismatch/1','Anti-windup gain/1');
wire(controller,'Anti-windup gain/1','Integral derivative/2');
for k=1:3
    names={'Measured angles','Measured rates','Measured rotor speeds'};
    add_block('simulink/Signal Routing/Mux',[model '/' names{k}], ...
        'Inputs','2','Position',[1600 390+100*k 1605 440+100*k]);
end
add_block('simulink/Signal Routing/Demux',[model '/Motor voltages'], ...
    'Outputs','2','Position',[1180 200 1185 270]);
add_block('simulink/Sinks/Terminator',[model '/Unused currents'],'Position',[1550 350 1570 370]);
set_param([model '/Angle scope'],'Position',[1770 170 1850 240]);
for k=1:2
    names={'Rotor speed scope','Voltage scope'};
    units={'Rotor speed [rad/s]','Applied voltage [V]'};
    add_block('simulink/Sinks/Scope',[model '/' names{k}], ...
        'NumInputPorts',num2str(3-k),'Position',[1770 280+110*k 1850 345+110*k]);
    config=get_param([model '/' names{k}],'ScopeConfiguration');
    config.ShowLegend=true; config.ShowGrid=true;
    config.TimeSpan=num2str(p.stopTime); config.YLabel=units{k};
end
wire(model,'Desired orientation/1','Position error/1');
wire(model,'Measured angles/1','Position error/2');
wire(model,'Position error/1','Position gain/1');
wire(model,'Position gain/1','Desired acceleration/1');
wire(model,'Measured rates/1','Rate feedback gain/1');
wire(model,'Rate feedback gain/1','Desired acceleration/2');
wire(model,'Measured angles/1','Mechanical inverse/1');
wire(model,'Measured rates/1','Mechanical inverse/2');
wire(model,'Desired acceleration/1','Mechanical inverse/3');
wire(model,'Mechanical inverse/1','IP motor control/1');
wire(model,'Measured rotor speeds/1','IP motor control/2');
wire(model,'IP motor control/1','Motor voltages/1');
for k=1:2
    wire(model,sprintf('Motor voltages/%d',k),sprintf('Course nonlinear plant/%d',k));
    wire(model,sprintf('Course nonlinear plant/%d',2*k-1),sprintf('Measured angles/%d',k));
    wire(model,sprintf('Course nonlinear plant/%d',2*k),sprintf('Measured rates/%d',k));
    wire(model,sprintf('Course nonlinear plant/%d',k+4),sprintf('Measured rotor speeds/%d',k));
end
wire(model,'Course nonlinear plant/7','Unused currents/1');
wire(model,'Desired orientation/1','Angle scope/1');
wire(model,'Measured angles/1','Angle scope/2');
wire(model,'Mechanical inverse/1','Rotor speed scope/1');
wire(model,'Measured rotor speeds/1','Rotor speed scope/2');
wire(model,'IP motor control/1','Voltage scope/1');
note=Simulink.Annotation(model,sprintf([ ...
    'Question 5: complete AERO 2 cascade; all gains retained from Questions 2 and 4\n' ...
    'q_d=[0.2;0.1] rad at 0.2 s; ideal sensors by default (meas_on=0)\n' ...
    'IP motor control applies +/-24 V and back-calculation anti-windup. Euler step 0.002 s.\n' ...
    'Double-click IP motor control to see the two identical vectorized speed loops.']));
note.Position=[35 20 1700 120];
configure_activity_scopes(model,5);
configure_course_reference(model);
save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
