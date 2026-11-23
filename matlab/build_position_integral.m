function model = build_position_integral
%BUILD_POSITION_INTEGRAL Optional position integral action on the full cascade.
project=setup_project; p=position_integral_parameters;
source='aero2_cascade_control'; model='aero2_position_integral';
load_system(fullfile(project.simulink,[source '.slx']));
if bdIsLoaded(model), close_system(model,0); end
save_system(source,fullfile(project.simulink,[model '.slx']));
% Keep the existing nonlinear inverse, motor control and supplied plant.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
while ~isempty(lines)
    delete_line(lines(1));
    lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
end
for name={'Position error','Position gain','Rate feedback gain','Desired acceleration','Error scope'}
    delete_block([model '/' name{1}]);
end
notes=find_system(model,'FindAll','on','Type','annotation');
for k=1:numel(notes), delete(get_param(notes(k),'Object')); end
workspace=get_param(model,'ModelWorkspace');
assignin(workspace,'Kref',p.referenceGain); assignin(workspace,'Kpos',p.positionGain);
assignin(workspace,'Dpos',p.rateGain); assignin(workspace,'Kint',p.integralGain);
add_block('simulink/Ports & Subsystems/Subsystem',[model '/Position integral control'], ...
    'Position',[270 170 470 330]);
c=[model '/Position integral control']; Simulink.SubSystem.deleteContents(c);
names={'Desired angles','Measured angles','Measured rates','Raw voltage','Applied voltage'};
for k=1:5
    add_block('simulink/Sources/In1',[c '/' names{k}],'Port',num2str(k), ...
        'Position',[25 40+90*k 55 60+90*k]);
end
add_block('simulink/Math Operations/Sum',[c '/Position error'],'Inputs','+-', ...
    'Position',[120 240 145 275]);
add_block('simulink/Math Operations/Gain',[c '/Reference gain'],'Gain','Kref', ...
    'Position',[220 90 285 130]);
add_block('simulink/Math Operations/Gain',[c '/Position feedback'],'Gain','Kpos', ...
    'Position',[220 165 285 205]);
add_block('simulink/Math Operations/Gain',[c '/Rate feedback'],'Gain','Dpos', ...
    'Position',[220 310 285 350]);
add_block('simulink/Math Operations/Gain',[c '/Integral gain'],'Gain','Kint', ...
    'Position',[220 235 285 275]);
add_block('simulink/User-Defined Functions/MATLAB Function',[c '/Outer anti-windup'], ...
    'Position',[355 400 505 505]);
root=sfroot; chart=root.find('-isa','Stateflow.EMChart','Path',[c '/Outer anti-windup']);
if isprop(chart,'VectorOutputs1D'), chart.VectorOutputs1D=true; end
chart.Script=sprintf(['function derivative = outer_aw(nominal,raw,applied)\n' ...
    'derivative=nominal;\n' ...
    'if any(abs(raw-applied)>1e-8)\n' ...
    '    derivative=zeros(2,1);\n' ...
    'end\nend\n']);
add_block('simulink/Continuous/Integrator',[c '/Integral contribution'], ...
    'InitialCondition','[0;0]','Position',[570 400 610 440]);
add_block('simulink/Math Operations/Sum',[c '/Desired acceleration'],'Inputs','+--+', ...
    'Position',[690 170 720 285]);
add_block('simulink/Sinks/Out1',[c '/Acceleration'],'Position',[800 210 830 230]);
wire(c,'Desired angles/1','Position error/1');
wire(c,'Measured angles/1','Position error/2');
wire(c,'Desired angles/1','Reference gain/1');
wire(c,'Measured angles/1','Position feedback/1');
wire(c,'Measured rates/1','Rate feedback/1');
wire(c,'Position error/1','Integral gain/1');
wire(c,'Integral gain/1','Outer anti-windup/1');
wire(c,'Raw voltage/1','Outer anti-windup/2');
wire(c,'Applied voltage/1','Outer anti-windup/3');
wire(c,'Outer anti-windup/1','Integral contribution/1');
wire(c,'Reference gain/1','Desired acceleration/1');
wire(c,'Position feedback/1','Desired acceleration/2');
wire(c,'Rate feedback/1','Desired acceleration/3');
wire(c,'Integral contribution/1','Desired acceleration/4');
wire(c,'Desired acceleration/1','Acceleration/1');
% Raw voltage is an essential outer anti-windup signal, not test instrumentation.
motor=[model '/IP motor control'];
add_block('simulink/Sinks/Out1',[motor '/Raw voltage output'],'Port','2', ...
    'Position',[700 220 730 240]);
wire(motor,'Raw voltage/1','Raw voltage output/1');
wire(model,'Desired orientation/1','Position integral control/1');
wire(model,'Measured angles/1','Position integral control/2');
wire(model,'Measured rates/1','Position integral control/3');
wire(model,'IP motor control/2','Position integral control/4');
wire(model,'IP motor control/1','Position integral control/5');
wire(model,'Position integral control/1','Mechanical inverse/3');
wire(model,'Measured angles/1','Mechanical inverse/1');
wire(model,'Measured rates/1','Mechanical inverse/2');
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
    'Optional: position integral control; Question 5 remains the baseline\n' ...
    'a=Kref*q_d-Kpos*q+Dpos*(dq_d-dq)+J; J_dot=Kint*(q_d-q), paused during voltage saturation\n' ...
    'Reference weight preserves the ideal critically damped step response; nominal +/-24 V\n' ...
    'Default yaw/pitch [0.2;0.1] rad at 0.2 s; zero initial state; ideal sensors; step 0.002 s']));
note.Position=[35 20 1750 120];
configure_activity_scopes(model,6);
configure_course_reference(model);
save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
