function model = build_position_integral_experiment
%BUILD_POSITION_INTEGRAL_EXPERIMENT Optional tests, disturbance and diagnostics.
project=setup_project; p=position_integral_parameters;
source='aero2_position_integral'; model=[source '_experiment'];
load_system(fullfile(project.simulink,[source '.slx']));
if bdIsLoaded(model), close_system(model,0); end
save_system(source,fullfile(project.experiments,[model '.slx']));
w=get_param(model,'ModelWorkspace');
reference=[0 0 0;p.stepTime-p.step 0 0;p.stepTime p.reference';p.stopTime p.reference'];
assignin(w,'Reference',reference); assignin(w,'Disturbance',[0 0 0;p.stopTime 0 0]);
assignin(w,'OuterAW',true); assignin(w,'VoltageLimits',[0 24;p.stopTime 24]);
assignin(w,'PlantX0',zeros(6,1));
set_param([model '/Course nonlinear plant/x'],'InitialCondition','PlantX0');
delete_line(model,'Desired orientation/1','Position integral control/1');
delete_line(model,'Desired orientation/1','Angle scope/1');
position=get_param([model '/Desired orientation'],'Position');
delete_block([model '/Desired orientation']);
add_block('simulink/Sources/From Workspace',[model '/Desired orientation'], ...
    'VariableName','Reference','Interpolate','off','OutputAfterFinalValue','Holding final value','Position',position);
wire(model,'Desired orientation/1','Position integral control/1');
wire(model,'Desired orientation/1','Angle scope/1');
% Test-only external torque enters the acceleration equation after cancellation.
plant=[model '/Course nonlinear plant'];
add_block('simulink/Sources/In1',[plant '/Disturbance torque'],'Port','3', ...
    'Position',[25 330 55 350]);
root=sfroot; chart=root.find('-isa','Stateflow.EMChart','Path',[plant '/dynamics']);
chart.Script=strrep(chart.Script,'function dx = fcn(u,x)', 'function dx = fcn(u,x,torque)');
% Accept spacing in the supplied function header; the mechanical equations stay intact.
if ~contains(chart.Script,'fcn(u,x,torque)')
    chart.Script=regexprep(chart.Script,'function dx\s*=\s*fcn\(u,\s*x\)', ...
        'function dx = fcn(u,x,torque)');
end
chart.Script=regexprep(chart.Script,'dx\s*=\s*tr_simp_dyn\(x,\s*u,\s*p\);',sprintf([ ...
    'dx = tr_simp_dyn(x,u,p);\n' ...
    'inertia=[p.Jph*cos(x(2))^2+p.jph*sin(x(2))^2; p.Jth];\n' ...
    'dx(3:4)=dx(3:4)+torque(:)./inertia;']));
assert(contains(chart.Script,'dx(3:4)=dx(3:4)+torque'),'Disturbance injection was not installed.');
wire(plant,'Disturbance torque/1','dynamics/3');
add_block('simulink/Sources/From Workspace',[model '/Disturbance torque'], ...
    'VariableName','Disturbance','Interpolate','off','OutputAfterFinalValue','Holding final value', ...
    'Position',[1120 800 1260 840]);
wire(model,'Disturbance torque/1','Course nonlinear plant/3');
add_block('simulink/Sinks/Out1',[plant '/True state'],'Port','10','Position',[800 340 830 360]);
wire(plant,'x/1','True state/1');
% Outer anti-windup comparison uses a workspace parameter, only in this copy.
c=[model '/Position integral control'];
gate=root.find('-isa','Stateflow.EMChart','Path',[c '/Outer anti-windup']);
gate.Script=strrep(gate.Script,'if any(', 'if OuterAW && any(');
data=Stateflow.Data(gate); data.Name='OuterAW'; data.Scope='Parameter';
add_block('simulink/Sinks/Out1',[c '/Integral output'],'Port','2','Position',[800 410 830 430]);
wire(c,'Integral contribution/1','Integral output/1');
% Dynamic voltage limits allow a temporary authority-loss/recovery test.
motor=[model '/IP motor control'];
delete_line(motor,'Raw voltage/1','Voltage limit/1');
delete_line(motor,'Voltage limit/1','Applied voltage/1');
delete_line(motor,'Voltage limit/1','Voltage mismatch/1');
position=get_param([motor '/Voltage limit'],'Position'); delete_block([motor '/Voltage limit']);
add_block('simulink/Discontinuities/Saturation Dynamic',[motor '/Voltage limit'],'Position',position);
add_block('simulink/Sources/In1',[motor '/Voltage authority'],'Port','3','Position',[25 400 55 420]);
add_block('simulink/Math Operations/Gain',[motor '/Negative limit'],'Gain','-1', ...
    'Position',[300 400 350 430]);
wire(motor,'Voltage authority/1','Voltage limit/1');
wire(motor,'Raw voltage/1','Voltage limit/2');
wire(motor,'Voltage authority/1','Negative limit/1');
wire(motor,'Negative limit/1','Voltage limit/3');
wire(motor,'Voltage limit/1','Applied voltage/1');
wire(motor,'Voltage limit/1','Voltage mismatch/1');
add_block('simulink/Sources/From Workspace',[model '/Voltage authority'], ...
    'VariableName','VoltageLimits','Interpolate','off','OutputAfterFinalValue','Holding final value', ...
    'Position',[800 800 940 840]);
wire(model,'Voltage authority/1','IP motor control/3');
add_block('simulink/Sinks/Out1',[motor '/Integral output'],'Port','3','Position',[700 300 730 320]);
wire(motor,'Integral state/1','Integral output/1');
add_block('simulink/Signal Routing/Mux',[model '/Log signals'],'Inputs','12', ...
    'Position',[2010 180 2015 900]);
add_block('simulink/Sinks/To Workspace',[model '/Recorded signals'], ...
    'VariableName','integral_data','SaveFormat','Timeseries','SampleTime','-1','MaxDataPoints','inf', ...
    'Position',[2070 470 2200 510]);
% Same first 24 columns as the Question 5 diagnostic; then J and disturbance.
sources={'Desired orientation/1','Measured angles/1','Measured rates/1', ...
    'Position integral control/1','Mechanical inverse/1','Measured rotor speeds/1', ...
    'IP motor control/2','IP motor control/1','IP motor control/3','Course nonlinear plant/10', ...
    'Position integral control/2','Disturbance torque/1'};
for k=1:numel(sources), wire(model,sources{k},sprintf('Log signals/%d',k)); end
wire(model,'Log signals/1','Recorded signals/1');
save_system(model);
Simulink.exportToVersion(model,fullfile(project.experiments,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
