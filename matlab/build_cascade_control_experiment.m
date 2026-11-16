function model = build_cascade_control_experiment
%BUILD_CASCADE_CONTROL_EXPERIMENT Diagnostic copy of the complete cascade.
project=setup_project; p=cascade_parameters;
source='aero2_cascade_control'; model=[source '_experiment'];
load_system(fullfile(project.simulink,[source '.slx']));
if bdIsLoaded(model), close_system(model,0); end
save_system(source,fullfile(project.experiments,[model '.slx']));
% Replace only the test source so arbitrary trajectories remain out of the answer.
reference=[0 0 0;p.stepTime-p.step 0 0;p.stepTime p.reference';p.stopTime p.reference'];
workspace=get_param(model,'ModelWorkspace'); assignin(workspace,'Reference',reference);
assignin(workspace,'PlantX0',p.initialState);
set_param([model '/Course nonlinear plant/x'],'InitialCondition','PlantX0');
position=get_param([model '/Desired orientation'],'Position');
delete_line(model,'Desired orientation/1','Position error/1');
delete_line(model,'Desired orientation/1','Angle scope/1');
delete_block([model '/Desired orientation']);
add_block('simulink/Sources/From Workspace',[model '/Desired orientation'], ...
    'VariableName','Reference','Interpolate','off','OutputAfterFinalValue','Holding final value','Position',position);
wire(model,'Desired orientation/1','Position error/1');
wire(model,'Desired orientation/1','Angle scope/1');
controller=[model '/IP motor control'];
for k=1:2
    names={'Raw voltage','Integral state'};
    add_block('simulink/Sinks/Out1',[controller '/Diagnostic ' names{k}], ...
        'Port',num2str(k+1),'Position',[700 160+90*k 730 180+90*k]);
    wire(controller,[names{k} '/1'],['Diagnostic ' names{k} '/1']);
end
% Expose true state only in this experiment copy; no plant equation is changed.
plant=[model '/Course nonlinear plant'];
add_block('simulink/Sinks/Out1',[plant '/True state'],'Port','8', ...
    'Position',[800 340 830 360]);
wire(plant,'x/1','True state/1');
add_block('simulink/Signal Routing/Mux',[model '/Log signals'],'Inputs','10', ...
    'Position',[2010 180 2015 820]);
add_block('simulink/Sinks/To Workspace',[model '/Recorded signals'], ...
    'VariableName','cascade_data','SaveFormat','Timeseries','SampleTime','-1', ...
    'MaxDataPoints','inf','Position',[2070 460 2200 500]);
% Pairs: qd, measured q, measured dq, a, desired omega, measured omega,
% raw voltage, applied voltage, integrator. Last six columns are true plant state.
sources={'Desired orientation/1','Measured angles/1','Measured rates/1', ...
    'Desired acceleration/1','Mechanical inverse/1','Measured rotor speeds/1', ...
    'IP motor control/2','IP motor control/1','IP motor control/3','Course nonlinear plant/8'};
for k=1:numel(sources), wire(model,sources{k},sprintf('Log signals/%d',k)); end
wire(model,'Log signals/1','Recorded signals/1');
save_system(model);
Simulink.exportToVersion(model,fullfile(project.experiments,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
