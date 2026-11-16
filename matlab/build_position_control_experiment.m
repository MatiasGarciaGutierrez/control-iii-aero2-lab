function model = build_position_control_experiment
%BUILD_POSITION_CONTROL_EXPERIMENT Add diagnostics to the simple Question 4 model.
project=setup_project; p=position_control_parameters;
source='aero2_position_control'; model=[source '_experiment'];
if ~isfile(fullfile(project.simulink,[source '.slx'])), build_position_control; end
if bdIsLoaded(model), close_system(model,0); end
load_system(fullfile(project.simulink,[source '.slx']));
save_system(source,fullfile(project.experiments,[model '.slx']));
add_block('simulink/Math Operations/Sum',[model '/Acceleration residual'], ...
    'Inputs','+-','Position',[1340 600 1370 645]);
wire(model,'Desired acceleration/1','Acceleration residual/1');
wire(model,'Ideal-speed course mechanics/2','Acceleration residual/2');
scopes={'Error scope','Rate scope','Acceleration scope','Rotor command scope','Residual scope'};
units={'Position error [rad]','Angular velocity [rad/s]','Acceleration [rad/s^2]', ...
    'Rotor speed [rad/s]','Acceleration residual [rad/s^2]'};
for k=1:5
    y=340+120*(k-1);
    if getSimulinkBlockHandle([model '/' scopes{k}])==-1
        add_block('simulink/Sinks/Scope',[model '/' scopes{k}], ...
            'NumInputPorts',num2str(1+(k==3)),'Position',[1510 y 1590 y+65]);
    end
    config=get_param([model '/' scopes{k}],'ScopeConfiguration');
    config.TimeSpan=num2str(p.stopTime); config.YLabel=units{k};
    config.ShowLegend=true; config.ShowGrid=true;
end
wire(model,'Acceleration residual/1','Residual scope/1');
add_block('simulink/Signal Routing/Mux',[model '/Log signals'], ...
    'Inputs','8','Position',[1740 220 1745 850]);
add_block('simulink/Sinks/To Workspace',[model '/Recorded signals'], ...
    'VariableName','position_data','SaveFormat','Timeseries', ...
    'SampleTime','-1','MaxDataPoints','inf','Position',[1810 480 1940 530]);
% Paired columns: reference, q, dq, a, main/tail omega, error, recovered ddq, residual.
signals={'Desired orientation/1','Angles and rates/1','Angles and rates/2', ...
    'Desired acceleration/1','Mechanical inverse/1','Position error/1', ...
    'Ideal-speed course mechanics/2','Acceleration residual/1'};
for k=1:numel(signals), wire(model,signals{k},sprintf('Log signals/%d',k)); end
wire(model,'Log signals/1','Recorded signals/1');
configure_course_reference(model,true);
save_system(model);
Simulink.exportToVersion(model,fullfile(project.experiments,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
