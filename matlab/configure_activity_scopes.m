function configure_activity_scopes(model,activity)
%CONFIGURE_ACTIVITY_SCOPES Add passive views appropriate to each lab activity.
stop=get_param(model,'StopTime');
if activity==1 || activity==2
    for k=1:2
        names={'Main','Tail'}; n=names{k};
        scope(model,'Error scope',2,[1350 520 1430 580],'Motor speed error [rad/s]',stop);
        connect(model,[n ' speed error/1'],sprintf('Error scope/%d',k));
        if activity==2
            scope(model,'Raw voltage scope',2,[1350 630 1430 690],'Raw motor voltage [V]',stop);
            scope(model,'Integral scope',2,[1350 740 1430 800],'Motor integral contribution [V]',stop);
            connect(model,[n ' raw voltage/1'],sprintf('Raw voltage scope/%d',k));
            connect(model,[n ' integral state/1'],sprintf('Integral scope/%d',k));
        end
    end
    exposeAcceleration(model,false);
    scope(model,'Motor acceleration scope',1,[1350 410 1430 470],'Rotor acceleration [rad/s^2]',stop);
    connect(model,'Course nonlinear plant/8','Motor acceleration scope/1');
elseif activity==4
    % Retain the two-output mechanical wrapper for direct, noise-free acceleration.
    root=sfroot; chart=root.find('-isa','Stateflow.EMChart','Path',[model '/Ideal-speed course mechanics']);
    if ~contains(chart.Script,'function [dx, acceleration]')
        chart.Script=strrep(chart.Script,'function dx = ideal_mechanics','function [dx, acceleration] = ideal_mechanics');
        chart.Script=strrep(chart.Script,'dx = fullDerivative(1:4);', ...
            sprintf('dx = fullDerivative(1:4);\nacceleration = fullDerivative(3:4);'));
    end
    scope(model,'Rate scope',1,[1510 350 1590 410],'Body angular velocity [rad/s]',stop);
    scope(model,'Acceleration scope',2,[1510 480 1590 540],'Body acceleration [rad/s^2]',stop);
    scope(model,'Error scope',1,[1510 610 1590 670],'Position error [rad]',stop);
    scope(model,'Rotor command scope',1,[1510 740 1590 800],'Desired rotor speed [rad/s]',stop);
    connect(model,'Angles and rates/2','Rate scope/1');
    connect(model,'Desired acceleration/1','Acceleration scope/1');
    connect(model,'Ideal-speed course mechanics/2','Acceleration scope/2');
    connect(model,'Position error/1','Error scope/1');
    connect(model,'Mechanical inverse/1','Rotor command scope/1');
elseif activity==0 || activity==5 || activity==6
    exposeAcceleration(model,true);
    if activity==0
        scope(model,'Rate scope',2,[960 640 1060 700],'Body angular velocity [rad/s]',stop);
        connect(model,'Course nonlinear plant/2','Rate scope/1');
        connect(model,'Course nonlinear plant/4','Rate scope/2');
        scope(model,'Acceleration scope',1,[960 750 1060 810],'Actual body acceleration [rad/s^2]',stop);
        connect(model,'Course nonlinear plant/8','Acceleration scope/1');
        motorPosition=[960 860 1060 920];
    else
        scope(model,'Rate scope',1,[1770 610 1850 670],'Measured body angular velocity [rad/s]',stop);
        scope(model,'Acceleration scope',2,[1770 720 1850 780],'Body acceleration [rad/s^2]',stop);
        connect(model,'Measured rates/1','Rate scope/1');
        source='Desired acceleration/1';
        if activity==6, source='Position integral control/1'; end
        connect(model,source,'Acceleration scope/1');
        connect(model,'Course nonlinear plant/8','Acceleration scope/2');
        motorPosition=[1770 830 1850 890];
        motor=[model '/IP motor control'];
        scope(motor,'Raw voltage scope',1,[850 160 940 220],'Raw motor voltage [V]',stop);
        scope(motor,'Integral scope',1,[850 300 940 360],'Motor integral contribution [V]',stop);
        connect(motor,'Raw voltage/1','Raw voltage scope/1');
        connect(motor,'Integral state/1','Integral scope/1');
        if activity==5
            scope(model,'Error scope',1,[1770 940 1850 1000],'Position error [rad]',stop);
            connect(model,'Position error/1','Error scope/1');
        else
            controller=[model '/Position integral control'];
            scope(controller,'Error scope',1,[950 230 1040 290],'Position error [rad]',stop);
            scope(controller,'Integral scope',1,[950 410 1040 470],'Outer integral contribution [rad/s^2]',stop);
            connect(controller,'Position error/1','Error scope/1');
            connect(controller,'Integral contribution/1','Integral scope/1');
        end
    end
    scope(model,'Motor acceleration scope',1,motorPosition,'Actual rotor acceleration [rad/s^2]',stop);
    connect(model,'Course nonlinear plant/9','Motor acceleration scope/1');
end
end

function exposeAcceleration(model,body)
% Observe the existing derivative output; do not edit the plant's equations.
plant=[model '/Course nonlinear plant'];
if getSimulinkBlockHandle([plant '/Motor acceleration'])~=-1, return; end
if body
    outputs='[2 2 2]'; ports=[2 3]; names={'Body acceleration','Motor acceleration'};
else
    outputs='[4 2]'; ports=2; names={'Motor acceleration'};
end
add_block('simulink/Signal Routing/Demux',[plant '/Motion derivatives'], ...
    'Outputs',outputs,'Position',[440 360 445 440]);
connect(plant,'dynamics/1','Motion derivatives/1');
add_block('simulink/Sinks/Terminator',[plant '/Unused derivative components'], ...
    'Position',[510 350 530 370]);
connect(plant,'Motion derivatives/1','Unused derivative components/1');
for k=1:numel(ports)
    add_block('simulink/Sinks/Out1',[plant '/' names{k}],'Port',num2str(7+k), ...
        'Position',[800 380+70*k 830 400+70*k]);
    connect(plant,sprintf('Motion derivatives/%d',ports(k)),[names{k} '/1']);
end
end

function scope(parent,name,ports,position,units,stop)
path=[parent '/' name];
if getSimulinkBlockHandle(path)==-1
    add_block('simulink/Sinks/Scope',path,'NumInputPorts',num2str(ports),'Position',position);
else
    set_param(path,'Position',position,'NumInputPorts',num2str(ports));
end
config=get_param(path,'ScopeConfiguration');
config.YLabel=units; config.TimeSpan=stop; config.ShowLegend=true; config.ShowGrid=true;
end

function connect(parent,source,destination)
parts=strsplit(destination,'/'); port=str2double(parts{end});
block=strjoin(parts(1:end-1),'/'); handles=get_param([parent '/' block],'PortHandles');
if get_param(handles.Inport(port),'Line')==-1
    add_line(parent,source,destination,'autorouting','on');
end
end
