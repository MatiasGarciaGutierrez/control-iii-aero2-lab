function connect_hardware_cascade(model,interface)
%CONNECT_HARDWARE_CASCADE Explicitly reconnect all top-level controller ports.
% Replacing a subsystem's contents invalidates its old external port handles.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
while ~isempty(lines)
    delete_line(lines(1));
    lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
end
wire(model,'Desired orientation/1','Position error/1');
wire(model,'Measured angles/1','Position error/2');
wire(model,'Position error/1','Position gain/1');
wire(model,'Position gain/1','Desired acceleration/1');
wire(model,'Measured rates/1','Rate feedback gain/1');
wire(model,'Rate feedback gain/1','Desired acceleration/2');
wire(model,'Desired rates/1','Reference velocity feedforward/1');
wire(model,'Reference velocity feedforward/1','Desired acceleration/3');
wire(model,'Measured angles/1','Mechanical inverse/1');
wire(model,'Measured rates/1','Mechanical inverse/2');
wire(model,'Desired acceleration/1','Mechanical inverse/3');
wire(model,'Mechanical inverse/1','IP motor control/1');
wire(model,'Measured rotor speeds/1','IP motor control/2');
wire(model,'IP motor control/1','Motor voltages/1');
for k=1:2
    wire(model,sprintf('Motor voltages/%d',k),sprintf('%s/%d',interface,k));
end
wire(model,'LED color/1',[interface '/3']);
wire(model,[interface '/1'],'Measured angles/1'); % raw yaw
wire(model,[interface '/4'],'Measured angles/2'); % raw pitch
wire(model,[interface '/3'],'Measured rates/1');  % estimated yaw rate
wire(model,[interface '/5'],'Measured rates/2');  % estimated pitch rate
wire(model,[interface '/7'],'Main tachometer filter/1'); % channel 14000
wire(model,[interface '/8'],'Tail tachometer filter/1'); % channel 14001
wire(model,'Main tachometer filter/1','Measured rotor speeds/1');
wire(model,'Tail tachometer filter/1','Measured rotor speeds/2');
wire(model,[interface '/2'],'Unused filtered yaw/1');
wire(model,[interface '/6'],'Unused filtered pitch/1');
wire(model,[interface '/9'],'Unused currents/1');
wire(model,'Desired orientation/1','Angle scope/1');
wire(model,'Measured angles/1','Angle scope/2');
wire(model,'Measured rates/1','Rate scope/1');
wire(model,'Mechanical inverse/1','Rotor speed scope/1');
wire(model,'Measured rotor speeds/1','Rotor speed scope/2');
wire(model,'IP motor control/1','Voltage scope/1');
wire(model,'Position error/1','Error scope/1');
wire(model,'Measured rates/1','Angular acceleration estimate/1');
wire(model,'Measured rotor speeds/1','Motor acceleration estimate/1');
wire(model,'Desired acceleration/1','Acceleration scope/1');
wire(model,'Angular acceleration estimate/1','Acceleration scope/2');
wire(model,'Motor acceleration estimate/1','Motor acceleration scope/1');
blocks=find_system(model,'SearchDepth',1,'Type','Block');
for k=1:numel(blocks)
    ports=get_param(blocks{k},'PortHandles');
    for port=ports.Inport
        line=get_param(port,'Line');
        assert(line~=-1 && get_param(line,'SrcPortHandle')~=-1, ...
            'Unconnected cascade input: %s',blocks{k});
    end
end
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
