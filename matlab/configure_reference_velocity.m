function configure_reference_velocity(model)
%CONFIGURE_REFERENCE_VELOCITY Add analytical course-rate feedforward to position control.
source=[model '/Desired rates']; gain=[model '/Reference velocity feedforward'];
if getSimulinkBlockHandle(source)==-1
    add_block('simulink/Sources/From Workspace',source,'VariableName','ReferenceRate', ...
        'Interpolate','on','OutputAfterFinalValue','Holding final value', ...
        'Position',[35 620 165 660]);
    add_block('simulink/Math Operations/Gain',gain,'Gain','Kff*Dq', ...
        'Position',[300 615 430 665]);
end
optional=getSimulinkBlockHandle([model '/Position integral control'])~=-1;
if optional
    set_param(gain,'Gain','Kff*Dpos');
    controller=[model '/Position integral control'];
    if getSimulinkBlockHandle([controller '/Reference velocity contribution'])==-1
        add_block('simulink/Sources/In1',[controller '/Reference velocity contribution'], ...
            'Port','6','Position',[25 610 55 630]);
        set_param([controller '/Desired acceleration'],'Inputs','+--++');
        add_line(controller,'Reference velocity contribution/1','Desired acceleration/5','autorouting','on');
    end
    destination='Position integral control/6';
else
    set_param(gain,'Gain','Kff*Dq');
    set_param([model '/Desired acceleration'],'Inputs','+-+');
    destination='Desired acceleration/3';
end
ports=get_param(gain,'PortHandles');
if get_param(ports.Inport,'Line')==-1
    add_line(model,'Desired rates/1','Reference velocity feedforward/1','autorouting','on');
end
if get_param(ports.Outport,'Line')==-1
    add_line(model,'Reference velocity feedforward/1',destination,'autorouting','on');
end
notes=find_system(model,'FindAll','on','SearchDepth',1,'Type','annotation');
for k=1:numel(notes)
    note=get_param(notes(k),'Object');
    note.Text=strrep(note.Text,'a=Kq*(q_d-q)-Dq*dq;', 'a=Kq*(q_d-q)+Dq*(dq_d-dq);');
    note.Text=strrep(note.Text,'a=Kref*q_d-Kpos*q-Dpos*dq+J;', ...
        'a=Kref*q_d-Kpos*q+Dpos*(dq_d-dq)+J;');
end
end
