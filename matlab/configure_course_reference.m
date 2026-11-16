function configure_course_reference(model,experiment)
%CONFIGURE_COURSE_REFERENCE Use the supplied 60 s trajectory as the saved default.
% Preserve all destinations, controller blocks and existing Scope styling.
if nargin<2, experiment=false; end
[U_v,U_h,reference]=aero2_course_reference;
workspace=get_param(model,'ModelWorkspace');
source=[model '/Desired orientation'];
position=get_param(source,'Position'); handles=get_param(source,'PortHandles');
line=get_param(handles.Outport(1),'Line'); destinations=get_param(line,'DstPortHandle');
paths=cell(numel(destinations),1);
for k=1:numel(destinations)
    parent=get_param(destinations(k),'Parent'); port=get_param(destinations(k),'PortNumber');
    paths{k}=[parent(numel(model)+2:end) '/' num2str(port)];
end
delete_line(line); delete_block(source);
if experiment
    % Validation can override this matrix with signed steps or other test cases.
    assignin(workspace,'Reference',reference);
    add_block('simulink/Sources/From Workspace',source, ...
        'VariableName','Reference','Interpolate','on','OutputAfterFinalValue','Holding final value', ...
        'Position',position);
else
    % Keep the course's original U_v/U_h names visible inside the source block.
    assignin(workspace,'U_v',U_v); assignin(workspace,'U_h',U_h);
    add_block('simulink/Ports & Subsystems/Subsystem',source,'Position',position);
    Simulink.SubSystem.deleteContents(source);
    add_block('simulink/Sources/From Workspace',[source '/Course yaw reference'], ...
        'VariableName','U_v','Interpolate','on','OutputAfterFinalValue','Holding final value', ...
        'Position',[30 60 180 100]);
    add_block('simulink/Sources/From Workspace',[source '/Course pitch reference'], ...
        'VariableName','U_h','Interpolate','on','OutputAfterFinalValue','Holding final value', ...
        'Position',[30 160 180 200]);
    add_block('simulink/Signal Routing/Mux',[source '/Yaw then pitch'],'Inputs','2', ...
        'Position',[250 80 255 180]);
    add_block('simulink/Sinks/Out1',[source '/Desired angles'], ...
        'Position',[320 120 350 140]);
    add_line(source,'Course yaw reference/1','Yaw then pitch/1','autorouting','on');
    add_line(source,'Course pitch reference/1','Yaw then pitch/2','autorouting','on');
    add_line(source,'Yaw then pitch/1','Desired angles/1','autorouting','on');
end
for k=1:numel(paths), add_line(model,'Desired orientation/1',paths{k},'autorouting','on'); end
set_param(model,'StopTime','60');
scopes=find_system(model,'LookUnderMasks','all','BlockType','Scope');
for k=1:numel(scopes)
    config=get_param(scopes{k},'ScopeConfiguration'); config.TimeSpan='60';
end
notes=find_system(model,'FindAll','on','Type','annotation');
for k=1:numel(notes)
    note=get_param(notes(k),'Object');
    text=regexprep(note.Text,'q_d=\[0\.2;0\.1\] rad at 0\.2 s[^\n]*', ...
        'Course U_v/U_h trajectory; Ts=0.002 s; stop 60 s; zero initial state; ideal sensors');
    note.Text=regexprep(text,'Default yaw/pitch \[0\.2;0\.1\] rad at 0\.2 s[^\n]*', ...
        'Course U_v/U_h trajectory; Ts=0.002 s; stop 60 s; zero initial state; ideal sensors');
end
end
