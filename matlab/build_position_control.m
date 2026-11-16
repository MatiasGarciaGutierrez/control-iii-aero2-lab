function model = build_position_control
%BUILD_POSITION_CONTROL Question 4: simple ideal-speed position-control answer.
project=setup_project; p=position_control_parameters;
source='aero2_mechanical_linearization'; model='aero2_position_control';
if bdIsLoaded(model), close_system(model,0); end
load_system(fullfile(project.simulink,[source '.slx']));
save_system(source,fullfile(project.simulink,[model '.slx']));
% Reuse the validated inverse and original ideal-speed course mechanics.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
while ~isempty(lines)
    delete_line(lines(1));
    lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
end
delete_block([model '/Virtual acceleration a']);
notes=find_system(model,'FindAll','on','Type','annotation');
for k=1:numel(notes), delete(get_param(notes(k),'Object')); end
workspace=get_param(model,'ModelWorkspace');
assignin(workspace,'X0',p.initialState);
assignin(workspace,'Kq',p.kp); assignin(workspace,'Dq',p.kd);
set_param(model,'StopTime',num2str(p.stopTime),'FixedStep',num2str(p.step));
add_block('simulink/Sources/Step',[model '/Desired orientation'], ...
    'Time',num2str(p.stepTime),'Before','[0;0]','After',mat2str(p.reference), ...
    'SampleTime','0','Position',[35 210 105 260]);
add_block('simulink/Math Operations/Sum',[model '/Position error'], ...
    'Inputs','+-','Position',[195 210 220 255]);
add_block('simulink/Math Operations/Gain',[model '/Position gain'], ...
    'Gain','Kq','Position',[305 205 375 255]);
add_block('simulink/Math Operations/Gain',[model '/Rate feedback gain'], ...
    'Gain','Dq','Orientation','left','Position',[305 440 375 490]);
add_block('simulink/Math Operations/Sum',[model '/Desired acceleration'], ...
    'Inputs','+-','Position',[470 205 495 250]);
set_param([model '/Mechanical inverse'],'Position',[620 175 830 300]);
set_param([model '/Ideal-speed course mechanics'],'Position',[945 190 1170 290]);
set_param([model '/Mechanical state'],'Position',[1260 210 1310 260]);
set_param([model '/Angles and rates'],'Position',[1260 440 1265 530]);
set_param([model '/Angle scope'],'Position',[1435 205 1520 280],'NumInputPorts','2');
config=get_param([model '/Angle scope'],'ScopeConfiguration');
config.TimeSpan=num2str(p.stopTime); config.YLabel='Angle [rad]';
wire(model,'Desired orientation/1','Position error/1');
wire(model,'Angles and rates/1','Position error/2');
wire(model,'Position error/1','Position gain/1');
wire(model,'Position gain/1','Desired acceleration/1');
wire(model,'Angles and rates/2','Rate feedback gain/1');
wire(model,'Rate feedback gain/1','Desired acceleration/2');
wire(model,'Desired acceleration/1','Mechanical inverse/3');
wire(model,'Angles and rates/1','Mechanical inverse/1');
wire(model,'Angles and rates/2','Mechanical inverse/2');
wire(model,'Mechanical inverse/1','Ideal-speed course mechanics/2');
wire(model,'Mechanical state/1','Ideal-speed course mechanics/1');
wire(model,'Ideal-speed course mechanics/1','Mechanical state/1');
wire(model,'Mechanical state/1','Angles and rates/1');
wire(model,'Desired orientation/1','Angle scope/1');
wire(model,'Angles and rates/1','Angle scope/2');
note=Simulink.Annotation(model,sprintf([ ...
    'Question 4: AERO 2 yaw/pitch position control, ideal rotor tracking\n' ...
    'a=Kq*(q_d-q)-Dq*dq; Kq=%.7f, Dq=%.7f; 2%% settling target 3 s\n' ...
    'q_d=[0.2;0.1] rad at 0.2 s; x(0)=0; inherited +/-60 deg pitch simulation envelope'],p.kp,p.kd));
note.Position=[35 25 1400 120];
save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
