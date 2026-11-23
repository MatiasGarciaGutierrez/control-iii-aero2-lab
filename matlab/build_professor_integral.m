function model = build_professor_integral
%BUILD_PROFESSOR_INTEGRAL Weighted position integral law on the same cascade.
project=setup_project;p=position_integral_parameters;
source='aero2_cascade_professor_format';model='aero2_integral_professor_format';
load_system(fullfile(project.simulink,[source '.slx']));
if bdIsLoaded(model),close_system(model,0);end
save_system(source,fullfile(project.simulink,[model '.slx']));
w=get_param(model,'ModelWorkspace');
assignin(w,'Kp',p.positionGain);assignin(w,'Kd',p.rateGain);
assignin(w,'Ki',p.integralGain);assignin(w,'b',p.referenceWeight);
% Freeze both outer integrators whenever either motor voltage is limited.
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/Outer integration gate'], ...
    'Position',[390 650 570 750]);
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/Outer integration gate']);
chart.VectorOutputs1D=true;
chart.Script=sprintf(['function gate = integration_gate(rawMain,rawTail,main,tail)\n' ...
    'gate=double(abs(rawMain-main)<=1e-8 && abs(rawTail-tail)<=1e-8);\nend\n']);
sources={'IP_m','IP_t','+//- 24V1','+//- 24V'};
for k=1:4,connect(model,sources{k},1,'Outer integration gate',k);end
loops={'Subsystem3','Subsystem2'};
for axis=1:2
    loop=[model '/' loops{axis}];
    add_block('simulink/Math Operations/Gain',[loop '/Reference weight'], ...
        'Gain','b','Position',[90 180 145 215]);
    add_block('simulink/Math Operations/Sum',[loop '/Weighted position error'], ...
        'Inputs','+-','Position',[170 180 195 215]);
    connect(loop,'ref',1,'Reference weight',1);
    connect(loop,'Reference weight',1,'Weighted position error',1);
    connect(loop,'angle',1,'Weighted position error',2);
    connect(loop,'Weighted position error',1,'Gain',1);
    connect(loop,'Sum2',1,'Gain2',1);
    add_block('simulink/Sources/In1',[loop '/Integration gate'], ...
        'Port','4','Position',[30 260 60 280]);
    add_block('simulink/Math Operations/Product',[loop '/Gated integral rate'], ...
        'Inputs','2','Position',[265 235 295 275]);
    connect(loop,'Gain2',1,'Gated integral rate',1);
    connect(loop,'Integration gate',1,'Gated integral rate',2);
    connect(loop,'Gated integral rate',1,'Integrator',1);
    connect(model,'Outer integration gate',1,loops{axis},4);
end
notes=find_system(model,'FindAll','on','Type','annotation');
for k=1:numel(notes),delete(get_param(notes(k),'Object'));end
note=Simulink.Annotation(model,sprintf([ ...
    'Optional integral cascade: same plant, tachometer filters and motor IP loops as Question 5.\n' ...
    'a=Kp*(b*q_d-q)+Kd*(dq_d-dq)+J; J_dot=Ki*(q_d-q).\n' ...
    'b=1/3; both outer integrators pause when either motor voltage saturates.']));
note.Position=[-800 720 300 810];set_param(model,'Description',note.Text);
set_param(model,'SimulationCommand','update');save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end

function connect(system,source,sourcePort,destination,destinationPort)
from=get_param([system '/' source],'PortHandles');to=get_param([system '/' destination],'PortHandles');
line=get_param(to.Inport(destinationPort),'Line');if line~=-1,delete_line(line);end
add_line(system,from.Outport(sourcePort),to.Inport(destinationPort),'autorouting','on');
end
