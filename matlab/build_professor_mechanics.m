function models = build_professor_mechanics
%BUILD_PROFESSOR_MECHANICS Questions 3-4 in the professor's signal-flow layout.
project=setup_project; p=position_control_parameters;
source='aero2_cascade_professor_format';
load_system(fullfile(project.simulink,[source '.slx']));
original='aero2_mechanical_linearization';
load_system(fullfile(project.simulink,[original '.slx']));
chart=find(sfroot,'-isa','Stateflow.EMChart', ...
    'Path',[original '/Ideal-speed course mechanics']);
plantScript=chart.Script;
models={'aero2_linearization_professor_format','aero2_position_professor_format'};
for question=3:4
    model=models{question-2};
    if bdIsLoaded(model),close_system(model,0);end
    load_system(fullfile(project.simulink,[source '.slx']));
    save_system(source,fullfile(project.simulink,[model '.slx']));
    w=get_param(model,'ModelWorkspace'); assignin(w,'X0',zeros(4,1));
    assignin(w,'Kp',p.kp); assignin(w,'Kd',p.kd); assignin(w,'Ki',0);
    % Only Question 5 introduces finite-bandwidth motors and voltage limits.
    for name={'IP_m','IP_t','+//- 24V','+//- 24V1','Transfer Fcn1','Transfer Fcn3'}
        delete_block([model '/' name{1}]);
    end
    plant=[model '/Twin rotor (simplified model)'];
    mask=Simulink.Mask.get(plant); if ~isempty(mask),delete(mask);end
    Simulink.SubSystem.deleteContents(plant);
    makePlant(plant,plantScript);
    set_param(plant,'ContentPreviewEnabled','off');
    % Preserve the professor's rotor ordering:
    % Demux output 1 -> Subsystem1 (main), output 2 -> Subsystem (tail).
    connect(model,'Subsystem1',1,'Twin rotor (simplified model)',1);
    connect(model,'Subsystem',1,'Twin rotor (simplified model)',2);
    tags={'Goto3','Goto5','Goto1','Goto2','Goto','Goto4'};
    for k=1:6,connect(model,'Twin rotor (simplified model)',k,tags{k},1);end
    connect(model,'Twin rotor (simplified model)',1,'Mux2',2);
    connect(model,'Twin rotor (simplified model)',3,'Mux3',2);
    inverse=find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/MATLAB Function']);
    script=inverse.Script; first=find(script==newline,1);
    inverse.Script=[script(1:first) ...
        'assert(abs(theta)<=pi/3,''Pitch envelope exceeded (limit 60 deg).'');' ...
        newline script(first+1:end)];
    notes=find_system(model,'FindAll','on','Type','annotation');
    for k=1:numel(notes),delete(get_param(notes(k),'Object'));end
    if question==3
        for name={'Subsystem2','Subsystem3','From4','From7','From9','From10'}
            delete_block([model '/' name{1}]);
        end
        names={'Yaw virtual acceleration','Pitch virtual acceleration'};
        for axis=1:2
            sources=find_system(model,'SearchDepth',1,'BlockType','FromWorkspace');
            variable={'U_v','U_h'};
            index=find(cellfun(@(b)strcmp(get_param(b,'VariableName'),variable{axis}),sources));
            old=sources{index}; position=get_param(old,'Position');
            ports=get_param(old,'PortHandles'); delete_line(get_param(ports.Outport,'Line'));
            delete_block(old);
            acceleration=[0.15 -0.1];
            add_block('simulink/Sources/Step',[model '/' names{axis}], ...
                'Time','0.2','Before','0','After',num2str(acceleration(axis)), ...
                'Position',position);
            connect(model,names{axis},1,'MATLAB Function',axis+3);
        end
        % Position scopes display angles only, not acceleration references.
        for axis=1:2
            muxes={'Mux2','Mux3'}; ports=get_param([model '/' muxes{axis}],'PortHandles');
            for k=1:numel(ports.Inport)
                line=get_param(ports.Inport(k),'Line'); if line~=-1,delete_line(line);end
            end
            set_param([model '/' muxes{axis}],'Inputs','1');
            connect(model,'Twin rotor (simplified model)',2*axis-1,muxes{axis},1);
        end
        set_param(model,'StopTime','2'); duration='2';
        noteText=['Question 3: virtual acceleration -> inverse -> signed square roots -> ideal mechanics.' newline ...
            'Step a=[0.15;-0.10] rad/s^2 at 0.2 s; x(0)=0; stop 2 s.' newline ...
            'Open Acceleration (rad_s2) to compare a with actual body acceleration.'];
    else
        duration='60';
        noteText=['Question 4: PD position loops -> inverse -> signed square roots -> ideal mechanics.' newline ...
            'Original smooth course references; x(0)=0; stop 60 s. Kp=3.7816, Kd=3.8893, Ki=0.' newline ...
            'Open Position (rad) for reference/angle tracking.'];
    end
    delete_block([model '/Terminator']);
    add_block('simulink/Signal Routing/Mux',[model '/Virtual accelerations'], ...
        'Inputs','2','Position',[-140 560 -135 630]);
    if question==3,inputs=names;else,inputs={'Subsystem3','Subsystem2'};end
    for axis=1:2,connect(model,inputs{axis},1,'Virtual accelerations',axis);end
    add_block('simulink/Sinks/Scope',[model '/Acceleration (rad_s2)'], ...
        'NumInputPorts','2','Position',[200 570 280 630]);
    connect(model,'Virtual accelerations',1,'Acceleration (rad_s2)',1);
    connect(model,'Twin rotor (simplified model)',7,'Acceleration (rad_s2)',2);
    cleanLines(model);
    scopes=find_system(model,'BlockType','Scope');
    for k=1:numel(scopes)
        config=get_param(scopes{k},'ScopeConfiguration');
        config.TimeSpan=duration; config.ShowLegend=true; config.ShowGrid=true;
    end
    note=Simulink.Annotation(model,[noteText newline ...
        'Ideal rotor tracking: no motor loops, voltage saturation or sensor noise. Pitch envelope +/-60 deg.']);
    note.Position=[-800 700 600 800];
    set_param(model,'Description',note.Text);
    set_param(model,'SimulationCommand','update');
    save_system(model);
    Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end
end

function makePlant(system,script)
for axis=1:2
    names={'Main rotor speed','Tail rotor speed'};
    add_block('simulink/Sources/In1',[system '/' names{axis}], ...
        'Port',num2str(axis),'Position',[30 50+80*axis 60 70+80*axis]);
end
add_block('simulink/Signal Routing/Mux',[system '/Rotor speeds'], ...
    'Inputs','2','Position',[120 140 125 235]);
add_block('simulink/User-Defined Functions/MATLAB Function',[system '/Mechanical dynamics'], ...
    'Position',[210 135 380 220]);
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',[system '/Mechanical dynamics']);
chart.Script=script; chart.VectorOutputs1D=true;
add_block('simulink/Continuous/Integrator',[system '/Mechanical state'], ...
    'InitialCondition','X0','Position',[450 140 490 180]);
add_block('simulink/Signal Routing/Demux',[system '/State outputs'], ...
    'Outputs','[1 1 1 1]','Position',[550 110 555 275]);
wire(system,'Main rotor speed/1','Rotor speeds/1');
wire(system,'Tail rotor speed/1','Rotor speeds/2');
wire(system,'Rotor speeds/1','Mechanical dynamics/2');
wire(system,'Mechanical state/1','Mechanical dynamics/1');
wire(system,'Mechanical dynamics/1','Mechanical state/1');
wire(system,'Mechanical state/1','State outputs/1');
names={'Yaw','Yaw rate','Pitch','Pitch rate','Main speed','Tail speed','Body acceleration'};
signals={'State outputs/1','State outputs/3','State outputs/2','State outputs/4', ...
    'Main rotor speed/1','Tail rotor speed/1','Mechanical dynamics/2'};
for k=1:7
    add_block('simulink/Sinks/Out1',[system '/' names{k}], ...
        'Port',num2str(k),'Position',[680 40+50*k 710 60+50*k]);
    wire(system,signals{k},[names{k} '/1']);
end
end

function connect(system,source,sourcePort,destination,destinationPort)
from=get_param([system '/' source],'PortHandles');
to=get_param([system '/' destination],'PortHandles');
line=get_param(to.Inport(destinationPort),'Line');
if line~=-1,delete_line(line);end
add_line(system,from.Outport(sourcePort),to.Inport(destinationPort),'autorouting','on');
end

function wire(system,source,destination)
add_line(system,source,destination,'autorouting','on');
end

function cleanLines(model)
changed=true;
while changed
    changed=false; lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
    for k=numel(lines):-1:1
        if ishandle(lines(k)) && isempty(get_param(lines(k),'LineChildren')) && ...
                (all(get_param(lines(k),'DstPortHandle')==-1) || get_param(lines(k),'SrcPortHandle')==-1)
            delete_line(lines(k));changed=true;
        end
    end
end
end
