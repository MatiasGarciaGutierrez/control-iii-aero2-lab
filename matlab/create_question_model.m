function model = create_question_model(experiment, question)
%CREATE_QUESTION_MODEL Keep exercise diagrams free of experiment instrumentation.
project=setup_project;
models={'aero2_motor_pi','aero2_motor_ip','aero2_mechanical_linearization'};
model=models{question};
if bdIsLoaded(model), close_system(model,0); end
save_system(experiment,fullfile(project.simulink,[model '.slx']));
% Remove all top-level wires before reconnecting the minimal diagram.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
while ~isempty(lines)
    delete_line(lines(1));
    lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
end
notes=find_system(model,'FindAll','on','Type','annotation');
for k=1:numel(notes), delete(get_param(notes(k),'Object')); end
if question<3
    keep={'Course nonlinear plant','Speed scope','Voltage scope'};
    stems={'speed reference','speed error','proportional','integral gain', ...
        'integral state','raw voltage','voltage limit'};
    if question==2
        stems=[stems,{'voltage mismatch','anti-windup gain','integral derivative'}];
    end
    names={'Main','Tail'};
    for k=1:2
        for j=1:numel(stems), keep{end+1}=[names{k} ' ' stems{j}]; end %#ok<AGROW>
    end
    removeOthers(model,keep);
    set_param([model '/Course nonlinear plant'],'Position',[850 180 1070 490]);
    set_param([model '/Speed scope'],'Position',[1180 180 1260 260]);
    set_param([model '/Voltage scope'],'Position',[1180 380 1260 440]);
    for k=1:2
        n=names{k}; y=180+(k-1)*(280+100*(question==2));
        set_param([model '/' n ' speed reference'],'Position',[40 y 90 y+40]);
        set_param([model '/' n ' speed error'],'Position',[160 y 185 y+40]);
        set_param([model '/' n ' proportional'],'Position',[270 y-55 330 y-15]);
        set_param([model '/' n ' integral gain'],'Position',[270 y+65 330 y+105]);
        set_param([model '/' n ' integral state'],'Position',[495 y+65 535 y+105]);
        set_param([model '/' n ' raw voltage'],'Position',[620 y 645 y+40]);
        set_param([model '/' n ' voltage limit'],'Position',[730 y 790 y+40]);
        wire(model,[n ' speed reference/1'],[n ' speed error/1']);
        wire(model,sprintf('Course nonlinear plant/%d',k+4),[n ' speed error/2']);
        wire(model,[n ' speed error/1'],[n ' integral gain/1']);
        if question==1
            wire(model,[n ' speed error/1'],[n ' proportional/1']);
            wire(model,[n ' integral gain/1'],[n ' integral state/1']);
        else
            % Explicit IP: proportional action only on measured speed.
            set_param([model '/' n ' raw voltage'],'Inputs','-+');
            wire(model,sprintf('Course nonlinear plant/%d',k+4),[n ' proportional/1']);
            set_param([model '/' n ' integral derivative'],'Position',[400 y+65 425 y+105]);
            set_param([model '/' n ' voltage mismatch'],'Position',[730 y+180 755 y+220]);
            set_param([model '/' n ' anti-windup gain'],'Position',[500 y+180 560 y+220], ...
                'Orientation','left');
            wire(model,[n ' integral gain/1'],[n ' integral derivative/1']);
            wire(model,[n ' voltage limit/1'],[n ' voltage mismatch/1']);
            wire(model,[n ' raw voltage/1'],[n ' voltage mismatch/2']);
            wire(model,[n ' voltage mismatch/1'],[n ' anti-windup gain/1']);
            wire(model,[n ' anti-windup gain/1'],[n ' integral derivative/2']);
            wire(model,[n ' integral derivative/1'],[n ' integral state/1']);
        end
        wire(model,[n ' proportional/1'],[n ' raw voltage/1']);
        wire(model,[n ' integral state/1'],[n ' raw voltage/2']);
        wire(model,[n ' raw voltage/1'],[n ' voltage limit/1']);
        wire(model,[n ' voltage limit/1'],sprintf('Course nonlinear plant/%d',k));
        wire(model,[n ' speed reference/1'],sprintf('Speed scope/%d',2*k-1));
        wire(model,sprintf('Course nonlinear plant/%d',k+4),sprintf('Speed scope/%d',2*k));
        wire(model,[n ' voltage limit/1'],sprintf('Voltage scope/%d',k));
    end
    % Terminate only unused plant outputs, keeping the original plant intact.
    for k=[1 2 3 4 7]
        name=sprintf('Unused output %d',k);
        add_block('simulink/Sinks/Terminator',[model '/' name], ...
            'Position',[1100 500+23*k 1120 520+23*k],'ShowName','off');
        wire(model,sprintf('Course nonlinear plant/%d',k),[name '/1']);
    end
    title=sprintf('Question %d: %s motor-speed control',question,upper(models{question}(13:end)));
    if question==2
        ws=get_param(model,'ModelWorkspace'); evalin(ws,'clear Beta');
        title='Question 2: IP with anti-windup (same Kp and Ki as Question 1)';
    end
else
    removeOthers(model,{'Mechanical inverse','Ideal-speed course mechanics', ...
        'Mechanical state','Angles and rates','Angle scope','Rate scope','Acceleration scope'});
    root=sfroot;
    inverse=root.find('-isa','Stateflow.EMChart','Path',[model '/Mechanical inverse']);
    inverse.Script=strrep(inverse.Script,'function [omegaDesired, s]', 'function omegaDesired');
    ws=get_param(model,'ModelWorkspace'); assignin(ws,'X0',zeros(4,1));
    add_block('simulink/Sources/Step',[model '/Virtual acceleration a'], ...
        'Time','0.2','Before','[0;0]','After','[0.15;-0.1]','SampleTime','0', ...
        'Position',[40 190 110 240]);
    set_param([model '/Mechanical inverse'],'Position',[270 150 450 280]);
    set_param([model '/Ideal-speed course mechanics'],'Position',[570 170 780 270]);
    set_param([model '/Mechanical state'],'Position',[870 190 920 240]);
    set_param([model '/Angles and rates'],'Position',[870 390 875 470]);
    set_param([model '/Angle scope'],'Position',[1050 190 1130 250]);
    set_param([model '/Acceleration scope'],'Position',[1050 300 1130 360]);
    set_param([model '/Rate scope'],'Position',[1050 440 1130 500]);
    wire(model,'Virtual acceleration a/1','Mechanical inverse/3');
    wire(model,'Mechanical inverse/1','Ideal-speed course mechanics/2');
    wire(model,'Mechanical state/1','Ideal-speed course mechanics/1');
    wire(model,'Ideal-speed course mechanics/1','Mechanical state/1');
    wire(model,'Mechanical state/1','Angles and rates/1');
    wire(model,'Angles and rates/1','Mechanical inverse/1');
    wire(model,'Angles and rates/2','Mechanical inverse/2');
    wire(model,'Angles and rates/1','Angle scope/1');
    wire(model,'Angles and rates/2','Rate scope/1');
    wire(model,'Virtual acceleration a/1','Acceleration scope/1');
    wire(model,'Ideal-speed course mechanics/2','Acceleration scope/2');
    scopes={'Angle scope','Rate scope','Acceleration scope'};
    units={'Angle [rad]','Body angular velocity [rad/s]','Body angular acceleration [rad/s^2]'};
    for k=1:numel(scopes)
        config=get_param([model '/' scopes{k}],'ScopeConfiguration');
        config.ShowLegend=true; config.ShowGrid=true; config.TimeSpan='2'; config.YLabel=units{k};
    end
    title='Question 3: feedback linearization, ideal rotor tracking; a is acceleration';
end
note=Simulink.Annotation(model,title); note.Position=[40 30 1100 80];
set_param(model,'SignalLogging','off');
save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end

function removeOthers(model,keep)
blocks=find_system(model,'SearchDepth',1,'Type','Block');
for k=1:numel(blocks)
    if ~ismember(get_param(blocks{k},'Name'),keep), delete_block(blocks{k}); end
end
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
