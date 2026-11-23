function export_professor_report_details
%EXPORT_PROFESSOR_REPORT_DETAILS Readable appendix insets from saved models.
project=setup_project;
names={'aero2_linearization_professor_format','aero2_position_professor_format', ...
    'aero2_integral_professor_format'};
paths={'Twin rotor (simplified model)','Subsystem3','Subsystem3'};
files={'professor_format_ideal_plant.png','professor_format_position_loop.png', ...
    'professor_format_integral_loop.png'};
for k=1:numel(names)
    load_system(fullfile(project.simulink,[names{k} '.slx']));
    if k>1,spaceController([names{k} '/' paths{k}],k==3);end
    file=fullfile(project.docs,'figures',files{k});
    print(['-s' names{k} '/' paths{k}],'-dpng','-r180',file);
    copyfile(file,fullfile(project.root,'report','images',files{k}));
end
% Execute the newly added R2021a integral export in the installed release.
% This does not establish execution in R2021a itself.
model=names{3};legacy=[model '_R2021a'];
load_system(fullfile(project.simulink,[legacy '.slx']));
for name={model,legacy}
    plant=[name{1} '/Twin rotor (simplified model)'];
    add_block('simulink/Sinks/To Workspace',[plant '/Export check'], ...
        'VariableName','export_state','SaveFormat','Timeseries');
    add_line(plant,'x/1','Export check/1');
end
a=sim(model);b=sim(legacy);
assert(max(abs(a.export_state.Data-b.export_state.Data),[],'all')<1e-10);
close_system(model,0);close_system(legacy,0);
fprintf('R2021a integral export agrees with the saved R2024b model.\n');
end

function spaceController(system,integral)
% Widen the native detail export without changing any block or connection.
blocks=find_system(system,'SearchDepth',1,'Type','block');edges=[];
for k=1:numel(blocks)
    if strcmp(blocks{k},system),continue;end
    ports=get_param(blocks{k},'PortHandles');
    for port=ports.Inport(:)'
        line=get_param(port,'Line');
        if line~=-1,edges(end+1,:)=[get_param(line,'SrcPortHandle') port];end %#ok<AGROW>
    end
end
lines=find_system(system,'FindAll','on','SearchDepth',1,'Type','line');
while ~isempty(lines)
    delete_line(lines(1));lines=find_system(system,'FindAll','on','SearchDepth',1,'Type','line');
end
names={'ref','angle','dangle','Derivative','de','Gain1','Gain2','Integrator','Sum3','v'};
positions={[30 55 60 75],[30 135 60 155],[30 225 60 245], ...
    [170 145 220 185],[290 145 315 185],[380 145 450 185], ...
    [260 250 330 290],[490 250 530 290],[620 60 650 280],[720 160 750 180]};
for k=1:numel(names),set_param([system '/' names{k}],'Position',positions{k});end
if integral
    set_param([system '/Reference weight'],'Position',[150 45 220 85]);
    set_param([system '/Weighted position error'],'Position',[280 45 305 85]);
    set_param([system '/Gain'],'Position',[380 45 450 85]);
    set_param([system '/Sum2'],'Position',[150 250 175 290]);
    set_param([system '/Integration gate'],'Position',[30 335 60 355]);
    set_param([system '/Gated integral rate'],'Position',[380 250 415 290]);
else
    set_param([system '/Sum2'],'Position',[170 45 195 85]);
    set_param([system '/Gain'],'Position',[380 45 450 85]);
end
for k=1:numel(blocks)
    type=get_param(blocks{k},'BlockType');
    if any(strcmp(type,{'Gain','Sum','Product','Integrator','Derivative'}))
        set_param(blocks{k},'ShowName','off');
    end
end
for k=1:size(edges,1),add_line(system,edges(k,1),edges(k,2),'autorouting','on');end
for k=1:size(edges,1)
    line=get_param(edges(k,2),'Line');assert(get_param(line,'SrcPortHandle')==edges(k,1));
end
end
