function model = build_hardware_cascade_offline
%BUILD_HARDWARE_CASCADE_OFFLINE No QUARC, HIL, USB, or physical outputs.
% Tests the EXACT hardware-model controller against the supplied course plant.
project=setup_project; source='aero2_hardware_cascade';
model='aero2_hardware_cascade_offline';
load_system(fullfile(project.simulink,[source '.slx']));
if bdIsLoaded(model), close_system(model,0); end
save_system(source,fullfile(project.experiments,[model '.slx']));
% Remove every hardware/library reference BEFORE any model update or simulation.
delete_block([model '/HIL Initialize']);
interface=[model '/AERO 2 hardware'];
Simulink.SubSystem.deleteContents(interface);
config=Simulink.ConfigSet; config.Name='Offline simulation';
old=getActiveConfigSet(model); oldName=old.Name;
attachConfigSet(model,config); setActiveConfigSet(model,config.Name);
detachConfigSet(model,oldName);
set_param(model,'SolverType','Fixed-step','Solver','ode1','FixedStep','0.002', ...
    'StopTime','60','ReturnWorkspaceOutputs','on','SaveTime','on', ...
    'UnconnectedInputMsg','error','UnconnectedOutputMsg','error');
set_param(interface,'Name','Offline AERO 2 surrogate');
interface=[model '/Offline AERO 2 surrogate'];
course='model_AERO2_simul_R2021a';
load_system(fullfile(project.course,[course '.slx']));
add_block([course '/Twin rotor (simplified model)'],[interface '/Course plant'], ...
    'Position',[150 80 330 290]);
for k=1:3
    names={'Main voltage','Tail voltage','Unused LED'};
    add_block('simulink/Sources/In1',[interface '/' names{k}], ...
        'Port',num2str(k),'Position',[30 70+70*k 60 90+70*k]);
end
wire(interface,'Main voltage/1','Course plant/1');
wire(interface,'Tail voltage/1','Course plant/2');
add_block('simulink/Sinks/Terminator',[interface '/LED terminator'],'Position',[90 280 110 300]);
wire(interface,'Unused LED/1','LED terminator/1');
for k=1:2
    names={'Yaw','Pitch'}; n=names{k};
    % Nominal transfer-function equivalent of the template: wn=10 rad/s,
    % zeta=0.8, x0=0. This does not execute the actual QUARC library block.
    add_block('simulink/Continuous/Transfer Fcn',[interface '/' n ' filtered'], ...
        'Numerator','100','Denominator','[1 16 100]', ...
        'Position',[420 30+190*k 520 60+190*k]);
    add_block('simulink/Continuous/Transfer Fcn',[interface '/' n ' rate'], ...
        'Numerator','[100 0]','Denominator','[1 16 100]', ...
        'Position',[420 90+190*k 520 120+190*k]);
    signal=sprintf('Course plant/%d',2*k-1);
    wire(interface,signal,[n ' filtered/1']); wire(interface,signal,[n ' rate/1']);
    add_block('simulink/Sinks/Terminator',[interface '/' n ' unused true rate'], ...
        'Position',[365 80+40*k 385 100+40*k]);
    wire(interface,sprintf('Course plant/%d',2*k),[n ' unused true rate/1']);
end
names={'Yaw','Filtered yaw','Estimated yaw rate','Pitch','Estimated pitch rate', ...
    'Filtered pitch','Main motor speed','Tail motor speed','Motor currents'};
signals={'Course plant/1','Yaw filtered/1','Yaw rate/1','Course plant/3', ...
    'Pitch rate/1','Pitch filtered/1','Course plant/5','Course plant/6','Course plant/7'};
for k=1:9
    add_block('simulink/Sinks/Out1',[interface '/' names{k}], ...
        'Port',num2str(k),'Position',[650 30+50*k 680 50+50*k]);
    wire(interface,signals{k},[names{k} '/1']);
end
controller=[model '/IP motor control'];
for k=1:2
    blocks={'Integral state','Raw voltage'};
    variables={'hardware_integral_data','hardware_raw_voltage_data'};
    add_block('simulink/Sinks/To Workspace',[controller '/' variables{k}], ...
        'VariableName',variables{k},'SaveFormat','Timeseries', ...
        'MaxDataPoints','inf','Position',[1040 500+70*k 1190 540+70*k]);
    wire(controller,[blocks{k} '/1'],[variables{k} '/1']);
end
connect_hardware_cascade(model,'Offline AERO 2 surrogate');
% Log the newly connected controller/plant signals.
% Diagnostic logging is confined to this offline experiment copy.
sources={'Desired orientation/1','Measured angles/1','Measured rates/1', ...
    'Desired acceleration/1','Mechanical inverse/1','Measured rotor speeds/1', ...
    'IP motor control/1'};
add_block('simulink/Signal Routing/Mux',[model '/Offline log signals'], ...
    'Inputs',num2str(numel(sources)),'Position',[1950 1320 1955 1530]);
add_block('simulink/Sinks/To Workspace',[model '/Offline data'], ...
    'VariableName','hardware_offline_data','SaveFormat','Timeseries', ...
    'MaxDataPoints','inf','Position',[2070 1390 2220 1430]);
for k=1:numel(sources), wire(model,sources{k},sprintf('Offline log signals/%d',k)); end
wire(model,'Offline log signals/1','Offline data/1');
notes=find_system(model,'FindAll','on','SearchDepth',1,'Type','annotation');
for k=1:numel(notes), delete(get_param(notes(k),'Object')); end
note=Simulink.Annotation(model,sprintf([ ...
    'OFFLINE ONLY: same controller as aero2_hardware_cascade; NO hardware blocks\n' ...
    'Course nonlinear plant + equivalent 10 rad/s body-rate estimator + 180 rad/s tachometer filters\n' ...
    'This checks controller behavior and filter delay, not actual hardware or QUARC timing.']));
note.Position=[30 10 1190 115];
set_param(model,'SimulationCommand','update');
save_system(model);
Simulink.exportToVersion(model,fullfile(project.experiments,[model '_R2021a.slx']),'R2021a');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
