function model = build_motor_ip
%BUILD_MOTOR_IP Extend the saved PI model with IP selection and anti-windup.
project = setup_project; p = motor_ip_parameters;
source = 'aero2_motor_pi'; model = 'aero2_motor_ip';
if bdIsLoaded(model), close_system(model, 0); end
load_system(fullfile(project.simulink, [source '.slx']));
% Save As creates a separate working model; it does not change the PI file.
save_system(source, fullfile(project.simulink, [model '.slx']));
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'Beta', p.beta);
assignin(workspace, 'Kaw', p.awGain);
notes = find_system(model, 'FindAll', 'on', 'Type', 'annotation');
for k=1:numel(notes), delete(get_param(notes(k), 'Object')); end
names = {'Main', 'Tail'};
for k=1:2
    n=names{k}; y=180+(k-1)*380;
    % Move existing loop blocks to make the two new feedback paths visible.
    blocks = {'speed reference',[40 y 90 y+40]; 'speed error',[290 y 315 y+40]; ...
        'proportional',[530 y-85 595 y-45]; 'integral gain',[410 y+60 475 y+100]; ...
        'integral state',[670 y+60 710 y+100]; 'raw voltage',[800 y 825 y+40]; ...
        'voltage limit',[930 y 990 y+40]};
    for j=1:size(blocks,1)
        set_param([model '/' n ' ' blocks{j,1}], 'Position', blocks{j,2});
    end
    % Combine the nominal step and an optional return step for recovery tests.
    delete_line(model, [n ' speed reference/1'], [n ' speed error/1']);
    delete_line(model, [n ' speed reference/1'], sprintf('Log signals/%d',k));
    delete_line(model, [n ' speed reference/1'], sprintf('Speed scope/%d',2*k-1));
    add_block('simulink/Sources/Step', [model '/' n ' recovery change'], ...
        'Time', num2str(p.recoveryTime), 'Before','0','After','0','SampleTime','0', ...
        'Position',[40 y+85 90 y+125]);
    add_block('simulink/Math Operations/Sum',[model '/' n ' combined reference'], ...
        'Inputs','++','Position',[175 y 200 y+40]);
    wire(model,[n ' speed reference/1'],[n ' combined reference/1']);
    wire(model,[n ' recovery change/1'],[n ' combined reference/2']);
    wire(model,[n ' combined reference/1'],[n ' speed error/1']);
    wire(model,[n ' combined reference/1'],sprintf('Log signals/%d',k));
    wire(model,[n ' combined reference/1'],sprintf('Speed scope/%d',2*k-1));
    % Two-degree-of-freedom PI form: Kp*(Beta*w_d-w) + I.
    delete_line(model,[n ' speed error/1'],[n ' proportional/1']);
    add_block('simulink/Math Operations/Gain',[model '/' n ' reference weight'], ...
        'Gain','Beta','Position',[290 y-110 355 y-70]);
    add_block('simulink/Math Operations/Sum',[model '/' n ' proportional error'], ...
        'Inputs','+-','Position',[430 y-85 455 y-45]);
    wire(model,[n ' combined reference/1'],[n ' reference weight/1']);
    wire(model,[n ' reference weight/1'],[n ' proportional error/1']);
    wire(model,sprintf('Course nonlinear plant/%d',k+4),[n ' proportional error/2']);
    wire(model,[n ' proportional error/1'],[n ' proportional/1']);
    % Back-calculation uses the voltage actually passed through the +/-24 V limit.
    delete_line(model,[n ' integral gain/1'],[n ' integral state/1']);
    add_block('simulink/Math Operations/Sum',[model '/' n ' voltage mismatch'], ...
        'Inputs','+-','Position',[960 y+190 985 y+230]);
    add_block('simulink/Math Operations/Gain',[model '/' n ' anti-windup gain'], ...
        'Gain','Kaw','Position',[710 y+190 775 y+230]);
    add_block('simulink/Math Operations/Sum',[model '/' n ' integral derivative'], ...
        'Inputs','++','Position',[560 y+60 585 y+100]);
    wire(model,[n ' voltage limit/1'],[n ' voltage mismatch/1']);
    wire(model,[n ' raw voltage/1'],[n ' voltage mismatch/2']);
    wire(model,[n ' voltage mismatch/1'],[n ' anti-windup gain/1']);
    wire(model,[n ' anti-windup gain/1'],[n ' integral derivative/2']);
    wire(model,[n ' integral gain/1'],[n ' integral derivative/1']);
    wire(model,[n ' integral derivative/1'],[n ' integral state/1']);
end
set_param([model '/Course nonlinear plant'],'Position',[1120 240 1360 590]);
existing={'Speed scope','Error scope','Voltage scope'};
for k=1:3
    set_param([model '/' existing{k}],'Position',[1500 130+140*(k-1) 1580 200+140*(k-1)]);
end
extra={'Raw voltage scope','Integrator scope'};
for k=1:2
    add_block('simulink/Sinks/Scope',[model '/' extra{k}], 'NumInputPorts','2', ...
        'Position',[1500 550+140*(k-1) 1580 620+140*(k-1)]);
    config=get_param([model '/' extra{k}],'ScopeConfiguration');
    config.ShowLegend=true; config.ShowGrid=true; config.TimeSpan='1'; config.YLabel='Voltage [V]';
    for j=1:2
        blocks={'raw voltage','integral state'};
        wire(model,[names{j} ' ' blocks{k} '/1'],sprintf('%s/%d',extra{k},j));
    end
end
set_param([model '/Log signals'],'Position',[1730 150 1735 810]);
set_param([model '/Recorded signals'],'Position',[1800 460 1930 500]);
for k=[2 4 7]
    set_param(sprintf('%s/Unused output %d',model,k),'Position',[1400 800+25*k 1420 820+25*k]);
end
note=Simulink.Annotation(model,sprintf([ ...
    'Question 2: IP with back-calculation anti-windup; same Kp and Ki as Question 1\n' ...
    'Model workspace: Beta=0 IP, Beta=1 PI; Kaw=20 enabled, Kaw=0 disabled\n' ...
    'v_raw=I+Kp*(Beta*w_d-w); I_dot=Ki*(w_d-w)+Kaw*(v_sat-v_raw)\n' ...
    'Default +/-100 rad/s at 0.1 s; recovery change defaults to zero; ideal sensors; step 0.002 s']));
note.Position=[40 20 1380 110];
save_system(model);
Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end

function wire(model, source, destination)
add_line(model,source,destination,'autorouting','on');
end
