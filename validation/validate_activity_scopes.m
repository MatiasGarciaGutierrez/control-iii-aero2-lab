function inventory = validate_activity_scopes
%VALIDATE_ACTIVITY_SCOPES Audit passive views, actual accelerations and exports.
project=setup_project;
models={'aero2_simulator','aero2_motor_pi','aero2_motor_ip', ...
    'aero2_mechanical_linearization','aero2_position_control', ...
    'aero2_cascade_control','aero2_position_integral'};
required={{'Voltage scope','Rotor speed scope','Body angle scope','Rate scope','Acceleration scope','Motor acceleration scope'}, ...
    {'Speed scope','Voltage scope','Error scope','Motor acceleration scope'}, ...
    {'Speed scope','Voltage scope','Error scope','Motor acceleration scope','Raw voltage scope','Integral scope'}, ...
    {'Angle scope','Rate scope','Acceleration scope'}, ...
    {'Angle scope','Rate scope','Acceleration scope','Error scope','Rotor command scope'}, ...
    {'Angle scope','Rate scope','Acceleration scope','Error scope','Rotor speed scope','Voltage scope', ...
    'Motor acceleration scope','IP motor control/Raw voltage scope','IP motor control/Integral scope'}, ...
    {'Angle scope','Rate scope','Acceleration scope','Rotor speed scope','Voltage scope','Motor acceleration scope', ...
    'IP motor control/Raw voltage scope','IP motor control/Integral scope', ...
    'Position integral control/Error scope','Position integral control/Integral scope'}};
inventory=table;
for activity=0:6
    model=models{activity+1}; load_system(fullfile(project.simulink,[model '.slx']));
    cleanup=onCleanup(@() close_system(model,0));
    scopes=required{activity+1};
    for k=1:numel(scopes)
        path=[model '/' scopes{k}];
        assert(strcmp(get_param(path,'BlockType'),'Scope'),'Required activity scope is missing.');
        ports=get_param(path,'PortHandles');
        for j=1:numel(ports.Inport)
            port=ports.Inport(j);
            assert(get_param(port,'Line')~=-1,'An activity scope has a disconnected input.');
        end
    end
    if activity==3 || activity==4
        rates='Angles and rates';
        sources={ [rates '/1'],[rates '/2'],'Mechanical inverse/1', ...
            scopeSource(model,'Acceleration scope',2),scopeSource(model,'Acceleration scope',1)};
        width=10;
    else
        baseSources={'Course nonlinear plant/1','Course nonlinear plant/3', ...
            'Course nonlinear plant/2','Course nonlinear plant/4', ...
            'Course nonlinear plant/5','Course nonlinear plant/6'};
        if activity==0
            voltage={'Main voltage limit/1','Tail voltage limit/1'};
        elseif activity<3
            voltage={'Main voltage limit/1','Tail voltage limit/1'};
        else
            voltage={'Motor voltages/1','Motor voltages/2'};
        end
        if activity==1 || activity==2
            sources=[baseSources voltage {scopeSource(model,'Motor acceleration scope',1)}]; width=10;
        else
            port=2; if activity==0, port=1; end
            sources=[baseSources voltage {scopeSource(model,'Acceleration scope',port), ...
                scopeSource(model,'Motor acceleration scope',1)}]; width=12;
        end
    end
    add_block('simulink/Signal Routing/Mux',[model '/Temporary audit mux'],'Inputs',num2str(numel(sources)));
    add_block('simulink/Sinks/To Workspace',[model '/Temporary audit log'], ...
        'VariableName','scope_audit','SaveFormat','Timeseries','SampleTime','-1');
    for k=1:numel(sources), add_line(model,sources{k},sprintf('Temporary audit mux/%d',k),'autorouting','on'); end
    add_line(model,'Temporary audit mux/1','Temporary audit log/1');
    output=sim(model); data=matrix(output.scope_audit,width);
    % Independent checks ensure the monitors show body acceleration and rotor
    % acceleration in the correct order rather than another state component.
    if activity==3 || activity==4
        residual=max(abs(data(:,7:8)-data(:,9:10)),[],'all');
    else
        motorActual=data(:,end-1:end);
        expected=-5.78*data(:,5:6)+96.532*data(:,7:8);
        residual=max(abs(motorActual-expected),[],'all');
        if activity~=1 && activity~=2
            for sample=1:20:size(data,1)
                [M,c,g,Gamma]=aero2_mechanical_terms(data(sample,1:2)',data(sample,3:4)');
                omega=data(sample,5:6)'; expected=M\(Gamma*(omega.*abs(omega))-c-g);
                residual=max(residual,max(abs(expected-data(sample,9:10)')));
            end
        end
    end
    assert(residual<1e-10,'An acceleration monitor disagrees with the plant equations.');
    clear cleanup;
    % Open and simulate the export as a separate model, not an actual R2021a runtime.
    exported=[model '_R2021a']; load_system(fullfile(project.simulink,[exported '.slx'])); sim(exported);
    row=table(activity,{[model '.slx']},numel(scopes),residual, ...
        'VariableNames',{'Activity','ModelFile','RequiredScopes','MaxAccelerationResidual'});
    inventory=[inventory;row]; %#ok<AGROW>
end
if ~isfolder(project.results), mkdir(project.results); end
writetable(inventory,fullfile(project.results,'activity_scope_inventory.csv')); disp(inventory);
fprintf('PASS: all activity scopes are connected; accelerations match equations; exports run in R2024b.\n');
end

function data=matrix(signal,width)
if signal.IsTimeFirst
    data=reshape(signal.Data,numel(signal.Time),width);
else
    data=reshape(signal.Data,width,[])';
end
assert(all(isfinite(data(:))),'Scope audit produced nonfinite signals.');
end

function source=scopeSource(model,name,index)
ports=get_param([model '/' name],'PortHandles');
line=get_param(ports.Inport(index),'Line'); src=get_param(line,'SrcPortHandle');
block=get_param(src,'Parent'); number=get_param(src,'PortNumber');
source=[block(numel(model)+2:end) '/' num2str(number)];
end
