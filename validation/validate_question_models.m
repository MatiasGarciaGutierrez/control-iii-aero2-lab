function checks = validate_question_models
%VALIDATE_QUESTION_MODELS Verify simple answers against their experiment copies.
project=setup_project;
models={'aero2_motor_pi','aero2_motor_ip','aero2_mechanical_linearization','aero2_position_control'};
checks=table;
for question=1:4
    model=models{question};
    load_system(fullfile(project.simulink,[model '.slx']));
    cleanup=onCleanup(@() close_system(model,0));
    assert(isempty(find_system(model,'SearchDepth',1,'BlockType','ToWorkspace')), ...
        'An exercise model contains experiment logging.');
    if question<3
        assert(isempty(find_system(model,'SearchDepth',1,'Name','Main recovery change')), ...
            'An exercise model contains a recovery experiment.');
        sources=cell(1,14); names={'Main','Tail'};
        for k=1:2
            n=names{k};
            pair={[n ' speed reference/1'],sprintf('Course nonlinear plant/%d',k+4), ...
                [n ' speed error/1'],[n ' raw voltage/1'],[n ' voltage limit/1'], ...
                [n ' integral state/1']};
            for j=1:6, sources{2*(j-1)+k}=pair{j}; end
        end
        sources(13:14)={'Course nonlinear plant/1','Course nonlinear plant/3'};
        columns=1:14;
        if question==2
            assert(isempty(find_system(model,'SearchDepth',1,'Name','Main reference weight')), ...
                'The simple IP model contains the PI/IP experiment selector.');
        end
    elseif question==3
        sources={'Virtual acceleration a/1','Angles and rates/1', ...
            'Angles and rates/2','Mechanical inverse/1'};
        columns=[1:2 5:10];
    else
        sources={'Desired orientation/1','Angles and rates/1','Angles and rates/2', ...
            'Desired acceleration/1','Mechanical inverse/1','Position error/1'};
        columns=1:12;
    end
    % Export the uninstrumented exercise diagram first.
    print(['-s' model],'-dpng','-r150', ...
        fullfile(project.docs,'figures',sprintf('question_%d_diagram.png',question)));
    % Logging exists only in memory for this check; never save this instrumented model.
    add_block('simulink/Signal Routing/Mux',[model '/Temporary validation mux'], ...
        'Inputs',num2str(numel(sources)));
    add_block('simulink/Sinks/To Workspace',[model '/Temporary validation log'], ...
        'VariableName','equivalence_data','SaveFormat','Timeseries','SampleTime','-1');
    for k=1:numel(sources)
        add_line(model,sources{k},sprintf('Temporary validation mux/%d',k),'autorouting','on');
    end
    add_line(model,'Temporary validation mux/1','Temporary validation log/1');
    simple=sim(model); simpleData=asMatrix(simple.equivalence_data,numel(columns));
    experiment=[model '_experiment'];
    load_system(fullfile(project.experiments,[experiment '.slx']));
    input=Simulink.SimulationInput(experiment);
    if question==3
        input=input.setVariable('X0',zeros(4,1),'Workspace',experiment);
        names={'Yaw virtual acceleration','Pitch virtual acceleration'};
        for k=1:2
            input=input.setBlockParameter([experiment '/' names{k}],'PulseWidth','99');
        end
    end
    output=sim(input);
    if question<3
        data=output.motor_data;
    elseif question==3
        data=output.linearization_data;
    else
        data=output.position_data;
    end
    width=14; if question==4, width=16; end
    reference=asMatrix(data,width);
    error=max(abs(simpleData-reference(:,columns)),[],'all');
    assert(isequal(simple.equivalence_data.Time,data.Time),'Simulation time vectors differ.');
    assert(error<1e-10,'Simple answer changed the controller or plant behavior.');
    row=table(question,error,'VariableNames',{'Question','MaxDifference'});
    checks=[checks;row]; %#ok<AGROW>
    clear cleanup; % Discard temporary instrumentation before continuing.
end
if ~isfolder(project.results), mkdir(project.results); end
writetable(checks,fullfile(project.results,'question_model_equivalence.csv'));
disp(checks);
fprintf('PASS: simple models match experiment dynamics, with no saved experiment clutter.\n');
end

function data=asMatrix(signal,width)
if signal.IsTimeFirst
    data=reshape(signal.Data,numel(signal.Time),width);
else
    data=reshape(signal.Data,width,[])';
end
assert(all(isfinite(data(:))),'A simulation produced nonfinite data.');
end
