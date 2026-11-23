function validate_professor_mechanics
%VALIDATE_PROFESSOR_MECHANICS Check Questions 3-4 after layout adaptation.
project=setup_project;
models={'aero2_linearization_professor_format','aero2_position_professor_format'};
oldModels={'aero2_mechanical_linearization','aero2_position_control'};
defaults=cell(2,1); signedRuns=cell(2,2); comparisons=zeros(2,1);
residuals=zeros(2,1);settlingTimes=zeros(2,2);
for question=3:4
    model=models{question-2}; load_system(fullfile(project.simulink,[model '.slx']));
    assert(isempty(find_system(model,'BlockType','Saturate')));
    assert(isempty(find_system(model,'BlockType','TransferFcn')));
    experiment=[model '_experiment'];
    if bdIsLoaded(experiment),close_system(experiment,0);end
    save_system(model,fullfile(project.experiments,[experiment '.slx']));
    plant=[experiment '/Twin rotor (simplified model)'];
    record(plant,'Mechanical state',1,'state');
    record(plant,'Mechanical dynamics',2,'acceleration');
    record(experiment,'Virtual accelerations',1,'command');
    save_system(experiment); defaults{question-2}=sim(experiment);
    r=defaults{question-2}; residual=max(abs(data(r.acceleration)-data(r.command)),[],'all');
    residuals(question-2)=residual;
    assert(residual<1e-10,'Mechanical cancellation failed.');
    assert(all(isfinite(data(r.state)),'all'));
    % Logging belongs to the experiment copy; temporarily instrument the
    % simple answer and its legacy export, then discard those additions.
    load_system(fullfile(project.simulink,[model '.slx']));
    record([model '/Twin rotor (simplified model)'],'Mechanical state',1,'state');
    simple=sim(model);
    assert(max(abs(data(simple.state)-data(r.state)),[],'all')<1e-12);
    close_system(model,0);
    legacy=[model '_R2021a'];load_system(fullfile(project.simulink,[legacy '.slx']));
    record([legacy '/Twin rotor (simplified model)'],'Mechanical state',1,'state');
    out=sim(legacy);
    assert(max(abs(data(out.state)-data(r.state)),[],'all')<1e-12);
    close_system(legacy,0);
    if question==4
        % Match the old model's analytical reference derivative in memory.
        % The saved answer retains the professor's Derivative blocks.
        analyticRates(experiment);
    end
    old=oldModels{question-2};load_system(fullfile(project.simulink,[old '.slx']));
    record(old,'Mechanical state',1,'state'); previous=sim(old);
    matched=sim(experiment);
    comparisons(question-2)=max(abs(data(previous.state)-data(matched.state)),[],'all');
    assert(comparisons(question-2)<1e-7,'Adaptation changed matched plant dynamics.');
    close_system(old,0);
    for direction=1:2
        signValue=3-2*direction; input=Simulink.SimulationInput(experiment);
        if question==3
            initial=[0.1;0.2;-0.05;0.04]; acceleration=signValue*[0.15;-0.1];
            input=input.setVariable('X0',initial,'Workspace',experiment);
            names={'Yaw virtual acceleration','Pitch virtual acceleration'};
            for axis=1:2
                input=input.setBlockParameter([experiment '/' names{axis}], ...
                    'After',num2str(acceleration(axis),17));
            end
            out=sim(input);t=out.state.Time; elapsed=max(t-0.2,0);
            expected=[initial(1:2)'+t*initial(3:4)'+0.5*elapsed.^2*acceleration' ...
                initial(3:4)'+elapsed*acceleration'];
            assert(max(abs(data(out.state)-expected),[],'all')<1e-3);
        else
            target=signValue*[0.2;0.1];
            [input,reference]=stepInput(input,experiment,target);
            out=sim(input);t=out.state.Time; x=data(out.state);
            pole=position_control_parameters; pole=pole.pole;
            elapsed=max(t-0.2,0); expected=(1-(1+pole*elapsed).*exp(-pole*elapsed))*target';
            assert(max(abs(x(:,1:2)-expected),[],'all')<5e-4);
            for axis=1:2
                selected=t>=0.2; times=t(selected)-0.2;
                last=find(abs(x(selected,axis)-target(axis))>0.02*abs(target(axis)),1,'last');
                settling=times(last+1);
                settlingTimes(direction,axis)=settling;
                assert(abs(settling-3)<0.02,'Position settling misses the 3 s target.');
                assert(max(signValue*x(:,axis))<=abs(target(axis))+1e-10);
            end
        end
        assert(max(abs(data(out.acceleration)-data(out.command)),[],'all')<1e-10);
        signedRuns{question-2,direction}=out;
    end
    close_system(experiment,0);
    fprintf('Question %d: cancellation, signed inputs, equivalence and export checks passed.\n',question);
end
folder=fullfile(project.results,'professor_mechanics');if ~isfolder(folder),mkdir(folder);end
save(fullfile(folder,'validation.mat'),'defaults','signedRuns','comparisons','residuals','settlingTimes');
disp('Matched maximum state differences:');disp(comparisons);
disp('Default acceleration residuals:');disp(residuals);
disp('Positive/negative yaw/pitch settling times [s]:');disp(settlingTimes);
export_professor_mechanics_figures;
end

function record(system,source,port,variable)
name=['Record ' variable];
add_block('simulink/Sinks/To Workspace',[system '/' name], ...
    'VariableName',variable,'SaveFormat','Timeseries','MaxDataPoints','inf');
from=get_param([system '/' source],'PortHandles');to=get_param([system '/' name],'PortHandles');
add_line(system,from.Outport(port),to.Inport(1),'autorouting','on');
end

function matrix=data(signal)
matrix=reshape(signal.Data,numel(signal.Time),[]);
end

function analyticRates(model)
project=setup_project;[~,~,~,rates]=aero2_course_reference;
w=get_param(model,'ModelWorkspace');
for axis=1:2
    loops={'Subsystem3','Subsystem2'};loop=[model '/' loops{axis}];
    block=[loop '/Derivative']; position=get_param(block,'Position');
    ports=get_param(block,'PortHandles');
    line=get_param(ports.Outport,'Line');to=get_param(line,'DstPortHandle');
    delete_line(line);delete_line(get_param(ports.Inport,'Line'));delete_block(block);
    variable=sprintf('AnalyticRate%d',axis);assignin(w,variable,[rates(:,1) rates(:,axis+1)]);
    add_block('simulink/Sources/From Workspace',block,'VariableName',variable, ...
        'Interpolate','on','OutputAfterFinalValue','Holding final value','Position',position);
    from=get_param(block,'PortHandles');add_line(loop,from.Outport(1),to);
end
end

function [input,reference]=stepInput(input,model,target)
reference=[0 0 0;0.198 0 0;0.2 target';8 target'];
for axis=1:2
    variables={'U_v','U_h'};
    input=input.setVariable(variables{axis},timeseries(reference(:,axis+1),reference(:,1)), ...
        'Workspace',model);
    input=input.setVariable(sprintf('AnalyticRate%d',axis),[0 0;8 0],'Workspace',model);
end
sources=find_system(model,'SearchDepth',1,'BlockType','FromWorkspace');
for k=1:numel(sources)
    input=input.setBlockParameter(sources{k},'Interpolate','off');
    input=input.setBlockParameter(sources{k},'OutputAfterFinalValue','Holding final value');
end
input=input.setModelParameter('StopTime','8');
end
