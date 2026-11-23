function metrics = validate_course_reference
%VALIDATE_COURSE_REFERENCE Exact course inputs and default 60 s tracking runs.
project=setup_project; [U_v,U_h,reference,referenceRate]=aero2_course_reference;
expected=originalReference(fullfile(project.course,'model_AERO2_simul_ini.m'));
assert(isequal(reference,expected),'Reference generator differs from the original course initializer.');
assert(isequal(U_v.Time,U_h.Time) && isequal(reference(:,2:3),[U_v.Data U_h.Data]), ...
    'Course timeseries order or scaling is incorrect.');
numericalRate=(reference(3:end,2:3)-reference(1:end-2,2:3))/0.004;
smooth=true(size(numericalRate,1),1);
for transition=[2 5 15 25 32 40 45]
    smooth=smooth & abs(reference(2:end-1,1)-transition)>0.004;
end
rateError=numericalRate-referenceRate(2:end-1,2:3);
assert(max(abs(rateError(smooth,:)),[],'all')<1e-6, ...
    'Analytical reference velocity differs from an independent centered difference.');
assert(max(abs(cumtrapz(referenceRate(:,1),referenceRate(:,2:3))-reference(:,2:3)),[],'all')<1e-6, ...
    'Reference velocity does not integrate to the original course angles.');
models={'aero2_position_control','aero2_cascade_control','aero2_position_integral'};
widths=[16 24 28]; runs=cell(3,1); noisyRuns=cell(3,1); metrics=table;
comparisonRuns=cell(3,2,2);
for k=1:3
    model=models{k}; load_system(fullfile(project.simulink,[model '.slx']));
    w=get_param(model,'ModelWorkspace');
    storedYaw=evalin(w,'U_v'); storedPitch=evalin(w,'U_h');
    assert(isequal(storedYaw.Time,U_v.Time) && isequal(storedPitch.Time,U_h.Time) && ...
        isequal(storedYaw.Data,U_v.Data) && isequal(storedPitch.Data,U_h.Data), ...
        'A saved model does not contain the course reference.');
    assert(strcmp(get_param(model,'StopTime'),'60'),'A position model stops before the full trajectory.');
    assert(strcmp(get_param([model '/Desired orientation/Course yaw reference'],'VariableName'),'U_v'));
    assert(strcmp(get_param([model '/Desired orientation/Course pitch reference'],'VariableName'),'U_h'));
    scopes=find_system(model,'LookUnderMasks','all','BlockType','Scope');
    for j=1:numel(scopes)
        config=get_param(scopes{j},'ScopeConfiguration');
        assert(strcmp(config.TimeSpan,'60'),'A Scope hides part of the course trajectory.');
    end
    % Temporary logging verifies that the simple answer model has the same
    % default course response as its diagnostic copy; never save these blocks.
    add_block('simulink/Signal Routing/Mux',[model '/Baseline log'],'Inputs','2');
    add_block('simulink/Sinks/To Workspace',[model '/Baseline recording'], ...
        'VariableName','baseline_data','SaveFormat','Timeseries','MaxDataPoints','inf');
    add_line(model,'Desired orientation/1','Baseline log/1');
    if k==1, angleSource='Angles and rates/1'; else, angleSource='Measured angles/1'; end
    add_line(model,angleSource,'Baseline log/2');
    add_line(model,'Baseline log/1','Baseline recording/1');
    experiment=[model '_experiment'];
    load_system(fullfile(project.experiments,[experiment '.slx']));
    ew=get_param(experiment,'ModelWorkspace');
    assert(isequal(evalin(ew,'Reference'),reference),'Experiment default reference differs from the course signal.');
    assert(strcmp(get_param(experiment,'StopTime'),'60'),'Experiment truncates the course signal.');
    assert(strcmp(get_param([experiment '/Desired orientation'],'Interpolate'),'on'), ...
        'Experiment must preserve the original linear interpolation.');
    modes=0; if k>1, modes=[0 1]; end % Q4 has ideal rotor tracking and no sensor model.
    for mode=modes
      for feedforward=0:1
        input=Simulink.SimulationInput(experiment);
        input=input.setVariable('Kff',feedforward,'Workspace',experiment);
        if k>1, input=input.setVariable('meas_on',mode,'Workspace',experiment); end
        out=sim(input);
        names={'position_data','cascade_data','integral_data'};
        signal=out.get(names{k}); r.time=signal.Time; r.data=matrix(signal,widths(k));
        simpleInput=Simulink.SimulationInput(model);
        simpleInput=simpleInput.setVariable('Kff',feedforward,'Workspace',model);
        if k>1, simpleInput=simpleInput.setVariable('meas_on',mode,'Workspace',model); end
        simple=sim(simpleInput); simpleData=matrix(simple.baseline_data,4);
        assert(isequal(simple.baseline_data.Time,r.time) && ...
            max(abs(simpleData-r.data(:,1:4)),[],'all')<1e-10, ...
            'Simple and diagnostic models differ on the course baseline.');
        comparisonRuns{k,mode+1,feedforward+1}=r;
        if feedforward
            if mode==0, runs{k}=r; else, noisyRuns{k}=r; end
        end
        if k==1, actual=r.data(:,3:4); voltage=[]; else
            actual=r.data(:,19:20); voltage=r.data(:,15:16);
        end
        metrics=[metrics;baselineMetrics(models(k),mode,feedforward,r,actual,voltage,reference)]; %#ok<AGROW>
      end
    end
    close_system(model,0);
end
% The fourth case is HIL-free: never simulate the actual QUARC interface.
offline='aero2_hardware_cascade_offline';
load_system(fullfile(project.experiments,[offline '.slx']));
assert(strcmp(get_param(offline,'SystemTargetFile'),'grt.tlc'),'Expected offline simulation target.');
refs=find_system(offline,'FollowLinks','off','LookUnderMasks','all','BlockType','Reference');
for j=1:numel(refs)
    assert(~contains(get_param(refs{j},'SourceBlock'),'quarc_library'),'Offline model contains a hardware reference.');
end
hardwareRuns=cell(2,2);
interface=[offline '/Offline AERO 2 surrogate'];
for axis=1:2
    add_block('simulink/Sinks/To Workspace',[interface '/Baseline true ' num2str(axis)], ...
        'VariableName',['baseline_true_' num2str(axis)],'SaveFormat','Timeseries','MaxDataPoints','inf');
    add_line(interface,sprintf('Course plant/%d',2*axis-1),['Baseline true ' num2str(axis) '/1']);
end
for mode=0:1
  for feedforward=0:1
    input=Simulink.SimulationInput(offline);
    w=get_param(offline,'ModelWorkspace');
    for pair={'U_v','U_h'}
        stored=evalin(w,pair{1});
        if strcmp(pair{1},'U_v'), expectedSignal=U_v; else, expectedSignal=U_h; end
        assert(isequal(stored.Time,expectedSignal.Time) && isequal(stored.Data,expectedSignal.Data), ...
            'Offline surrogate default differs from the course reference.');
    end
    input=input.setVariable('meas_on',mode,'Workspace',offline);
    input=input.setVariable('Kff',feedforward,'Workspace',offline);
    out=sim(input); signal=out.hardware_offline_data;
    r.time=signal.Time; r.data=matrix(signal,14);
    r.actual=[matrix(out.baseline_true_1,1) matrix(out.baseline_true_2,1)];
    hardwareRuns{mode+1,feedforward+1}=r;
    metrics=[metrics;baselineMetrics({offline},mode,feedforward,r,r.actual,r.data(:,13:14),reference)]; %#ok<AGROW>
  end
end
close_system(offline,0);
for mode=0:1
    selected=strcmp(metrics.Model,'aero2_cascade_control') & metrics.MeasurementEffects==mode;
    before=metrics(selected & metrics.ReferenceFeedforward==0,:);
    after=metrics(selected & metrics.ReferenceFeedforward==1,:);
    assert(all(after.TrackingRMS<before.TrackingRMS/2), ...
        'Velocity feedforward must materially reduce course tracking RMS on both axes.');
    assert(all(after.PeakError<before.PeakError), 'Feedforward increased peak tracking error.');
    assert(all(after.SaturationDuration==0),'Course feedforward caused voltage saturation.');
end
if ~isfolder(project.results), mkdir(project.results); end
writetable(metrics,fullfile(project.results,'course_reference_metrics.csv'));
save(fullfile(project.results,'course_reference_validation.mat'), ...
    'reference','referenceRate','runs','noisyRuns','comparisonRuns','hardwareRuns','models','metrics');
disp(metrics);
fprintf('PASS: common course baseline, ideal/noisy tracking and HIL-free filtered surrogate.\n');
end

function rows=baselineMetrics(model,mode,feedforward,r,actual,voltage,reference)
delivered=interp1(reference(:,1),reference(:,2:3),r.time,'linear');
assert(max(abs(r.data(:,1:2)-delivered),[],'all')<1e-12,'Delivered reference differs from the course trajectory.');
assert(abs(r.time(end)-60)<1e-10,'Baseline run must cover all 60 s.');
assert(all(isfinite(r.data(:))),'Course tracking produced nonfinite signals.');
assert(max(abs(actual(:,2)))<=pi/3,'Course trajectory left the pitch envelope.');
peakVoltage=NaN; saturation=NaN;
if ~isempty(voltage)
    peakVoltage=max(abs(voltage),[],'all');
    assert(peakVoltage<=24+1e-9,'Course tracking exceeded voltage limits.');
    saturation=sum(any(abs(voltage)>=24-1e-9,2))*0.002;
end
error=delivered-actual; measuredError=delivered-r.data(:,3:4); rows=table;
for axis=1:2
    row=table(model,mode,feedforward,axis,sqrt(mean(error(:,axis).^2)),max(abs(error(:,axis))), ...
        sqrt(mean(measuredError(:,axis).^2)),peakVoltage,saturation, ...
        'VariableNames',{'Model','MeasurementEffects','ReferenceFeedforward','Axis','TrackingRMS','PeakError', ...
        'MeasuredTrackingRMS','PeakVoltage','SaturationDuration'});
    rows=[rows;row]; %#ok<AGROW>
end
end

function reference=originalReference(file)
% Isolated function workspace: no course setup variables escape into the base workspace.
eval(fileread(file));
reference=[U_v.Time U_v.Data U_h.Data];
end

function data=matrix(signal,width)
if signal.IsTimeFirst
    data=reshape(signal.Data,numel(signal.Time),width);
else
    data=reshape(signal.Data,width,[])';
end
end
