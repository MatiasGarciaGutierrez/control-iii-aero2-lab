function metrics = validate_course_reference
%VALIDATE_COURSE_REFERENCE Exact course inputs and default 60 s tracking runs.
project=setup_project; [U_v,U_h,reference]=aero2_course_reference;
expected=originalReference(fullfile(project.course,'model_AERO2_simul_ini.m'));
assert(isequal(reference,expected),'Reference generator differs from the original course initializer.');
assert(isequal(U_v.Time,U_h.Time) && isequal(reference(:,2:3),[U_v.Data U_h.Data]), ...
    'Course timeseries order or scaling is incorrect.');
models={'aero2_position_control','aero2_cascade_control','aero2_position_integral'};
widths=[16 24 28]; runs=cell(3,1); metrics=table;
for k=1:3
    model=models{k}; load_system(fullfile(project.simulink,[model '.slx']));
    w=get_param(model,'ModelWorkspace');
    storedYaw=evalin(w,'U_v'); storedPitch=evalin(w,'U_h');
    assert(isequal(storedYaw.Data,U_v.Data) && isequal(storedPitch.Data,U_h.Data), ...
        'A saved model does not contain the course reference.');
    assert(strcmp(get_param(model,'StopTime'),'60'),'A position model stops before the full trajectory.');
    assert(strcmp(get_param([model '/Desired orientation/Course yaw reference'],'VariableName'),'U_v'));
    assert(strcmp(get_param([model '/Desired orientation/Course pitch reference'],'VariableName'),'U_h'));
    scopes=find_system(model,'LookUnderMasks','all','BlockType','Scope');
    for j=1:numel(scopes)
        config=get_param(scopes{j},'ScopeConfiguration');
        assert(strcmp(config.TimeSpan,'60'),'A Scope hides part of the course trajectory.');
    end
    experiment=[model '_experiment'];
    load_system(fullfile(project.experiments,[experiment '.slx'])); out=sim(experiment);
    if k==1, signal=out.position_data; else
        if k==2, signal=out.cascade_data; else, signal=out.integral_data; end
    end
    r.time=signal.Time; r.data=matrix(signal,widths(k)); runs{k}=r;
    delivered=interp1(reference(:,1),reference(:,2:3),r.time,'linear');
    assert(max(abs(r.data(:,1:2)-delivered),[],'all')<1e-12,'Delivered angle reference differs from the course trajectory.');
    assert(abs(r.time(end)-60)<1e-10,'Default run did not cover all 60 s.');
    measured=r.data(:,3:4); error=delivered-measured;
    assert(all(isfinite(r.data(:))),'Default course tracking produced nonfinite signals.');
    assert(max(abs(measured(:,2)))<=pi/3,'Course trajectory left the pitch envelope.');
    for axis=1:2
        row=table(models(k),axis,sqrt(mean(error(:,axis).^2)),max(abs(error(:,axis))), ...
            'VariableNames',{'Model','Axis','TrackingRMS','PeakError'});
        metrics=[metrics;row]; %#ok<AGROW>
    end
    if k>1
        assert(max(abs(r.data(:,15:16)),[],'all')<=24,'Default tracking exceeded the voltage limits.');
    end
end
if ~isfolder(project.results), mkdir(project.results); end
writetable(metrics,fullfile(project.results,'course_reference_metrics.csv'));
save(fullfile(project.results,'course_reference_validation.mat'),'reference','runs','models','metrics');
disp(metrics);
fprintf('PASS: original reference reproduced exactly; all position models default to the full course trajectory.\n');
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
