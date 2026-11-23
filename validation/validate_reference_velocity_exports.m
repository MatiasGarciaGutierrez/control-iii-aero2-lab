function validate_reference_velocity_exports
%VALIDATE_REFERENCE_VELOCITY_EXPORTS Run R2021a exports in the installed release.
% This checks exported dynamics; it does not establish execution in R2021a.
project=setup_project;
s=load(fullfile(project.results,'course_reference_validation.mat'));
signals={'position_data','cascade_data','integral_data'}; widths=[16 24 28];
for k=1:3
    model=[s.models{k} '_experiment_R2021a'];
    load_system(fullfile(project.experiments,[model '.slx']));
    input=Simulink.SimulationInput(model);
    input=input.setVariable('Kff',1,'Workspace',model);
    if k>1, input=input.setVariable('meas_on',0,'Workspace',model); end
    output=sim(input); signal=output.get(signals{k});
    data=matrix(signal,widths(k)); reference=s.runs{k};
    assert(isequal(signal.Time,reference.time) && ...
        max(abs(data-reference.data),[],'all')<1e-10, ...
        'Exported course feedforward dynamics differ from the working model.');
end
model='aero2_hardware_cascade_offline_R2021a';
load_system(fullfile(project.experiments,[model '.slx']));
input=Simulink.SimulationInput(model);
input=input.setVariable('Kff',1,'Workspace',model);
input=input.setVariable('meas_on',0,'Workspace',model);
output=sim(input); signal=output.hardware_offline_data;
reference=s.hardwareRuns{1,2};
assert(isequal(signal.Time,reference.time) && ...
    max(abs(matrix(signal,14)-reference.data),[],'all')<1e-10, ...
    'Exported filtered surrogate differs on the course trajectory.');
fprintf('PASS: reference-velocity exports agree in the installed MATLAB release.\n');
end

function data=matrix(signal,width)
if signal.IsTimeFirst
    data=reshape(signal.Data,numel(signal.Time),width);
else
    data=reshape(signal.Data,width,[])';
end
end
