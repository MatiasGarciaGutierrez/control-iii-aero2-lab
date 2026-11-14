function project = setup_project
%SETUP_PROJECT Return absolute paths and add existing implementation folders.
% Does not initialize the course model, change folder, or run simulations.
project.root = fileparts(mfilename('fullpath'));
project.course = fullfile(project.root, 'course');
project.matlab = fullfile(project.root, 'matlab');
project.simulink = fullfile(project.root, 'simulink');
project.validation = fullfile(project.root, 'validation');
project.docs = fullfile(project.root, 'docs');
project.results = fullfile(project.root, 'results');

folders = {project.root, project.matlab, ...
    fullfile(project.matlab, 'parameters'), ...
    fullfile(project.matlab, 'controllers'), ...
    fullfile(project.matlab, 'mechanics'), ...
    project.simulink, project.validation};
for k = 1:numel(folders)
    if isfolder(folders{k})
        addpath(folders{k});
    end
end
end
