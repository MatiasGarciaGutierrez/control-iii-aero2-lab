function export_pendulum_figures
%EXPORT_PENDULUM_FIGURES Export diagram and plots, not UI screenshots.
project = setup_project;
model = 'part1_pendulum';
load_system(fullfile(project.simulink, [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
folder = fullfile(project.docs, 'figures');
print(['-s' model], '-dpng', '-r160', fullfile(folder, 'pendulum_diagram.png'));
recording = load(fullfile(project.results, 'pendulum_validation.mat'), 'runs');
run = recording.runs{2};
handle = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 650]);
subplot(2,1,1);
plot(run.time, run.state(:,1), 'LineWidth', 1.7);
yline(pi, '--', 'Downward vertical'); grid on;
ylabel('Angle [rad]');
title('Open-loop nonlinear pendulum: u = 0, theta(0) = 0.1 rad');
subplot(2,1,2);
plot(run.time, run.state(:,2), 'LineWidth', 1.7); grid on;
xlabel('Simulation time [s]'); ylabel('Angular velocity [rad/s]');
print(handle, fullfile(folder, 'pendulum_free_response.png'), '-dpng', '-r160');
close(handle);
end
