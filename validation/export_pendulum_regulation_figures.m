function export_pendulum_regulation_figures
%EXPORT_PENDULUM_REGULATION_FIGURES Export diagrams and logged response.
project = setup_project; model = 'part1_pendulum_regulation';
load_system(fullfile(project.simulink, [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
folder = fullfile(project.docs, 'figures');
print(['-s' model], '-dpng', '-r150', fullfile(folder, 'pendulum_regulation_diagram.png'));
print(['-s' model '/Linear state feedback'], '-dpng', '-r150', ...
    fullfile(folder, 'pendulum_regulation_feedback.png'));
recording = load(fullfile(project.results, 'pendulum_regulation_validation.mat'), ...
    'runs', 'targetSettling');
run = recording.runs{1};
handle = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 700]);
subplot(3,1,1);
plot(run.time, run.theta, 'LineWidth', 1.8); hold on;
yline(0.002, '--', '2% of initial angle');
yline(-0.002, '--'); xline(recording.targetSettling, ':', '0.9723 s');
grid on; ylabel('Angle [rad]');
title('Upright regulation: v = -36 theta - 12 omega');
subplot(3,1,2);
plot(run.time, run.omega, 'LineWidth', 1.8); grid on; ylabel('Rate [rad/s]');
subplot(3,1,3);
plot(run.time, run.u, 'LineWidth', 1.8); grid on;
ylabel('Plant input u [N m]'); xlabel('Simulation time [s]');
print(handle, fullfile(folder, 'pendulum_regulation_response.png'), '-dpng', '-r150');
close(handle);
end
