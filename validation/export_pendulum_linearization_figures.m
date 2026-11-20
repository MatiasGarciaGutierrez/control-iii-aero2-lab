function export_pendulum_linearization_figures
%EXPORT_PENDULUM_LINEARIZATION_FIGURES Diagram exports and logged-signal plots.
project = setup_project; model = 'part1_pendulum_linearization';
load_system(fullfile(project.simulink, [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
folder = fullfile(project.docs, 'figures');
print(['-s' model], '-dpng', '-r150', fullfile(folder, 'pendulum_linearization_diagram.png'));
print(['-s' model '/Linearizing controller'], '-dpng', '-r150', ...
    fullfile(folder, 'pendulum_linearization_controller.png'));
recording = load(fullfile(project.results, 'pendulum_linearization_validation.mat'), 'runs');
run = recording.runs{1};
handle = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 700]);
subplot(3,1,1);
plot(run.time, run.alpha, 'LineWidth', 1.8); hold on;
plot(run.time, run.v, '--', 'LineWidth', 1.3); grid on;
ylabel('Acceleration [rad/s^2]'); legend('Actual acceleration','Virtual input v');
title('Question 1(b): nonlinear cancellation, without stabilization');
subplot(3,1,2);
plot(run.time, run.theta, 'LineWidth', 1.8); grid on; ylabel('Angle [rad]');
subplot(3,1,3);
plot(run.time, run.u, 'LineWidth', 1.8); grid on;
ylabel('Plant input u [N m]'); xlabel('Simulation time [s]');
print(handle, fullfile(folder, 'pendulum_linearization_response.png'), '-dpng', '-r150');
close(handle);
end
