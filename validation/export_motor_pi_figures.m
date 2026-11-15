function export_motor_pi_figures
%EXPORT_MOTOR_PI_FIGURES Export actual diagram and logged speed-loop results.
project = setup_project;
model = 'aero2_motor_pi';
load_system(fullfile(project.simulink, [model '.slx']));
s = load(fullfile(project.results, 'motor_pi_validation.mat'));
folder = fullfile(project.docs, 'figures');
print(['-s' model], '-dpng', '-r140', fullfile(folder, 'motor_pi_diagram.png'));
f = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 1100 850]);
for motor = 1:2
    for signIndex = 1:2
        runIndex = 2*(motor-1)+signIndex;
        r = s.runs{runIndex};
        style = '-'; if signIndex == 2, style = '--'; end
        subplot(3,2,motor); hold on;
        plot(r.time, r.data(:,motor), [style 'k'], 'LineWidth', 1);
        plot(r.time, r.data(:,motor+2), style, 'LineWidth', 1.6);
        ylabel('Speed [rad/s]');
        subplot(3,2,motor+2); hold on;
        plot(r.time, r.data(:,motor+4), style, 'LineWidth', 1.6);
        ylabel('Error [rad/s]');
        subplot(3,2,motor+4); hold on;
        plot(r.time, r.data(:,motor+8), style, 'LineWidth', 1.6);
        ylabel('Applied voltage [V]');
    end
end
names = {'Main motor', 'Tail motor'};
for k = 1:6
    subplot(3,2,k); grid on; xlim([0 0.7]); xlabel('Simulation time [s]');
    if k<=2
        title(names{k}); legend('+100 reference','+100 measured', ...
            '-100 reference','-100 measured', 'Location','southeast');
    else
        legend('+100 test','-100 test','Location','best');
    end
    if k>=5, ylim([-25 25]); yline(24, ':k', 'HandleVisibility', 'off'); yline(-24, ':k', 'HandleVisibility', 'off'); end
end
print(f, fullfile(folder, 'motor_pi_steps.png'), '-dpng', '-r150');
close(f);
end
