function export_simulator_figures
%EXPORT_SIMULATOR_FIGURES Export the diagram and recorded scope signals.
% These are diagram/trace exports, not screenshots of native parameter dialogs.
project = setup_project;
model = 'aero2_simulator';
load_system(fullfile(project.simulink, [model '.slx']));
recording = load(fullfile(project.results, 'simulator_validation.mat'), 'pulse');
folder = fullfile(project.docs, 'figures');
if ~isfolder(folder)
    mkdir(folder);
end
print(['-s' model], '-dpng', '-r160', fullfile(folder, 'simulator_diagram.png'));
names = {'voltage_scope', 'rotor_speed_scope', 'body_angle_scope'};
titles = {'Voltage scope signals', 'Rotor speed scope signals', 'Body angle scope signals'};
units = {'Voltage [V]', 'Speed [rad/s]', 'Angle [rad]'};
legends = {{'Main motor', 'Tail motor'}, {'Main rotor', 'Tail rotor'}, {'Yaw', 'Pitch'}};
for k = 1:3
    figureHandle = figure('Visible', 'off', 'Color', 'w', ...
        'Position', [100 100 1000 360]);
    columns = 2*k-1:2*k;
    plot(recording.pulse.time, recording.pulse.data(:,columns), 'LineWidth', 1.7);
    grid on;
    xlim([0 1]);
    if k == 1
        ylim([-1.2 1.2]);
    end
    xlabel('Simulation time [s]'); ylabel(units{k}); title(titles{k});
    legend(legends{k}, 'Location', 'best');
    print(figureHandle, fullfile(folder, [names{k} '.png']), '-dpng', '-r160');
    close(figureHandle);
end
fprintf('Exported Simulink diagram and three measured-signal figures to docs/figures.\n');
end
