function export_dc_motor_figures
%EXPORT_DC_MOTOR_FIGURES Export block diagrams and logged-signal evidence.
project = setup_project; folder = fullfile(project.docs,'figures');
models = {'part1_dc_motor','part1_dc_motor_linearization'};
for index = 1:2
    load_system(fullfile(project.simulink,[models{index} '.slx']));
    print(['-s' models{index}],'-dpng','-r150', ...
        fullfile(folder,[models{index} '_diagram.png']));
end
print('-spart1_dc_motor/Nonlinear DC motor','-dpng','-r150', ...
    fullfile(folder,'dc_motor_plant.png'));
print('-spart1_dc_motor_linearization/Linearizing controller','-dpng','-r150', ...
    fullfile(folder,'dc_motor_linearizing_controller.png'));
close_system(models{1},0);close_system(models{2},0);
recording = load(fullfile(project.results,'dc_motor_validation.mat'),'runs');
run = recording.runs{1}; crossing = recording.runs{4};
handle = figure('Visible','off','Color','w','Position',[100 100 1000 800]);
subplot(4,1,1);
plot(run.time,run.jerk,'LineWidth',1.8);hold on;
plot(run.time,run.v,'--','LineWidth',1.3);grid on;
ylabel('Jerk [rad/s^3]');legend('Actual jerk','Virtual input v');
title('DC motor input-output linearization: theta third derivative = v');
subplot(4,1,2);plot(run.time,run.theta,'LineWidth',1.8);grid on;ylabel('Angle [rad]');
subplot(4,1,3);plot(run.time,run.u,'LineWidth',1.8);grid on;ylabel('Voltage [V]');
subplot(4,1,4);plot(crossing.time,crossing.omega,'LineWidth',1.8);hold on;
yline(0,'--');grid on;ylabel('Speed [rad/s]');xlabel('Simulation time [s]');
title('Separate validation case: continuous passage through zero speed');
print(handle,fullfile(folder,'dc_motor_linearization_response.png'),'-dpng','-r150');
close(handle);
end
