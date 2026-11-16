function export_cascade_control_figures
%EXPORT_CASCADE_CONTROL_FIGURES Diagrams and logged evidence for Question 5.
project=setup_project; s=load(fullfile(project.results,'cascade_control_validation.mat'));
model='aero2_cascade_control'; load_system(fullfile(project.simulink,[model '.slx']));
folder=fullfile(project.docs,'figures');
print(['-s' model],'-dpng','-r150',fullfile(folder,'question_5_diagram.png'));
print(['-s' model '/IP motor control'],'-dpng','-r150',fullfile(folder,'cascade_ip_diagram.png'));
axesNames={'Yaw','Pitch'}; cases=[1 3 5];
f=figure('Visible','off','Color','w','Position',[50 50 1150 850]);
for j=1:3
    r=s.runs{cases(j)}; ideal=s.idealRuns{cases(j)};
    for axis=1:2
        subplot(3,2,2*(j-1)+axis);
        plot(r.time,r.data(:,axis),':k','LineWidth',1.6); hold on;
        plot(r.time,ideal(:,axis+2),'--','LineWidth',1.3);
        plot(r.time,r.data(:,axis+18),'Color',[0 0.447 0.741],'LineWidth',1.4);
        target=r.data(end,axis);
        if target~=0
            yline(target+0.02*abs(target),':','HandleVisibility','off');
            yline(target-0.02*abs(target),':','HandleVisibility','off');
        else
            ylim([-0.005 0.005]);
        end
        xline(s.p.stepTime+3,':k','HandleVisibility','off');
        title([s.names{cases(j)} ': ' axesNames{axis}]);
        xlabel('Time [s]'); ylabel('Angle [rad]'); grid on; xlim([0 5]);
        legend('Desired','Ideal motors (Q4)','IP motors (Q5)','Location','best');
    end
end
print(f,fullfile(folder,'cascade_position_steps.png'),'-dpng','-r150'); close(f);
r=s.runs{5};
f=figure('Visible','off','Color','w','Position',[50 50 1150 850]);
for axis=1:2
    subplot(3,2,axis);
    plot(r.time,r.data(:,8+axis),':k','LineWidth',1.5); hold on;
    plot(r.time,r.data(:,22+axis),'LineWidth',1.3);
    ylabel('Rotor speed [rad/s]'); names={'Main motor','Tail motor'}; title(names{axis});
    legend('Desired','Actual','Location','best');
    subplot(3,2,axis+2);
    plot(r.time,r.data(:,12+axis),'--','LineWidth',1.3); hold on;
    plot(r.time,r.data(:,14+axis),'LineWidth',1.3);
    yline(24,':k','HandleVisibility','off'); yline(-24,':k','HandleVisibility','off');
    ylabel('Voltage [V]'); legend('Raw','Applied','Location','best');
    subplot(3,2,axis+4);
    plot(r.time,r.data(:,16+axis),'LineWidth',1.3);
    ylabel('Integral contribution [V]');
end
for j=1:6, subplot(3,2,j); xlabel('Time [s]'); grid on; xlim([0 8]); end
print(f,fullfile(folder,'cascade_motor_signals.png'),'-dpng','-r150'); close(f);
r=s.runs{5};
f=figure('Visible','off','Color','w','Position',[50 50 1100 650]);
subplot(2,2,1); plot(r.time,r.data(:,1:2)-r.data(:,19:20),'LineWidth',1.3);
ylabel('Position error [rad]'); legend('Yaw','Pitch');
subplot(2,2,2); plot(r.time,r.data(:,21:22),'LineWidth',1.3);
ylabel('Body angular velocity [rad/s]'); legend('Yaw rate','Pitch rate');
subplot(2,2,3); plot(r.time,r.data(:,7:8),'LineWidth',1.3);
ylabel('Desired acceleration [rad/s^2]'); legend('Yaw','Pitch');
subplot(2,2,4); plot(r.time,r.data(:,9:10)-r.data(:,23:24),'LineWidth',1.3);
ylabel('Rotor speed error [rad/s]'); legend('Main','Tail');
for j=1:4, subplot(2,2,j); grid on; xlabel('Time [s]'); xlim([0 8]); end
print(f,fullfile(folder,'cascade_motion_signals.png'),'-dpng','-r150'); close(f);
r=s.noisyRuns{5};
f=figure('Visible','off','Color','w','Position',[50 50 1100 650]);
for axis=1:2
    subplot(2,2,axis);
    plot(r.time,r.data(:,axis),':k','LineWidth',1.4); hold on;
    plot(r.time,r.data(:,axis+2),'Color',[0.7 0.7 0.7]);
    plot(r.time,r.data(:,axis+18),'Color',[0 0.447 0.741],'LineWidth',1.2);
    title([axesNames{axis} ': noisy measurement test']); ylabel('Angle [rad]');
    legend('Desired','Sensor measurement','True simulated angle','Location','best');
    subplot(2,2,axis+2);
    plot(r.time,r.data(:,axis)-r.data(:,axis+2),'Color',[0.7 0.7 0.7]); hold on;
    plot(r.time,r.data(:,axis)-r.data(:,axis+18),'LineWidth',1.2);
    xlim([6 8]); ylabel('Final-window error [rad]');
    legend('Measured error','True error','Location','best');
end
for j=1:4, subplot(2,2,j); grid on; xlabel('Time [s]'); end
print(f,fullfile(folder,'cascade_sensor_effects.png'),'-dpng','-r150'); close(f);
f=figure('Visible','off','Color','w','Position',[50 50 1100 650]);
for axis=1:2
    subplot(2,2,axis);
    plot(s.smooth.time,s.smooth.data(:,axis),':k','LineWidth',1.5); hold on;
    plot(s.smooth.time,s.smooth.data(:,axis+18),'LineWidth',1.3);
    plot(s.smoothNoisy.time,s.smoothNoisy.data(:,axis+18),'--','LineWidth',1.1);
    title([axesNames{axis} ': supplied smooth reference']); ylabel('Angle [rad]');
    legend('Desired','Ideal sensors','Noisy sensors','Location','best');
    subplot(2,2,axis+2);
    plot(s.smooth.time,s.smooth.data(:,axis)-s.smooth.data(:,axis+18),'LineWidth',1.3); hold on;
    plot(s.smoothNoisy.time,s.smoothNoisy.data(:,axis)-s.smoothNoisy.data(:,axis+18),'--','LineWidth',1.1);
    ylabel('Tracking error [rad]'); legend('Ideal sensors','Noisy sensors','Location','best');
end
for j=1:4, subplot(2,2,j); grid on; xlabel('Time [s]'); xlim([0 60]); end
print(f,fullfile(folder,'cascade_smooth_tracking.png'),'-dpng','-r150'); close(f);
end
