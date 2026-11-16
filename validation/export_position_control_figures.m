function export_position_control_figures
%EXPORT_POSITION_CONTROL_FIGURES Export the simple diagram and experiment traces.
project=setup_project;
load_system(fullfile(project.simulink,'aero2_position_control.slx'));
s=load(fullfile(project.results,'position_control_validation.mat'));
folder=fullfile(project.docs,'figures');
print('-saero2_position_control','-dpng','-r150',fullfile(folder,'question_4_diagram.png'));
f=figure('Visible','off','Color','w','Position',[50 50 1100 850]);
cases=[1 3 5]; names={'Yaw-only command','Pitch-only command','Simultaneous command'};
axesNames={'Yaw','Pitch'};
for test=1:3
    r=s.runs{cases(test)};
    for axis=1:2
        subplot(3,2,2*(test-1)+axis);
        plot(r.time,r.data(:,axis),':k','LineWidth',1.6); hold on;
        plot(r.time,r.data(:,axis+2),'LineWidth',1.5);
        target=r.data(end,axis);
        if target~=0
            yline(target+0.02*abs(target),':','HandleVisibility','off');
            yline(target-0.02*abs(target),':','HandleVisibility','off');
        else
            ylim([-0.01 0.01]);
        end
        xline(s.p.stepTime+s.p.targetSettlingTime,':k','HandleVisibility','off');
        grid on; xlim([0 5]); xlabel('Time [s]'); ylabel('Angle [rad]');
        title([names{test} ': ' axesNames{axis}]);
        legend('Desired','Measured (ideal motor tracking)','Location','best');
    end
end
print(f,fullfile(folder,'position_control_steps.png'),'-dpng','-r150'); close(f);
r=s.runs{5};
f=figure('Visible','off','Color','w','Position',[50 50 1050 650]);
subplot(2,2,1); plot(r.time,r.data(:,11:12),'LineWidth',1.5);
ylabel('Position error [rad]'); legend('Yaw','Pitch');
subplot(2,2,2); plot(r.time,r.data(:,5:6),'LineWidth',1.5);
ylabel('Angular velocity [rad/s]'); legend('Yaw rate','Pitch rate');
subplot(2,2,3); plot(r.time,r.data(:,7:8),'LineWidth',1.5);
ylabel('Desired acceleration [rad/s^2]'); legend('Yaw','Pitch');
subplot(2,2,4); plot(r.time,r.data(:,9:10),'LineWidth',1.5);
ylabel('Desired rotor speed [rad/s]'); legend('Main','Tail');
for k=1:4, subplot(2,2,k); grid on; xlim([0 8]); xlabel('Time [s]'); end
print(f,fullfile(folder,'position_control_signals.png'),'-dpng','-r150'); close(f);
end
