function export_position_integral_figures
%EXPORT_POSITION_INTEGRAL_FIGURES Optional controller and comparison evidence.
project=setup_project; s=load(fullfile(project.results,'position_integral_validation.mat'));
model='aero2_position_integral'; load_system(fullfile(project.simulink,[model '.slx']));
folder=fullfile(project.docs,'figures');
print(['-s' model],'-dpng','-r150',fullfile(folder,'position_integral_diagram.png'));
print(['-s' model '/Position integral control'],'-dpng','-r150',fullfile(folder,'position_integral_controller.png'));
r=s.runs{5};
f=figure('Visible','off','Color','w','Position',[50 50 1100 650]);
for axis=1:2
    names={'Yaw','Pitch'};
    subplot(2,2,axis);
    plot(r.time,r.data(:,axis),':k','LineWidth',1.5); hold on;
    plot(s.pd.time,s.pd.data(:,18+axis),'--','LineWidth',1.3);
    plot(r.time,r.data(:,18+axis),'Color',[0 0.447 0.741],'LineWidth',1.4);
    title([names{axis} ': nominal step']); ylabel('Angle [rad]');
    legend('Desired','Question 5 PD','Optional integral','Location','best');
    subplot(2,2,axis+2);
    plot(s.pd.time,s.pd.data(:,20+axis),'--','LineWidth',1.3); hold on;
    plot(r.time,r.data(:,20+axis),'LineWidth',1.4);
    ylabel('Body angular velocity [rad/s]'); legend('Question 5 PD','Optional integral');
end
for k=1:4, subplot(2,2,k); grid on; xlabel('Time [s]'); xlim([0 8]); end
print(f,fullfile(folder,'position_integral_nominal.png'),'-dpng','-r150'); close(f);
a=s.disturbancePD; b=s.disturbanceIntegral;
f=figure('Visible','off','Color','w','Position',[50 50 1100 850]);
for axis=1:2
    subplot(3,2,axis);
    plot(a.time,a.data(:,axis)-a.data(:,18+axis),'--','LineWidth',1.3); hold on;
    plot(b.time,b.data(:,axis)-b.data(:,18+axis),'LineWidth',1.4);
    title([names{axis} ': constant external torque at 5 s']); ylabel('Position error [rad]');
    legend('PD only','With integral','Location','best');
    subplot(3,2,axis+2);
    plot(a.time,a.data(:,20+axis),'--','LineWidth',1.3); hold on;
    plot(b.time,b.data(:,20+axis),'LineWidth',1.4);
    ylabel('Body angular velocity [rad/s]'); legend('PD only','With integral');
    subplot(3,2,axis+4);
    plot(b.time,b.data(:,24+axis),'LineWidth',1.4);
    ylabel('Outer integral contribution [rad/s^2]');
end
for k=1:6, subplot(3,2,k); grid on; xlabel('Time [s]'); xlim([4 20]); xline(5,':k','HandleVisibility','off'); end
print(f,fullfile(folder,'position_integral_disturbance.png'),'-dpng','-r150'); close(f);
a=s.recoveryWithout; b=s.recoveryWith;
f=figure('Visible','off','Color','w','Position',[50 50 1100 850]);
subplot(3,2,1);
plot(b.time,b.data(:,1),':k','LineWidth',1.5); hold on;
plot(a.time,a.data(:,19),'--','LineWidth',1.3); plot(b.time,b.data(:,19),'LineWidth',1.4);
ylabel('Yaw [rad]'); title('Temporary voltage-authority loss');
legend('Desired','Outer AW off','Outer AW on','Location','best');
subplot(3,2,2); plot(a.time,a.data(:,21),'--','LineWidth',1.3); hold on;
plot(b.time,b.data(:,21),'LineWidth',1.4); ylabel('Yaw rate [rad/s]');
legend('Outer AW off','Outer AW on');
subplot(3,2,3); plot(a.time,a.data(:,25),'--','LineWidth',1.3); hold on;
plot(b.time,b.data(:,25),'LineWidth',1.4); ylabel('Yaw integral contribution [rad/s^2]');
legend('Outer AW off','Outer AW on');
subplot(3,2,4); plot(b.time,b.data(:,15:16),'LineWidth',1.2);
ylabel('Applied voltage [V]'); legend('Main','Tail');
subplot(3,2,5); plot(a.time,a.data(:,1)-a.data(:,19),'--','LineWidth',1.3); hold on;
plot(b.time,b.data(:,1)-b.data(:,19),'LineWidth',1.4); ylabel('Yaw error [rad]');
legend('Outer AW off','Outer AW on');
subplot(3,2,6); plot(b.time,2+22*(b.time>=4),':k','LineWidth',1.5);
ylabel('Voltage authority [V]'); ylim([0 26]);
for k=1:6, subplot(3,2,k); grid on; xlabel('Time [s]'); xlim([0 14]); xline(4,':k','HandleVisibility','off'); end
print(f,fullfile(folder,'position_integral_recovery.png'),'-dpng','-r150'); close(f);
end
