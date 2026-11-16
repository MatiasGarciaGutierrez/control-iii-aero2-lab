function export_hardware_cascade_figures
%EXPORT_HARDWARE_CASCADE_FIGURES Diagrams and OFFLINE evidence only.
project=setup_project; folder=fullfile(project.docs,'figures');
s=load(fullfile(project.results,'hardware_cascade','validation.mat'));
model='aero2_hardware_cascade'; load_system(fullfile(project.simulink,[model '.slx']));
print(['-s' model],'-dpng','-r150',fullfile(folder,'hardware_cascade_diagram.png'));
print(['-s' model '/IP motor control'],'-dpng','-r150',fullfile(folder,'hardware_ip_diagram.png'));
% Include the existing Question 5 reference result when it is available.
baselinePath=fullfile(project.results,'cascade_control_validation.mat');
baseline=[];
if isfile(baselinePath)
    b=load(baselinePath,'runs'); baseline=b.runs{5};
end
r=s.runs{6}; names={'Yaw','Pitch'};
f=figure('Visible','off','Color','w','Position',[50 50 1100 850]);
for axis=1:2
    subplot(3,2,axis);
    plot(r.time,r.data(:,axis),':k','LineWidth',1.4); hold on;
    plot(r.time,r.data(:,axis+2),'LineWidth',1.3);
    if ~isempty(baseline)
        plot(baseline.time,baseline.data(:,axis+18),'--','LineWidth',1.2);
        legend('Desired','With hardware filters','Q5 direct rates','Location','best');
    else
        legend('Desired','With hardware filters','Location','best');
    end
    target=r.data(end,axis);
    yline(target+.02*abs(target),':','HandleVisibility','off');
    yline(target-.02*abs(target),':','HandleVisibility','off');
    xline(3.2,':k','HandleVisibility','off');
    title([names{axis} ' position (offline)']); ylabel('Angle [rad]');
    subplot(3,2,axis+2);
    plot(r.time,r.data(:,axis+4),'LineWidth',1.2);
    ylabel('Estimated body rate [rad/s]'); title([names{axis} ' rate']);
    subplot(3,2,axis+4);
    plot(r.time,r.data(:,axis+12),'LineWidth',1.2);
    ylabel('Applied voltage [V]'); motors={'Main','Tail'}; title([motors{axis} ' motor']);
end
for k=1:6, subplot(3,2,k); grid on; xlabel('Time [s]'); xlim([0 8]); end
print(f,fullfile(folder,'hardware_cascade_offline_steps.png'),'-dpng','-r150'); close(f);
r=s.course;
f=figure('Visible','off','Color','w','Position',[50 50 1100 600]);
for axis=1:2
    subplot(2,1,axis);
    plot(r.time,r.data(:,axis),':k','LineWidth',1.4); hold on;
    plot(r.time,r.data(:,axis+2),'LineWidth',1.2);
    ylabel('Angle [rad]'); title([names{axis} ' course trajectory (offline)']);
    legend('Course reference','Offline response','Location','best');
    grid on; xlabel('Time [s]'); xlim([0 60]);
end
print(f,fullfile(folder,'hardware_cascade_offline_course.png'),'-dpng','-r150'); close(f);
end
