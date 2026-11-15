function export_motor_ip_figures
%EXPORT_MOTOR_IP_FIGURES Plot recorded comparisons and export actual diagrams.
project=setup_project; model='aero2_motor_ip';
load_system(fullfile(project.simulink,[model '.slx']));
s=load(fullfile(project.results,'motor_ip_validation.mat'));
folder=fullfile(project.docs,'figures');
print(['-s' model],'-dpng','-r140',fullfile(folder,'motor_ip_diagram.png'));
f=figure('Visible','off','Color','w','Position',[50 50 1050 700]);
for motor=1:2
    for mode=1:2
        r=s.runs{mode,1};
        subplot(2,2,motor); hold on;
        plot(r.time,r.data(:,motor+2),'LineWidth',1.6);
        ylabel('Speed [rad/s]');
        subplot(2,2,motor+2); hold on;
        plot(r.time,r.data(:,motor+8),'LineWidth',1.6);
        ylabel('Applied voltage [V]');
    end
    subplot(2,2,motor); plot(r.time,r.data(:,motor),':k','LineWidth',1.2);
    names={'Main +100 rad/s','Tail -100 rad/s'}; title(names{motor});
    legend('PI','IP','Reference','Location','best');
    subplot(2,2,motor+2); legend('PI','IP','Location','best');
end
for k=1:4, subplot(2,2,k); grid on; xlim([0 0.8]); xlabel('Time [s]'); end
print(f,fullfile(folder,'motor_ip_nominal.png'),'-dpng','-r150'); close(f);
f=figure('Visible','off','Color','w','Position',[50 50 1050 1000]);
columns=[3 4;7 8;9 10;11 12];
units={'Speed [rad/s]','Raw voltage [V]','Applied voltage [V]','Integral contribution [V]'};
for motor=1:2
    for panel=1:4
        subplot(4,2,2*(panel-1)+motor); hold on;
        for enabled=1:2
            r=s.recoveryRuns{enabled,1};
            plot(r.time,r.data(:,columns(panel,motor)),'LineWidth',1.5);
        end
        if panel==1
            plot(r.time,r.data(:,motor),':k','LineWidth',1.1);
            title(names{motor}); legend('IP without AW','IP with AW','Reference','Location','best');
        end
        xline(s.p.recoveryTime,':k','HandleVisibility','off');
        grid on; xlim([0 5]); ylabel(units{panel}); xlabel('Time [s]');
    end
end
print(f,fullfile(folder,'motor_ip_recovery.png'),'-dpng','-r150'); close(f);
end
