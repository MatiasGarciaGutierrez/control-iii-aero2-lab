function export_professor_cascade_figures
%EXPORT_PROFESSOR_CASCADE_FIGURES Plot recorded runs and export saved blocks.
project=setup_project;
load(fullfile(project.results,'professor_format','validation.mat'),'runs','stress');
[~,~,reference]=aero2_course_reference;
model='aero2_cascade_professor_format';
load_system(fullfile(project.simulink,[model '.slx']));
figure('Visible','off'); tiledlayout(1,3);
for axis=1:2
    nexttile; hold on; plot(reference(:,1),reference(:,axis+1),'k:','DisplayName','Course reference');
    for mode=0:1
        data=squeeze(runs{mode+1}.true_state.Data);
        names={'Ideal measurements','Course sensors'};
        plot(runs{mode+1}.true_state.Time,data(:,axis),'DisplayName',names{mode+1});
    end
    labels={'Yaw [rad]','Pitch [rad]'}; ylabel(labels{axis}); xlabel('Time [s]'); grid on;
    legend('Location','best','FontSize',8);
end
nexttile; plot(stress.time,stress.voltage); yline(24,'k:'); yline(-24,'k:');
ylabel('Motor voltage [V]'); xlabel('Time [s]'); grid on;
legend('Voltage','Limits','Location','best');
set(gcf,'Position',[100 100 1200 280]);
exportgraphics(gcf,fullfile(project.docs,'figures','professor_format_validation.png'),'Resolution',180);
close(gcf);
print(['-s' model],'-dpng','-r130', ...
    fullfile(project.docs,'figures','professor_format_cascade.png'));
print(['-s' model '/IP_m'],'-dpng','-r130', ...
    fullfile(project.docs,'figures','professor_format_ip.png'));
end
