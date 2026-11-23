function export_professor_mechanics_figures
%EXPORT_PROFESSOR_MECHANICS_FIGURES Saved diagrams and recorded Q3/Q4 responses.
project=setup_project;
load(fullfile(project.results,'professor_mechanics','validation.mat'),'defaults','signedRuns');
models={'aero2_linearization_professor_format','aero2_position_professor_format'};
for k=1:2
    load_system(fullfile(project.simulink,[models{k} '.slx']));
    print(['-s' models{k}],'-dpng','-r130', ...
        fullfile(project.docs,'figures',sprintf('professor_format_question%d.png',k+2)));
end
figure('Visible','off','Position',[100 100 1050 530]);tiledlayout(2,2);
for axis=1:2
    nexttile; hold on; r=defaults{1};
    command=reshape(r.command.Data,numel(r.command.Time),[]);
    acceleration=reshape(r.acceleration.Data,numel(r.acceleration.Time),[]);
    plot(r.command.Time,command(:,axis),'k--','DisplayName','Virtual input');
    plot(r.acceleration.Time,acceleration(:,axis),'DisplayName','Actual acceleration');
    labels={'Yaw','Pitch'};ylabel([labels{axis} ' acceleration [rad/s^2]']);
    xlabel('Time [s]');grid on;legend('Location','best');
end
r=signedRuns{2,1};x=reshape(r.state.Data,numel(r.state.Time),[]);selected=r.state.Time>=0.2;
for axis=1:2
    nexttile;hold on;targets=[0.2 0.1];target=targets(axis);
    yline(target,'k--','DisplayName','Reference');
    yline(0.98*target,':','HandleVisibility','off');
    yline(1.02*target,':','HandleVisibility','off');xline(3,':','HandleVisibility','off');
    plot(r.state.Time(selected)-0.2,x(selected,axis),'DisplayName','Simulink');
    labels={'Yaw [rad]','Pitch [rad]'};ylabel(labels{axis});xlabel('Time after step [s]');
    xlim([0 5]);grid on;legend('Location','best');
end
exportgraphics(gcf,fullfile(project.docs,'figures','professor_format_mechanics_validation.png'),'Resolution',180);
close(gcf);
end
