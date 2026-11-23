function export_course_reference_figures
%EXPORT_COURSE_REFERENCE_FIGURES Default course inputs and position-model traces.
project=setup_project; s=load(fullfile(project.results,'course_reference_validation.mat'));
folder=fullfile(project.docs,'figures');
load_system(fullfile(project.simulink,'aero2_cascade_control.slx'));
print('-saero2_cascade_control/Desired orientation','-dpng','-r150', ...
    fullfile(folder,'course_reference_source.png'));
f=figure('Visible','off','Color','w','Position',[50 50 1100 850]);
labels={'Q4: ideal motors','Q5: full cascade','Optional: position integral'};
axesNames={'Yaw','Pitch'};
for k=1:3
    r=s.runs{k};
    for axis=1:2
        subplot(3,2,2*(k-1)+axis);
        plot(r.time,r.data(:,axis),':k','LineWidth',1.5); hold on;
        plot(r.time,r.data(:,axis+2),'Color',[0 0.447 0.741],'LineWidth',1.3);
        title([labels{k} ' - ' axesNames{axis}]); ylabel('Angle [rad]');
        grid on; xlabel('Time [s]'); xlim([0 60]); legend('Course reference','Simulated angle','Location','best');
    end
end
print(f,fullfile(folder,'course_reference_tracking.png'),'-dpng','-r150'); close(f);
% Regenerate only diagrams whose saved default source changed.
for k=1:3
    model=s.models{k}; load_system(fullfile(project.simulink,[model '.slx']));
    names={'question_4_diagram.png','question_5_diagram.png','position_integral_diagram.png'};
    print(['-s' model],'-dpng','-r150',fullfile(folder,names{k}));
end
print('-saero2_position_integral/Position integral control','-dpng','-r150', ...
    fullfile(folder,'position_integral_controller.png'));
end
