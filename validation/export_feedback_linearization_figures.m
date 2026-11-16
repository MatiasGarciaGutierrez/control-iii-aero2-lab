function export_feedback_linearization_figures
%EXPORT_FEEDBACK_LINEARIZATION_FIGURES Export actual model and logged signals.
project=setup_project; model='aero2_mechanical_linearization_experiment';
load_system(fullfile(project.experiments,[model '.slx']));
s=load(fullfile(project.results,'feedback_linearization_validation.mat'));
r=s.nominal; folder=fullfile(project.docs,'figures');
print(['-s' model],'-dpng','-r140',fullfile(folder,'feedback_linearization_diagram.png'));
f=figure('Visible','off','Color','w','Position',[50 50 1100 900]);
for axis=1:2
    subplot(3,2,axis);
    plot(r.time,r.data(:,axis),':k','LineWidth',2); hold on;
    plot(r.time,r.data(:,axis+2),'LineWidth',1.3); grid on;
    names={'Yaw','Pitch'}; title([names{axis} ' acceleration']);
    ylabel('Acceleration [rad/s^2]'); legend('Virtual input a','Recovered acceleration','Location','best');
    subplot(3,2,axis+2);
    t=r.time;
    expectedRate=s.p.initialState(axis+2)+s.p.acceleration(axis)* ...
        (max(t-0.2,0)-max(t-0.6,0));
    plot(t,expectedRate,':k','LineWidth',2); hold on;
    plot(t,r.data(:,axis+6),'LineWidth',1.3); grid on;
    yline(0,':k','HandleVisibility','off');
    title([names{axis} ' angular velocity']); ylabel('Angular velocity [rad/s]');
    legend('Analytical velocity','Course mechanics','Location','best');
    subplot(3,2,axis+4);
    expected=s.p.initialState(axis)+s.p.initialState(axis+2)*t + ...
        0.5*s.p.acceleration(axis)*(max(t-0.2,0).^2-max(t-0.6,0).^2);
    plot(t,expected,':k','LineWidth',2); hold on;
    plot(t,r.data(:,axis+4),'LineWidth',1.3); grid on;
    title([names{axis} ' position']); ylabel('Angle [rad]');
    legend('Analytical double integrator','Course mechanics','Location','best');
end
for k=1:6, subplot(3,2,k); xlabel('Time [s]'); xlim([0 2]); end
print(f,fullfile(folder,'feedback_linearization_response.png'),'-dpng','-r150'); close(f);
% Keep numerical residuals separate so the main figure reads a -> dq -> q.
f=figure('Visible','off','Color','w','Position',[50 50 1100 350]);
for axis=1:2
    subplot(1,2,axis);
    plot(r.time,r.data(:,axis+12),'LineWidth',1); grid on;
    title([names{axis} ' acceleration residual']); ylabel('a - recovered [rad/s^2]');
    xlabel('Time [s]'); xlim([0 2]);
end
print(f,fullfile(folder,'feedback_linearization_residual.png'),'-dpng','-r150'); close(f);
f=figure('Visible','off','Color','w','Position',[50 50 1000 370]);
plot(r.time,r.data(:,9:10),'LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('Desired rotor speed [rad/s]');
title('Algebraic rotor commands: ideal motor tracking assumed');
legend('Main rotor','Tail rotor','Location','best');
print(f,fullfile(folder,'feedback_linearization_rotors.png'),'-dpng','-r150'); close(f);
end
