function export_cstr_figures
%EXPORT_CSTR_FIGURES Export diagrams and academic simulation evidence.
project = setup_project;folder = fullfile(project.docs,'figures');model = 'part1_cstr_tracking';
load_system(fullfile(project.simulink,[model '.slx']));
cleanup = onCleanup(@()close_system(model,0)); %#ok<NASGU>
print(['-s' model],'-dpng','-r150',fullfile(folder,'cstr_tracking_diagram.png'));
print(['-s' model '/Linearizing controller'],'-dpng','-r150',fullfile(folder,'cstr_linearizing_controller.png'));
recording = load(fullfile(project.results,'cstr_validation.mat'),'runs','C330');run = recording.runs{1};
handle = figure('Visible','off','Color','w','Position',[100 100 1200 300]);
subplot(1,3,1);plot(run.time,run.T,'LineWidth',1.6);hold on;
plot(run.time,run.reference,'--','LineWidth',1.1);xlim([0 10]);grid on;
ylabel('Temperature [K]');xlabel('Time [s]');legend('Reactor','Reference','Location','southeast');
subplot(1,3,3);plot(run.time,run.Tc,'LineWidth',1.6);xlim([0 10]);grid on;
ylabel('Coolant temperature [K]');xlabel('Time [s]');
subplot(1,3,2);plot(run.time,run.CA,'LineWidth',1.6);hold on;
yline(recording.C330,'--','Final equilibrium');grid on;
ylabel('Concentration [mol/L]');xlabel('Time [s]');
print(handle,fullfile(folder,'report_cstr_response.png'),'-dpng','-r180');close(handle);
copyfile(fullfile(folder,'report_cstr_response.png'),fullfile(project.root,'report','images','report_cstr_response.png'));
end
