function validate_ballbeam
%VALIDATE_BALLBEAM Reproduce the optional model and independently check its law.
project=setup_project;model='part1_ballbeam_experiment';
load_system(fullfile(project.experiments,[model '.slx']));out=sim(model);
t=(0:0.005:10)';x=interp1(out.log_x.Time,reshape(out.log_x.Data,numel(out.log_x.Time),4),t,'pchip');
u=interp1(out.log_u.Time,out.log_u.Data(:),t,'pchip');
% Independent integration of the full nonlinear plant (not the reduced law).
[ti,xi]=ode45(@dynamics,[0 10],[0.2;0;0;0],odeset('MaxStep',0.005,'RelTol',1e-8,'AbsTol',1e-10));
independent=interp1(ti,xi,t,'pchip');assert(max(abs(independent-x),[],'all')<2e-6);
expected=0.2*exp(-2*t).*(1+2*t+(2*t).^2/2+(2*t).^3/6);
last=find(abs(x(:,1))>0.004,1,'last');settling=t(last+1);
deviation=max(abs(x(:,1)-expected));angle=max(abs(x(:,3)));
assert(abs(settling-4.545)<0.01 && deviation<1e-4 && max(abs(x(:,3)))<pi/2);
assert(min(x(:,1))>-1e-6 && abs(x(end,1))<0.004);
% Temporarily record the simple/exported answer, then discard instrumentation.
for name={'part1_ballbeam','part1_ballbeam_R2021a'}
    m=name{1};load_system(fullfile(project.simulink,[m '.slx']));
    add_block('simulink/Sinks/To Workspace',[m '/Validation recording'], ...
        'VariableName','state','SaveFormat','Timeseries','MaxDataPoints','inf');
    add_line(m,'Mux x/1','Validation recording/1');r=sim(m);
    y=interp1(r.state.Time,reshape(r.state.Data,numel(r.state.Time),4),t,'pchip');
    assert(max(abs(y-x),[],'all')<2e-6);close_system(m,0);
end
% Independent nominal motor check for the gains reported in the lab script.
a=5.78;b=96.532;kp=0.8;ki=10;wf=180;
A=[-a -b*kp b;wf -wf 0;0 -ki 0];B=[0;0;ki];
tm=(0:0.0005:1)';[~,xm]=ode45(@(~,s)A*s+B*100,tm,zeros(3,1), ...
    odeset('MaxStep',0.0005,'RelTol',1e-10,'AbsTol',1e-11));
voltage=xm(:,3)-kp*xm(:,2);last=find(abs(xm(:,1)-100)>2,1,'last');
motorSettling=tm(last+1);motorPoles=eig(A);
assert(max(abs(voltage))<24 && max(xm(:,1))<=100+1e-6);
folder=fullfile(project.results,'ballbeam');if ~isfolder(folder),mkdir(folder);end
save(fullfile(folder,'validation.mat'),'t','x','u','expected','settling','deviation','angle', ...
    'motorSettling','motorPoles','voltage');
fprintf('Ball/beam: Ts %.4f s, max linear error %.4g m, peak angle %.4f rad, u(0) %.4f N m.\n', ...
    settling,deviation,angle,u(1));
fprintf('Nominal filtered motor: Ts %.4f s, peak voltage %.4f V.\n',motorSettling,max(abs(voltage)));
disp(motorPoles);
figure('Visible','off','Position',[100 100 1000 250]);tiledlayout(1,2);
nexttile;plot(t,x(:,1),'DisplayName','Nonlinear plant');hold on;
plot(t,expected,'--','DisplayName','Linear design');yline(0.004,':','HandleVisibility','off');
yline(-0.004,':','HandleVisibility','off');grid on;xlabel('Time [s]');ylabel('Position [m]');legend;
nexttile;plot(t,u);grid on;xlabel('Time [s]');ylabel('Torque [N m]');
file=fullfile(project.docs,'figures','report_ballbeam_response.png');exportgraphics(gcf,file,'Resolution',200);
copyfile(file,fullfile(project.root,'report','images','report_ballbeam_response.png'));close(gcf);
end

function dx=dynamics(~,x)
m=0.11;J=0.02;g=9.81;B=5/7;
r=x(1);dr=x(2);theta=x(3);rate=x(4);
z=[r;dr;-B*g*sin(theta);-B*g*rate*cos(theta)];v=-[16 32 24 8]*z;
u=(m*r^2+J)*rate^2*tan(theta)+2*m*r*dr*rate+m*g*r*cos(theta) ...
    -(m*r^2+J)/(B*g*cos(theta))*v;
dx=[dr;B*(r*rate^2-g*sin(theta));rate;(u-2*m*r*dr*rate-m*g*r*cos(theta))/(m*r^2+J)];
end
