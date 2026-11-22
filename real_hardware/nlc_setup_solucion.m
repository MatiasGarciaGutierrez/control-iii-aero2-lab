%NLC_SETUP  Workspace variables of lin_feedback_simplified.slx (run before simulating).

Ts   = 0.002;    % sampling period [s]
tmax = 60;       % simulation length [s]
meas_on = 1;     % 0: ideal sampled measurements, 1: encoder quantization + sensor noise

% simplified-model parameters (aero2_model.pdf); not used by the blocks of the model
Jph = 0.6407;
jph = 0.01;
Jth = 0.7164;
Dph = 0;               % B_q = 0
Dth = 0;
Cor = Jph - jph;
L   = 0.167;
G   = 0.0254;
kF  = 2.5526e-05;
kT  = 4.2487e-06;

% outer loop (after FL each axis is a double integrator):
% v = Kp e + Kd (dref/dt - dq_meas) + Ki int(e), triple pole at -p_out
% p_out = 1.2;
% Kd = 3*p_out;  Kp = 3*p_out^2;  Ki = p_out^3;

Kp=2.5;
Kd=2;
Ki=1;

% low-pass filter between the FL law and the signed square roots (block "LPF tau")
w_tau = 5;       % cutoff [rad/s]

% rotor-speed I-P loops with anti-windup (design: wn = 30 rad/s, zeta = 0.9 on dw = -5.78 w + 96.532 v)
% kp_PI_motor = 0.499523;
% ki_PI_motor = 9.32333;
% kw_PI_motor = 2;

kp_PI_motor=0.8;
ki_PI_motor=10;
kw_PI_motor=0.5;

% references: critically damped steps (time constant Tr) + a slow sine
% sampled at Ts (the PID differentiates the reference)
t_ref = (0:Ts:tmax)';
sstep = @(t,t0,A,T) A*(1 - (1 + (t-t0)/T).*exp(-(t-t0)/T)).*(t>=t0);
Tr = 2.5;
yaw_ref   = sstep(t_ref,5,0.5,Tr) + sstep(t_ref,25,-1.0,Tr) + sstep(t_ref,40,0.5,Tr);
pitch_ref = sstep(t_ref,2,0.3,Tr) + sstep(t_ref,15,-0.5,Tr) + sstep(t_ref,32,0.4,Tr) ...
          + 0.1*sin(2*pi*0.05*(t_ref-45)).*sstep(t_ref,45,1,Tr);
U_v = timeseries(2*yaw_ref,   t_ref);   % yaw   reference psi   [rad]
U_h = timeseries(2*pitch_ref, t_ref);   % pitch reference theta [rad]
clear t_ref sstep Tr yaw_ref pitch_ref
