% NLC_SETUP_REAL  Workspace variables of lin_feedback_real.slx (QUARC / real AERO 2).
% Run this script before building / connecting the model.
% Same controller design as nlc_setup.m (simulation), translated to the
% block names of the real-device model.

%% OPTIONS TO EDIT DEPENDING ON EXERCISE
test_mode = 1;   % 1: speed-loop test   2: full position control
ref_mode  = 1;   % 1: references of the ini file        2: plain steps (to measure ts)
spd_b     = 0;   % 1 -> PI Controller,  0 -> IP Controller
spd_aw    = 1;   % 1: anti-windup ON,  0: OFF
pos_ff    = 0;   % 1: feedforward of dq_d and ddq_d,  0: pure regulation
pos_int   = 1;   % OPTIONAL 1: 1 -> integral action in the position loop
tau_lpf   = 0;   % 1: low-pass filter on tau_d before the square roots (real-device option), 0: as in simulation
% (no "mismatch" option here: the real plant already differs from the model;
%  the MATLAB Function of the real model uses the nominal parameters)


%% WORKSPACE CONSTANTS
Ts   = 0.002;    % sampling period [s]
tmax = 60;       % run length [s]

% simplified-model parameters (handout table) - also written inside the MATLAB Function
Jph = 0.6407; jph = 0.01; Jth = 0.7164;
Dph = 0; Dth = 0; % B_q = 0
Cor = Jph - jph;
L   = 0.167; G   = 0.0254;
kF  = 2.5526e-05; kT  = 4.2487e-06;

%% SPEED BLADE PI CONTROLLER  (blocks IP_m and IP_t)
% Motor model:  dw/dt = -a_w*w + b_w*v  (PPT)
a_w = 5.78;  b_w = 96.532;
Vmax = 24;                               % Saturation of [V] (blocks "+/- 24V")

% PI  C(s) = Kp + Ki/s  ->  In CL: s^2 + (a_w + b_w*Kp) s + b_w*Ki
% Desired  s^2 + 2*zeta*wn s + wn^2  with  ts(2%) ~ 4/(zeta*wn)
ts_w   = 0.3;                            % Required settling time [s]
zeta_w = 0.8;                            % fast response with low (but existing) oscillations
wn_w   = 4/(zeta_w*ts_w);                % 16.67 rad/s
spd_Kp = (2*zeta_w*wn_w - a_w)/b_w;      % 0.2164
spd_Ki = wn_w^2/b_w;                     % 2.878
% WITH ANTI-WINDUP:
spd_Kaw = spd_aw*spd_Ki/spd_Kp;          % Back-calculation gain 1/Ti with Ti = Kp/Ki  [1/s]

% PI : v = Kp*(  w_d - w) + Ki*int(w_d - w)
% IP : v = Kp*(  0  - w) + Ki*int(w_d - w) -> Same poles as before, with no zero

% names used by the original real-device template (kept for compatibility)
kp_PI_motor = spd_Kp;
ki_PI_motor = spd_Ki;
kw_PI_motor = spd_Kaw;


%% FEEDBACK LINEARISATION  (MATLAB Function + "tau -> w" subsystems)
% With w = w_d:  Mq(q) ddq + cq(q,dq) + gq(q) = Gamma(theta) tau,  tau = [wm|wm|; wt|wt|]
% tau_d = Gamma^-1 (Mq nu + cq + gq)  ->  ddq = nu ;   w_d = sign(tau_d) sqrt(|tau_d|)
% Valid for cos(theta) ~= 0  (|theta| < pi/2) and |w_d| <= w_max.
w_max = 380;                             % From 24 V in steady state: 24*b_w/a_w = 401
w_tau = 20;                              % cutoff of the optional tau_d low-pass filter [rad/s] (only if tau_lpf = 1)


%% LINEAR POSITION CONTROLLER  (Subsystem2 = yaw, Subsystem3 = pitch)
% Linearized system: two decoupled double integrators  ddpsi = nu1, ddtheta = nu2
% nu = pos_ff*ddq_d + Kd*(pos_ff*dq_d - dq) + Kp*(bP*q_d - q) + Ki*int(q_d - q)
ts_q = 3;                                % required settling time [s]
if pos_int == 0
    % PD, critically damped (zeta = 1): exact ts(2%) = 5.834/wn
    wn_q   = 5.834/ts_q;                 % 1.945
    pos_Kp = wn_q^2;                     % 3.78
    pos_Kd = 2*wn_q;                     % 3.89
    pos_Ki = 0;
else
    % optional integral action: triple pole at -p,  ts(2%) ~ 7.52/p
    p_q    = 7.52/ts_q;                  % 2.5
    pos_Kd = 3*p_q;                      % 7.5
    pos_Kp = 3*p_q^2;                    % 18.75
    pos_Ki = p_q^3;                      % 15.6
end
if ref_mode == 2 && pos_ff == 1
    warning('Steps cannot be differentiated: pos_ff set to 0 for ref_mode = 2.');
    pos_ff = 0;
end
% set-point weight of the P term: 1 for PD or when the reference is fed forward,
% 0 (I-PD form) when the integral is used for plain set-point regulation (avoids the PID zero)
pos_bP = double(pos_ff == 1 || pos_int == 0);

% gain names used by the blocks of Subsystem2 / Subsystem3
% Kp = pos_Kp;
% Kd = pos_Kd;
% Ki = pos_Ki;

Kp = 1;
Kd = 0.5;
Ki = 0;

%% REFERENCES
if test_mode == 1
    % ---- speed-loop test: steps of the blade speed references -------------
    % WARNING (real device): the body will turn in yaw during this test.
    tmax = 9;
    t_w = (0:Ts:tmax)';
    wm_ref =  150*(t_w>=0.5) + 50*(t_w>=3) + 150*(t_w>=5) - 250*(t_w>=7);   % 0-150-200-350-100
    wt_ref = -150*(t_w>=0.5) - 50*(t_w>=3) - 150*(t_w>=5) + 250*(t_w>=7);   % same, reversed
    W_test = timeseries([wm_ref wt_ref], t_w);
    clear t_w wm_ref wt_ref
else
    W_test = timeseries([0 0; 0 0], [0; tmax]);
end

if ref_mode == 1
    % original references: critically damped steps (time constant Tr) + slow sine
    t_ref = (0:Ts:tmax)';
    sstep = @(t,t0,A,T) A*(1 - (1 + (t-t0)/T).*exp(-(t-t0)/T)).*(t>=t0);
    Tr = 2.5;
    yaw_ref   = sstep(t_ref,5,0.5,Tr) + sstep(t_ref,25,-1.0,Tr) + sstep(t_ref,40,0.5,Tr);
    pitch_ref = sstep(t_ref,2,0.3,Tr) + sstep(t_ref,15,-0.5,Tr) + sstep(t_ref,32,0.4,Tr) ...
              + 0.1*sin(2*pi*0.05*(t_ref-45)).*sstep(t_ref,45,1,Tr);
    U_v = timeseries(2*yaw_ref,   t_ref);   % yaw   reference psi   [rad]
    U_h = timeseries(2*pitch_ref, t_ref);   % pitch reference theta [rad]
    clear t_ref sstep Tr yaw_ref pitch_ref
else
    % plain steps to check the 3 s settling time
    if test_mode == 2, tmax = 20; end
    t_ref = (0:Ts:tmax)';
    U_v = timeseries( 0.5*(t_ref>=1) - 1.0*(t_ref>=11), t_ref);   % psi:   0 -> 0.5 -> -0.5 rad
    U_h = timeseries( 0.3*(t_ref>=1) - 0.5*(t_ref>=11), t_ref);   % theta: 0 -> 0.3 -> -0.2 rad
    clear t_ref
end

if spd_b == 1, spd_name = 'PI'; else, spd_name = 'IP'; end
fprintf('Speed loop : Kp = %.4f  Ki = %.4f  Kaw = %.2f  (%s)\n', spd_Kp, spd_Ki, spd_Kaw, spd_name);
fprintf('Position   : Kp = %.3f  Kd = %.3f  Ki = %.3f  ff = %d  bP = %d\n', ...
        pos_Kp, pos_Kd, pos_Ki, pos_ff, pos_bP);
fprintf('FL / w_d   : w_max = %g rad/s   tau LPF = %d (w_tau = %g rad/s)   test_mode = %d\n', ...
        w_max, tau_lpf, w_tau, test_mode);
