function p = motor_ip_parameters
%MOTOR_IP_PARAMETERS Retain Question 1 gains and add back-calculation tracking.
p = motor_pi_parameters;
p.trackingTime = 0.05;
p.awGain = 1/p.trackingTime;
p.beta = 0; % 0: IP, 1: PI. Integral action always uses full speed error.
p.recoveryTime = 1.1;
end
