function p = motor_pi_parameters
%MOTOR_PI_PARAMETERS Question 1: identical continuous PI loops for both motors.
p.decay = 5.78;             % Motor equation: w_dot = -decay*w + inputGain*v.
p.inputGain = 96.532;
p.targetSettlingTime = 0.3;
p.pole = 4/p.targetSettlingTime; % Initial 2% settling-time approximation.
p.kp = (2*p.pole-p.decay)/p.inputGain;
p.ki = p.pole^2/p.inputGain;
p.sampleTime = 0.002;
p.voltageLimit = 24;
p.stepTime = 0.1;
p.reference = [100 -100];   % rad/s, main and tail.
p.stopTime = 1;
end
