function p = position_control_parameters
%POSITION_CONTROL_PARAMETERS Critically damped ideal yaw/pitch position loops.
p.targetSettlingTime = 3;
p.tolerance = 0.02;
% Exact zero-state step error: (1+lambda*t)*exp(-lambda*t).
p.settlingFactor = fzero(@(tau) (1+tau)*exp(-tau)-p.tolerance, [1 10]);
p.pole = p.settlingFactor/p.targetSettlingTime;
p.kp = p.pole^2;
p.kd = 2*p.pole;
p.reference = [0.2;0.1]; % Step-test yaw/pitch angles [rad]; saved models use the course trajectory.
p.stepTime = 0.2;
p.step = 0.002;
p.stopTime = 8; % Step-test duration; saved position models run the 60 s course reference.
p.initialState = zeros(4,1);
end
