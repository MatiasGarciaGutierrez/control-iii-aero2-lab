function p=hardware_cascade_parameters
%HARDWARE_CASCADE_PARAMETERS Filter-aware candidate for professor review.
% Retain the template's 10 rad/s, zeta=0.8 rate estimator and motor-loop gains.
p=cascade_parameters;
p.kp=2.25; % Position feedback [s^-2], selected by full-cascade simulation.
p.kd=2.25; % Rate feedback [s^-1].
p.filterFrequency=10; p.filterDamping=0.8;
end
