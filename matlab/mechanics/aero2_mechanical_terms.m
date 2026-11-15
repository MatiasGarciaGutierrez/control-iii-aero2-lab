function [M, c, g, Gamma] = aero2_mechanical_terms(q, dq)
%AERO2_MECHANICAL_TERMS M*q_ddot+c+g=Gamma*(omega.*abs(omega)).
% Signal order: q=[yaw;pitch], dq=[yaw rate;pitch rate], omega=[main;tail].
p = aero2_mechanical_parameters;
theta = q(2);
cor = p.Jyaw-p.jyaw;
M = [p.Jyaw*cos(theta)^2+p.jyaw*sin(theta)^2, 0; 0, p.Jpitch];
c = [-cor*sin(2*theta)*dq(2)*dq(1); ...
    0.5*cor*sin(2*theta)*dq(1)^2];
g = [0; p.gravity*sin(theta)];
Gamma = [cos(theta), 0; 0, 1]*[-p.kt, p.lever*p.kf; p.lever*p.kf, p.kt];
end
