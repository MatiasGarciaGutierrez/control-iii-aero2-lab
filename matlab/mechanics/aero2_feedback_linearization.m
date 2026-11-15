function [omegaDesired, s] = aero2_feedback_linearization(q, dq, a)
%AERO2_FEEDBACK_LINEARIZATION Ideal motor tracking: choose omega for q_ddot=a.
% The signed square root is defined at zero but has no finite derivative there.
p = aero2_mechanical_parameters;
assert(abs(q(2)) <= p.pitchLimit, 'Pitch envelope exceeded (limit 60 deg).');
[M, c, g, Gamma] = aero2_mechanical_terms(q, dq);
s = Gamma\(M*a+c+g);
omegaDesired = sign(s).*sqrt(abs(s));
end
