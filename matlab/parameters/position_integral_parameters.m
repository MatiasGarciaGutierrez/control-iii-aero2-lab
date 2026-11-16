function p = position_integral_parameters
%POSITION_INTEGRAL_PARAMETERS Third-order feedback with nominal step matching.
p=cascade_parameters;
p.integralPole=p.pole;
p.referenceGain=p.pole^2;
p.positionGain=p.pole^2+2*p.pole*p.integralPole;
p.rateGain=2*p.pole+p.integralPole;
p.integralGain=p.integralPole*p.pole^2;
p.referenceWeight=p.referenceGain/p.positionGain;
p.disturbance=[0.01;-0.008]; % Constant yaw/pitch torque [N m], experiment only.
p.disturbanceTime=5;
p.disturbanceStop=20;
end
