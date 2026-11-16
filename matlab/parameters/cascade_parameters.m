function p = cascade_parameters
%CASCADE_PARAMETERS Connect the gains already designed in Questions 2 and 4.
p = position_control_parameters;
p.motor = motor_ip_parameters;
p.initialState = zeros(6,1); % [yaw; pitch; yaw rate; pitch rate; main; tail].
p.planningTolerance = 0.20;
p.finalWindow = 2;
end
