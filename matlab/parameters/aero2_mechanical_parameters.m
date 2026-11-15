function p = aero2_mechanical_parameters
%AERO2_MECHANICAL_PARAMETERS Simplified course model, Part II, pages 4-5.
p.Jyaw = 0.6407;
p.jyaw = 0.01;
p.Jpitch = 0.7164;
p.gravity = 0.0254;
p.kt = 4.2487e-6;
p.kf = 2.5526e-5;
p.lever = 0.167;
p.pitchLimit = pi/3; % Simulation envelope, not an approved hardware limit.
p.initialState = [0.1; 0.2; -0.05; 0.04]; % [yaw; pitch; yaw rate; pitch rate].
p.acceleration = [0.15; -0.1];
p.step = 0.002;
end
