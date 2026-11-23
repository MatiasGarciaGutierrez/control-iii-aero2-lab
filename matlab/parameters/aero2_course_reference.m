function [U_v,U_h,reference,referenceRate] = aero2_course_reference
%AERO2_COURSE_REFERENCE Exact course angle trajectory, yaw first, pitch second.
% Reproduce only the reference section of course/model_AERO2_simul_ini.m.
% Do not run that initializer or alter its file, gains or measurement settings.
Ts=0.002; tmax=60;
t_ref=(0:Ts:tmax)';
sstep=@(t,t0,A,T) A*(1-(1+(t-t0)/T).*exp(-(t-t0)/T)).*(t>=t0);
Tr=2.5;
yaw_ref=sstep(t_ref,5,0.5,Tr)+sstep(t_ref,25,-1.0,Tr)+sstep(t_ref,40,0.5,Tr);
pitch_ref=sstep(t_ref,2,0.3,Tr)+sstep(t_ref,15,-0.5,Tr)+sstep(t_ref,32,0.4,Tr) ...
    +0.1*sin(2*pi*0.05*(t_ref-45)).*sstep(t_ref,45,1,Tr);
U_v=timeseries(2*yaw_ref,t_ref);
U_h=timeseries(2*pitch_ref,t_ref);
reference=[t_ref U_v.Data U_h.Data];
% Analytical derivatives of the smooth course signals, sampled on the same grid.
dr=@(t,t0,A,T) A*max(t-t0,0)/T^2.*exp(-max(t-t0,0)/T);
yawRate=dr(t_ref,5,0.5,Tr)+dr(t_ref,25,-1,Tr)+dr(t_ref,40,0.5,Tr);
pitchRate=dr(t_ref,2,0.3,Tr)+dr(t_ref,15,-0.5,Tr)+dr(t_ref,32,0.4,Tr) ...
    +0.1*(2*pi*0.05*cos(2*pi*0.05*(t_ref-45)).*sstep(t_ref,45,1,Tr) ...
    +sin(2*pi*0.05*(t_ref-45)).*dr(t_ref,45,1,Tr));
referenceRate=[t_ref 2*yawRate 2*pitchRate];
end
