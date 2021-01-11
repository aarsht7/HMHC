function [Vel, Acc] = get_vel_acc(CoM_pos,CoM_orient, dt)
% Orcan Maktal
% 04.12.2020
%09.12.2020 mod by Thanh
% This function is to calculate the velocity and acceleriton from the
% positional & angular test data in discrete time 

%dt : time step
% CoM_pos : has (6 x length(t)) elements

Vel(1:3,:) = gradient(CoM_pos)/dt;
Vel(4:6,:) = gradient(CoM_orient)/dt;
Acc = gradient(Vel)/dt;
end