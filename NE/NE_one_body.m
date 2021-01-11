function [Fout, Mout] = NE_one_body(Fin, Min, m, I, r_in, r_out, vel, acc)
%Thanh Nguyen
% This fucntion is to present the NE equations of one body 

%Fin, Min: external forces and moments by the antecedent body(ies) acting on
%           the current body and/or the interaction force  w/
%           environment. In case of more than one input force, all the
%           forces will be stacked in a matrix F_in (3xN), N = numbers of
%           input forces, similar for moments.
%Fout, Mout: external force and moments by the next body(ies) acting on the
%           current body or the interaciton force w/ environment. 
%Xd, Xdd: the velocity and acceleration of COM
%r_in, r_out: the distance vectors from the joints to COM corresponding
%           to Fin and Fout. r_rin has same size with F_in, r_out has same
%           size with F_out.
%vel, acc: velocity and acceleration of COM
%equations are written in the frame origin at COM 
% NE equation: m*v' = Fin + Fout + m*g
%              I*omega'' + omega x (I*omega) = tau_in + tau_out + r_in x
%              Fin + r_out x Fout


k=size(Fin,2);
g = [0;0;-9.81]; 
omega = vel(4:6);
vdot = acc(1:3);
omegadot = acc(4:6);
Fout =  m*vdot - m*g;
Mout = (I*omegadot + cross(omega,(I*omega)));
for i=1:1:k
Fout = Fout - Fin(:,i,1);
Mout = Mout  - Min(:,i,1)- cross(0.001*r_in(:,i,1), Fin(:,i,1));
end
Mout = Mout- cross(0.001*r_out(:,1,1), Fout(:,1,1));
end