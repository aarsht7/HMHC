
%Thanh Nguyen
% This function to plot the measured force plate data vs the computed ground reaction ...force
% and moments 
filename = "ForceData/" + Motion;
filename = strcat(filename, '.csv');%ex: filename = 'ForceData/medJump'
F = csvread(filename,  1, 1);
T = F(:,1);
Fx = F(:,2);
Fy = F(:,3);
Fz = F(:,4);
Mx = F(:,5);
My = F(:,6);
Mz = F(:,7);
% find offset
% finding time of 1st peak in Forceplate data
for i=1:length(T)-(length(T)/5)
    grad1 = Fz(i+10)-Fz(i);
    if grad1 > 50 
        t1=i;
        break
    end
end
% finding time of 1st peak in Computed data
for i=1:length(F_Grnd)-50
    grad2 = F_Grnd(3,i+1)-F_Grnd(3,i);
    if grad2 > 100 
        t2=i;
        break
    end
end


%plot 
%close all;
k = size(F_Grnd,2) ;
f = 1/60;
t = 0 +  (f:f:(f*k)) ; %add offset here: change the offset here to match with force data 
% time of 1st peak in force plate and computed data 
offset = T(t1)-t(t2);
t = t+offset;

figure(111); hold on;
subplot(3,1,1); plot(T,Fx);title('force Fx (N)'); hold on;
plot(t,F_Grnd(1,1:k));hold on;
legend('measured','computed');
xlabel('time (s)');
subplot(3,1,2); plot(T,Fy);title('force Fy (N)'); hold on;
plot(t,F_Grnd(2,1:k));hold on;
xlabel('time (s)');
subplot(3,1,3); plot(T,Fz);title('force Fz (N)'); hold on;
plot(t,F_Grnd(3,1:k));hold on;
xlabel('time (s)');

figure(222);hold on;
subplot(3,1,1); plot(T,Mx);title('moment Mx (N.m)'); hold on;
plot(t,M_Grnd(1,1:k));hold on;
legend('measured', 'computed');
xlabel('time (s)');
subplot(3,1,2); plot(T,My);title('moment My (N.m)'); hold on;
plot(t,M_Grnd(2,1:k));hold on;
xlabel('time (s)');
subplot(3,1,3); plot(T,Mz);title('moment Mz (N.m)'); hold on;
plot(t,M_Grnd(3,1:k));hold on;
xlabel('time (s)');

