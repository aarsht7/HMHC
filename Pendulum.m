
%% Single Inverted Pendulum 
clear all
clc

syms theta(t) F
length = 1;
m = 1;
mass = 1;
g = 9.8;

%position 
x_pos = length*sin(theta);
y_pos = length*cos(theta);
%velocity
vx = diff(x_pos);
vy = diff(y_pos);
%acceleration 
ax = diff(vx);
ay = diff(vy);
%Equation of Force
Fx = mass*ax(t) == -F*sin(theta(t));
Fy = mass*ay(t) == F*cos(theta(t)) - mass*g;
Force = solve(Fx,F);
eqn_1 = subs(Fx,F,Force);
eqn_2 = subs(Fy,F,Force);
%OSlving Differential Equations
[V,S] = odeToVectorField(eqn_1,eqn_2);
M = matlabFunction(V,'vars',{'t','Y'});
initCond = [pi/5 0];
sols = ode45(M,[0 10],initCond);


dtheta = sols.y(2,:);
thetaddot = diff(dtheta);
thetaddot = [thetaddot 0 ];
sintheta = sin(sols.y(1,:));
costheta = cos(sols.y(1,:));

x_pos = @(t) length*sin(deval(sols,t,1));
y_pos = @(t) length*cos(deval(sols,t,1));
Tau = (mass*length*length*thetaddot)+(mass*length*g*sintheta);
Force_x = mass*length*((costheta.*thetaddot) - ((dtheta).^2.*sintheta));
Force_y = mass*(length*((costheta.*(dtheta).^2)-(sintheta.*thetaddot))-g);
momentum = length.*(Force_x .*costheta + Force_y.*sintheta);

% Animation 
figure(1)
subplot(2,4,2)
plot(sols.x,sols.y(1,:))
hold on
title('Pendulum Motion')
xlabel('Time (s)')
ylabel(' \theta')
hold on
subplot(2,4,6)
plot(sols.x,Tau)
title('Torque')
xlabel('Time(s)')
ylabel('\tau')
hold on
subplot(2,4,[1 5])
title('Animation')
xlabel('x pos')
ylabel('y pos')
hold on
fanimator(@(t) plot(x_pos(t),y_pos(t),'ko','MarkerFaceColor','k'),'AnimationRange',[0 10]);
axis equal;
hold on;
fanimator(@(t) plot([0 x_pos(t)],[0 y_pos(t)],'k-'),'AnimationRange',[0 10]);
fanimator(@(t) text(-0.3,1.5,"Timer: "+num2str(t,2)),'AnimationRange',[0 10]);
hold off;
subplot(2,4,3)
fplot(x_pos,[0 10])
title('x Position')
xlabel('Time(s)')
ylabel('x')
hold on;
subplot(2,4,7)
fplot(y_pos,[0 10])
title('y Position')
xlabel('Time(s)')
ylabel('y')
hold on
subplot(2,4,4)
plot(sols.x,Force_x);
title('Reaction Force')
xlabel('Time(s)')
ylabel('F')
hold on;
subplot(2,4,4)
plot(sols.x,Force_y);
legend('Fx','Fy')
hold on 
subplot(2,4,8)
plot(sols.x,momentum);
title('momentum')
xlabel('Time(s)')
ylabel('m')
playAnimation
%% double Inverted Pendulum Dynamics and Animation
clear all
clc

syms theta_1(t) theta_2(t) F1 F2

mass1 = 1;
mass2 = 1;
g = 9.8;
length1 = 1;
length2 = 1;

x1 = length1*sin(theta_1);
y1 = -length1*cos(theta_1);
x2 = x1 + length2*sin(theta_2);
y2 = y1 - length2*cos(theta_2);

vx1 = diff(x1);
vy1 = diff(y1);
vx2 = diff(x2);
vy2 = diff(y2);

ax1 = diff(vx1);
ay1 = diff(vy1);
ax2 = diff(vx2);
ay2 = diff(vy2);

eqx1 = mass1*ax1(t) == -F1*sin(theta_1(t)) + F2*sin(theta_2(t));
eqy1 = mass1*ay1(t) == F1*cos(theta_1(t)) - F2*cos(theta_2(t)) - mass1*g ;

eqx2 = mass2*ax2(t) == -F2*sin(theta_2(t));
eqy2 = mass2*ay2(t) == F2*cos(theta_2(t)) - mass2*g;
Force = solve([eqx1 eqy1],[F1 F2]);

eqRed_1 = subs(eqx2,[F1 F2],[Force.F1 Force.F2]);
eqRed_2 = subs(eqy2,[F1 F2],[Force.F1 Force.F2]);

length1 = 1;
length2 = 1;
mass1 = 1;
mass2 = 1;
g = 9.8;
eqn_1 = subs(eqRed_1);
eqn_2 = subs(eqRed_2);
[V,S] = odeToVectorField(eqn_1,eqn_2);
M = matlabFunction(V,'vars',{'t','Y'});
initCond = [pi/4 0 pi/6 0];
sols = ode45(M,[0 10],initCond);

dtheta1 = sols.y(2,:);
thetaddot1 = diff(dtheta1);
thetaddot1 = [thetaddot1 0 ];
sintheta1 = sin(sols.y(1,:));
costheta1 = cos(sols.y(1,:));
dtheta2 = sols.y(4,:);
thetaddot2 = diff(dtheta2);
thetaddot2 = [thetaddot2 0 ];
sintheta2 = sin(sols.y(3,:));
costheta2 = cos(sols.y(3,:));
sintheta12 = sin(sols.y(1,:)+sols.y(3,:));

x1 = @(t) length1*sin(deval(sols,t,1));
y1 = @(t) length1*cos(deval(sols,t,1));
x2 = @(t) length1*sin(deval(sols,t,1))+length2*sin(deval(sols,t,3));
y2 = @(t) length1*cos(deval(sols,t,1 ))+length2*cos(deval(sols,t,3));
Tau1 =  mass1*length1*length1*thetaddot1+mass1*length1*g*sintheta1;
Tau2 = mass2*length2*length2*(thetaddot1+thetaddot2)+mass2*length2*g*sintheta12;
Forcex_1 = mass1*length1*((costheta1.*thetaddot1) - ((dtheta1).^2.*sintheta1));
Forcex_2 = mass2*((length1*(costheta1.*thetaddot1 - sintheta1.*(dtheta1).^2))+(length2*(costheta2.*thetaddot2 - sintheta2.*(dtheta2).^2)));
Forcey_2 = mass2*(length2*((costheta2.*(dtheta2).^2)-(sintheta2.*thetaddot2))-g);
Forcey_1 = mass2*(length2*((costheta2.*(dtheta2).^2)-(sintheta2.*thetaddot2))+(length1*((costheta1.*(dtheta1).^2)-(sintheta1.*thetaddot1)))-g);
momentum2 = length2.*(Forcex_2 .*costheta2 + Forcey_2.*sintheta2);
momentum1 = length1.*(Forcex_1 .*costheta1 + Forcey_1.*sintheta1)+momentum2; 

%Animation
figure(2)
plot(sols.x,sols.y)
subplot(2,4,2)
plot(sols.x,sols.y([1 3],:))
hold on
legend(' \theta1 ','\theta2')
title('Pendulum Motion')
xlabel('Time (s)')
ylabel(' \theta')
hold on
subplot(2,4,[1 5])
title('Animation')
xlabel('x pos')
ylabel('y pos')
hold on
fanimator(@(t) plot(x1(t),y1(t),'ko','MarkerFaceColor','k'));
axis equal;
hold on;
fanimator(@(t) plot([0 x1(t)],[0 y1(t)],'k-'));
fanimator(@(t) plot(x2(t),y2(t),'ko','MarkerFaceColor','k'));
fanimator(@(t) plot([x1(t) x2(t)],[y1(t) y2(t)],'k-'));
fanimator(@(t) text(-0.3,-0.3,"Timer: "+num2str(t,2)));
hold on
subplot(2,4,3)
fplot(x1,[0 10])
title('x Position')
xlabel('Time(s)')
ylabel('x')
hold on;
fplot(x2,[0 10])
legend(' x1 ','x2')
subplot(2,4,7)
fplot(y1,[0 10])
title('y Position')
xlabel('Time(s)')
ylabel('y')
hold on;
fplot(y2,[0 10])
legend(' y1 ','y2')
hold on
subplot(2,4,6)
plot(sols.x,Tau1)
hold on
plot(sols.x,Tau2)
title('Torque')
xlabel('Time(s)')
ylabel('\tau')
legend('\tau1','\tau2')
hold on;
subplot(2,4,4)
plot(sols.x,Forcex_1);
hold on;
plot(sols.x,Forcex_2);
hold on;
plot(sols.x,Forcey_1);
hold on;
plot(sols.x,Forcey_2);
title('Forces')
xlabel('Time(s)')
ylabel('F')
legend('Fx1','Fx2','Fy1','Fy2')
hold on;
subplot(2,4,8)
plot(sols.x,momentum2);
hold on
plot(sols.x,momentum1);
hold on
title('momentum')
xlabel('Time(s)')
ylabel('m')
legend('m1','m2')
playAnimation