dt = 0.0167; % Camera capture frequency = 60Hz

%% Pendulum comparision

CoM_pose_1P = [0;0;0];
V_CoM_1P = [];
A_CoM_1P = [];
CoM_pose_2P = [0;0;0;0;0;0];
V_CoM_2P = [];
A_CoM_2P = [];
for i=1:length(motion(1).J)
    model = hanavan(P,M,i,motion);


    m = (model.R_Hand.mass+model.L_Hand.mass+ ...
                    model.R_Forearm.mass+model.L_Forearm.mass+ ...
                    model.R_Upperarm.mass+model.L_Upperarm.mass+ ...
                    model.R_Foot.mass+model.L_Foot.mass+model.R_Shank.mass+...
                    model.L_Shank.mass+model.R_Thigh.mass+model.L_Thigh.mass+...
                    model.Head.mass+model.umTrunk.mass+model.lowerTrunk.mass);

    CoM_pose_1P(:,i+1) = (model.R_Hand.CoM .* model.R_Hand.mass+ ...
                    model.L_Hand.CoM .* model.L_Hand.mass+ ...
                    model.R_Forearm.CoM .* model.R_Forearm.mass+ ...
                    model.L_Forearm.CoM .* model.L_Forearm.mass+ ...
                    model.R_Upperarm.CoM .* model.R_Upperarm.mass+ ...
                    model.L_Upperarm.CoM .* model.L_Upperarm.mass+ ...
                    model.R_Foot.CoM .* model.R_Foot.mass+ ...
                    model.L_Foot.CoM .* model.L_Foot.mass+ ...
                    model.R_Shank.CoM .* model.R_Shank.mass+ ...
                    model.L_Shank.CoM .* model.L_Shank.mass+ ...
                    model.R_Thigh.CoM .* model.R_Thigh.mass+ ...
                    model.L_Thigh.CoM .* model.L_Thigh.mass+ ...
                    model.Head.CoM .* model.Head.mass+ ...
                    model.umTrunk.CoM .* model.umTrunk.mass+...
                    model.lowerTrunk.CoM .* model.lowerTrunk.mass)/m;



     m2 = (model.R_Hand.mass+model.L_Hand.mass+ ...
            model.R_Forearm.mass+model.L_Forearm.mass+ ...
            model.R_Upperarm.mass+model.L_Upperarm.mass+ ...
            model.Head.mass+model.umTrunk.mass+model.lowerTrunk.mass);
     CoM_pose_2P(4:6,i+1) = (model.R_Hand.CoM .* model.R_Hand.mass+ ...
                    model.L_Hand.CoM .* model.L_Hand.mass+ ...
                    model.R_Forearm.CoM .* model.R_Forearm.mass+ ...
                    model.L_Forearm.CoM .* model.L_Forearm.mass+ ...
                    model.R_Upperarm.CoM .* model.R_Upperarm.mass+ ...
                    model.L_Upperarm.CoM .* model.L_Upperarm.mass+ ...
                    model.Head.CoM .* model.Head.mass+ ...
                    model.umTrunk.CoM .* model.umTrunk.mass+...
                    model.lowerTrunk.CoM .* model.lowerTrunk.mass)/m2;

    m1 = (model.R_Foot.mass+model.L_Foot.mass+model.R_Shank.mass+...
          model.L_Shank.mass+model.R_Thigh.mass+model.L_Thigh.mass);       
    CoM_pose_2P(1:3,i+1) = (model.R_Foot.CoM .* model.R_Foot.mass+ ...
                    model.L_Foot.CoM .* model.L_Foot.mass+ ...
                    model.R_Shank.CoM .* model.R_Shank.mass+ ...
                    model.L_Shank.CoM .* model.L_Shank.mass+ ...
                    model.R_Thigh.CoM .* model.R_Thigh.mass+ ...
                    model.L_Thigh.CoM .* model.L_Thigh.mass)/m1;


    V_CoM_1P(:,i) =  (CoM_pose_1P(:,i+1)-CoM_pose_1P(:,i))/dt/1000;
    V_CoM_2P(:,i) =  (CoM_pose_2P(:,i+1)-CoM_pose_2P(:,i))/dt/1000;

    A_CoM_1P(:,i) =  (CoM_pose_1P(:,i+1)-CoM_pose_1P(:,i))/dt^2/1000;
    A_CoM_2P(:,i) =  (CoM_pose_2P(:,i+1)-CoM_pose_2P(:,i))/dt^2/1000;


end



%% single pendulum

for i=1:length(CoM_pose_1P)-1
    pos = [CoM_pose_1P(1,i+1),CoM_pose_1P(3,i+1)]; % in sagittal plane
    vel = [V_CoM_1P(1,i);V_CoM_1P(3,i)];
    acc = [A_CoM_1P(1,i);A_CoM_1P(3,i)];
    L = norm(pos);
    v = sqrt(vel(1)^2 + vel(2)^2);
    a = sqrt(acc(1)^2 + acc(2)^2);
    cost = pos(1)/L;
    sint = pos(2)/L;
    omega = v/L;
    domega = a/L;
    F_xP(i) = m*L*(domega*cost - omega^2 * sint);
    F_zP(i) = m*9.81 - m*L*(domega*sint + omega^2 * cost);
    M_yP(i) = L*(F_xP(i)*cost + F_zP(i)*sint);
end




%% Dual pendulum
for i=1:length(CoM_pose_2P)-1
    pos1 = [CoM_pose_2P(1,i+1),CoM_pose_2P(3,i+1)]; % in sagittal plane
    vel1 = [V_CoM_2P(1,i);V_CoM_2P(3,i)];
    acc1 = [A_CoM_2P(1,i);A_CoM_2P(3,i)];
    L1 = norm([motion(1).J(1,1),motion(1).J(1,3)]);
    s1 = norm(pos1);
    v1 = sqrt(vel1(1)^2 + vel1(2)^2);
    a1 = sqrt(acc1(1)^2 + acc1(2)^2);
    omega1 = v1/s1;
    domega1 = a1/s1;
    theta1 = atan2(pos1(1),pos1(2));
    cost1= cos(theta1);
    sint1= sin(theta1);
    a1 = s1*(domega1*[cost1;-sint1] - omega^2*[sint1;cost1]);
    F_x1 = m1*a1(1);
    F_z1 = m1*a1(2) + m1*9.81;
    
    pos2 = [CoM_pose_2P(4,i+1),CoM_pose_2P(6,i+1)]; % in sagittal plane
    vel2 = [V_CoM_2P(4,i);V_CoM_2P(6,i)];
    acc2 = [A_CoM_2P(4,i);A_CoM_2P(6,i)];
    p2 = (L1*[sint1;cost1] - pos2);
    theta2 = atan2(p2(1),p2(2));
    s2 = norm(p2);
    v2 = sqrt(vel2(1)^2 + vel2(2)^2);
    a2 = sqrt(acc2(1)^2 + acc2(2)^2);
    omega2 = omega1+(v2/s2);
    domega2 = domega1 + (a2/s2);
    cost2 = cos(theta1+theta2);
    sint2 = sin(theta1+theta2);
    a2= omega1*L1*[cost1;-sint1] + s2*((domega1+domega2)*[cost2;-sint2] - (domega1+domega2)^2*[sint2;cost2]);
    F_x2 = m2*a2(1);
    F_z2 = m2*a2(2) + m2*9.81;
    
    F_xP2(i) = F_x1 + F_x2;
    F_zP2(i) = F_z1 + F_z2;
    M_yP2(i) = s1*(F_x1*cost1 + F_z1*sint1) + (L1*cost1 + s2*cost2)*F_x2+...
                (L1*sint1 + s2*sint2)*F_x2 ;
    
end


figure()
subplot(3,1,1)
hold on
plot(F_xP(2:end),'r');hold on
plot(F_xP2(2:end),'g');hold on
plot(F_Grnd(1,:),'b');
xlabel('instance');
ylabel('Fx(N)');
legend('Single pendulum','Dual pendulum','NE')
hold on
subplot(3,1,2)
hold on
plot(F_zP(2:end),'r');hold on
plot(F_zP2(2:end),'g');hold on
plot(F_Grnd(3,:),'b');
xlabel('instance');
ylabel('Fz(N)');
legend('Single pendulum','Dual pendulum','NE')
hold on
subplot(3,1,3)
hold on
plot(M_yP(2:end),'r');hold on
plot(M_yP2(2:end),'g');hold on
plot(M_Grnd(2,:),'b');
xlabel('instance');
ylabel('My(N)');
legend('Single pendulum','Dual pendulum','NE')

figure()
plot(CoM_pose_1P(1,2:end)/1000,CoM_pose_1P(3,2:end)/1000)
title('CoM motion in Sagittal plane (single pendulum)')
xlabel('X pos(m)')
ylabel('Z pos(m)')
xlim([-0.5 0.5])
ylim([0 2])

figure()
plot(CoM_pose_2P(1,2:end)/1000,CoM_pose_2P(3,2:end)/1000);hold on
plot(CoM_pose_2P(4,2:end)/1000,CoM_pose_2P(6,2:end)/1000);
title('CoM motion in Sagittal plane (dual pendulum)')
xlabel('X pos(m)')
ylabel('Z pos(m)')
xlim([-0.5 0.5])
ylim([0 2])
