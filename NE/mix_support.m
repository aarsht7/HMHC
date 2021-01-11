%%Thanh Nguyen 09/12/2020

%This function is to calculate the vel and acc of segments
addpath('../Hanavan_model')
load("P.mat")
global Motion M
load(Motion+"_q.mat")
%% calibration parameter
a = 1e-3; %from milimeter to meter
b = pi/180; %degree to radian
dt = 1/60; %time step
fpass = 1; %low pass filter
for i = 1:1:length(motion(1).J)
 model_sq(i) = hanavan(P, M, i, motion);%left and right are reverse Arsh messed it up :)
 %% Right hand 
 RH_com(:,i) = a*model_sq(i).L_Hand.CoM;
 RH_orient(:,i) = b*model_sq(i).L_Hand.orient;
 RH_rout(:,i) = a*model_sq(i).L_Hand.r_out;
 RH_rin(:,i) = a*model_sq(i).L_Hand.r_in;
 %% Left hand 
 LH_com(:,i) = a*model_sq(i).R_Hand.CoM;
 LH_orient(:,i) = b*model_sq(i).R_Hand.orient;
 LH_rout(:,i) = a*model_sq(i).R_Hand.r_out;
 LH_rin(:,i) = a*model_sq(i).R_Hand.r_in;
 %% Right forearm 
 RFa_com(:,i) = a*model_sq(i).L_Forearm.CoM;
 RFa_orient(:,i) = b*model_sq(i).L_Forearm.orient;
 RFa_rout(:,i) = a*model_sq(i).L_Forearm.r_out;
 RFa_rin(:,i) = a*model_sq(i).L_Forearm.r_in;
 %% Left forearm 
 LFa_com(:,i) = a*model_sq(i).R_Forearm.CoM;
 LFa_orient(:,i) = b*model_sq(i).R_Forearm.orient;
 LFa_rout(:,i) = a*model_sq(i).R_Forearm.r_out;
 LFa_rin(:,i) = a*model_sq(i).R_Forearm.r_in;
 %% Right Upperarm 
 RUa_com(:,i) = a*model_sq(i).L_Upperarm.CoM;
 RUa_orient(:,i) = b*model_sq(i).L_Upperarm.orient;
 RUa_rout(:,i) = a*model_sq(i).L_Upperarm.r_out;
 RUa_rin(:,i) = a*model_sq(i).L_Upperarm.r_in;
 %% Left Upperarm 
 LUa_com(:,i) = a*model_sq(i).R_Upperarm.CoM;
 LUa_orient(:,i) = b*model_sq(i).R_Upperarm.orient;
 LUa_rout(:,i) = a*model_sq(i).R_Upperarm.r_out;
 LUa_rin(:,i) = a*model_sq(i).R_Upperarm.r_in;
 %% Right Foot 
 RF_com(:,i) = a*model_sq(i).L_Foot.CoM;
 RF_orient(:,i) = b*model_sq(i).L_Foot.orient;
 RF_rout(:,i) = a*model_sq(i).L_Foot.r_out;
 RF_rin(:,i) = a*model_sq(i).L_Foot.r_in;
 %% Left Foot 
 LF_com(:,i) = a*model_sq(i).R_Foot.CoM;
 LF_orient(:,i) = b*model_sq(i).R_Foot.orient;
 LF_rout(:,i) = a*model_sq(i).R_Foot.r_out;
 LF_rin(:,i) = a*model_sq(i).R_Foot.r_in;
 %% Right Shank 
 RS_com(:,i) = a*model_sq(i).L_Shank.CoM;
 RS_orient(:,i) = b*model_sq(i).L_Shank.orient;
 RS_rout(:,i) = a*model_sq(i).L_Shank.r_out;
 RS_rin(:,i) = a*model_sq(i).L_Shank.r_in;
 %% Left Thigh 
 LS_com(:,i) = a*model_sq(i).R_Shank.CoM;
 LS_orient(:,i) = b*model_sq(i).R_Shank.orient;
 LS_rout(:,i) = a*model_sq(i).R_Shank.r_out;
 LS_rin(:,i) = a*model_sq(i).R_Shank.r_in;
 %% Right Thigh 
 RT_com(:,i) = a*model_sq(i).L_Thigh.CoM;
 RT_orient(:,i) = b*model_sq(i).L_Thigh.orient;
 RT_rout(:,i) = a*model_sq(i).L_Thigh.r_out;
 RT_rin(:,i) = a*model_sq(i).L_Thigh.r_in;
 %% Left Thigh 
 LT_com(:,i) = a*model_sq(i).R_Thigh.CoM;
 LT_orient(:,i) = b*model_sq(i).R_Thigh.orient;
 LT_rout(:,i) = a*model_sq(i).R_Thigh.r_out;
 LT_rin(:,i) = a*model_sq(i).R_Thigh.r_in;
 %% Head 
 HD_com(:,i) = a*model_sq(i).Head.CoM;
 HD_orient(:,i) = b*model_sq(i).Head.orient;
 HD_rout(:,i) = a*model_sq(i).Head.r_out;
 HD_rin(:,i) = a*model_sq(i).Head.r_in;
  %% Lower Trunk
 LTr_com(:,i) = a*model_sq(i).lowerTrunk.CoM;
 LTr_orient(:,i) = b*model_sq(i).lowerTrunk.orient;
 LTr_rout(:,:,i) = a*model_sq(i).lowerTrunk.r_out;
 LTr_rin(:,i) = a*model_sq(i).lowerTrunk.r_in;
   %% Upper+middle Trunk
 umTr_com(:,i) = a*model_sq(i).umTrunk.CoM;
 umTr_orient(:,i) = b*model_sq(i).umTrunk.orient;
 umTr_rout(:,i) = a*model_sq(i).umTrunk.r_out;
 umTr_rin(:,:,i) = a*model_sq(i).umTrunk.r_in;
 %% distance between two feet along Z axis

d(i) = (RF_com(3,i)- LF_com(3,i));
%suppose left foot always is the standing foot
end
%% Velocities and acceleration
%Arms
[RH_v, RH_a] = get_vel_acc(RH_com,RH_orient,dt);
[LH_v, LH_a] = get_vel_acc(LH_com,LH_orient,dt);
[RFa_v, RFa_a] = get_vel_acc(RFa_com,RFa_orient,dt);
[LFa_v, LFa_a] = get_vel_acc(LFa_com,LFa_orient,dt);
[RUa_v, RUa_a] = get_vel_acc(RUa_com,RUa_orient,dt);
[LUa_v, LUa_a] = get_vel_acc(LUa_com,LUa_orient,dt);
%Legs
[RF_v, RF_a] = get_vel_acc(RF_com,RF_orient,dt);
[LF_v, LF_a] = get_vel_acc(LF_com,LF_orient,dt);
[RS_v, RS_a] = get_vel_acc(RS_com,RS_orient,dt);
[LS_v, LS_a] = get_vel_acc(LS_com,LS_orient,dt);
[RT_v, RT_a] = get_vel_acc(RT_com,RT_orient,dt);
[LT_v, LT_a] = get_vel_acc(LT_com,LT_orient,dt);
%body
[HD_v, HD_a] = get_vel_acc(HD_com,HD_orient,dt);
[LTr_v, LTr_a] = get_vel_acc(LTr_com,LTr_orient,dt);
[umTr_v, umTr_a] = get_vel_acc(umTr_com,umTr_orient,dt);

model = model_sq(1);% take parameter from the 1st instant

%%%%%Ground Reaction Force %%%%%%%%%

%% Neck joint
 Fin = [0;0;0]; Min=[0;0;0];
for i = 1:1:length(motion(1).J)
[F_neck(:,i), M_neck(:,i)] = NE_one_body(Fin,Min,model.Head.mass,model.Head.I,...
  HD_rin(:,i),HD_rout(:,i), HD_v(:,i), HD_a(:,i));
end


%% Right shoulder joint 
Fin = [0;0;0]; Min=[0;0;0];
for i = 1:1:length(motion(1).J)
[F_RH(:,i), M_RH(:,i)] = NE_one_body(Fin,Min,model.L_Hand.mass,model.L_Hand.I,...
  model.L_Hand.r_in,model.L_Hand.r_out, RH_v(:,i), RH_a(:,i));
[F_RFa(:,i), M_RFa(:,i)] = NE_one_body(-F_RH(:,i), -M_RH(:,i),model.L_Forearm.mass,model.L_Forearm.I,...
  model.L_Forearm.r_in,model.L_Forearm.r_out, RFa_v(:,i), RFa_a(:,i));
[F_RShd(:,i), M_RShd(:,i)] = NE_one_body(-F_RFa(:,i), -M_RFa(:,i),model.L_Upperarm.mass,model.L_Upperarm.I,...
  model.L_Upperarm.r_in,model.L_Upperarm.r_out, RUa_v(:,i), RUa_a(:,i));
end
%% Left shoulder joint
Fin = [0;0;0]; Min=[0;0;0];
for i = 1:1:length(motion(1).J)
[F_LH(:,i), M_LH(:,i)] = NE_one_body(Fin,Min,model.R_Hand.mass,model.R_Hand.I,...
  model.R_Hand.r_in,model.R_Hand.r_out, LH_v(:,i), LH_a(:,i));
[F_LFa(:,i), M_LFa(:,i)] = NE_one_body(-F_LH(:,i), -M_LH(:,i),model.R_Forearm.mass,model.R_Forearm.I,...
  model.R_Forearm.r_in,model.R_Forearm.r_out, LFa_v(:,i), LFa_a(:,i));
[F_LShd(:,i), M_LShd(:,i)] = NE_one_body(-F_LFa(:,i), -M_LFa(:,i),model.R_Upperarm.mass,model.R_Upperarm.I,...
  model.R_Upperarm.r_in,model.R_Upperarm.r_out, LUa_v(:,i), LUa_a(:,i));
end
%% Upper+middle Trunk -- waist joint
for i = 1:1:length(motion(1).J)
    Fin = [-F_neck(:,i),-F_RShd(:,i),-F_LShd(:,i)];
    Min = [-M_neck(:,i),-M_RShd(:,i),-M_LShd(:,i)];
[F_umTr(:,i), M_umTr(:,i)] = NE_one_body(Fin,Min,model.umTrunk.mass,model.umTrunk.I,...
  model.umTrunk.r_in,model.umTrunk.r_out, umTr_v(:,i), umTr_a(:,i));
end
%criteria to justify whether the actor has lifted the foot or not is
%compare the difference betweem d(i) = R_Foot.CoM(3,i) - L_Foot(3,i). If it is
%larger than at least half of the length of the foot, then we assume it is
%lifted off ground 
%two cases: double support and single support
do = 1*a*model.R_Foot.L;


for i = 1:1:length(motion(1).J)
    if (d(i) - do) < 0 %double support
        %% Lower trunk -- hip joints
        Fin = -F_umTr(:,i);
        Min = -M_umTr(:,i);
        [F_LTr(:,i), M_LTr(:,i)] = NE_one_body(Fin,Min,model.lowerTrunk.mass,model.lowerTrunk.I,...
            model.lowerTrunk.r_in,model.lowerTrunk.r_out, LTr_v(:,i), LTr_a(:,i));
        F_LTr(:,i) = 1/2*F_LTr(:,i);
        M_LTr(:,i) = 1/2*M_LTr(:,i);
        
        %% Ground reaction force and Moment
        Fin = -F_LTr(:,i);
        Min = -M_LTr(:,i);
        %Right foot
        
        [F_RT(:,i), M_RT(:,i)] = NE_one_body(Fin,Min,model.L_Thigh.mass,model.L_Thigh.I,...
            model.L_Thigh.r_in,model.L_Thigh.r_out, RT_v(:,i), RT_a(:,i));
        [F_RS(:,i), M_RS(:,i)] = NE_one_body(-F_RT(:,i), -M_RT(:,i),model.L_Shank.mass,model.L_Shank.I,...
            model.L_Shank.r_in,model.L_Shank.r_out, RS_v(:,i), RS_a(:,i));
        [F_GrndR(:,i), M_GrndR(:,i)] = NE_one_body(-F_RS(:,i), -M_RS(:,i),model.L_Foot.mass,model.L_Foot.I,...
            model.L_Foot.r_in,model.L_Foot.r_out, RF_v(:,i), RF_a(:,i));
        
        %left foot
        
        [F_LT(:,i), M_LT(:,i)] = NE_one_body(Fin,Min,model.R_Thigh.mass,model.R_Thigh.I,...
            model.R_Thigh.r_in,model.R_Thigh.r_out, LT_v(:,i), LT_a(:,i));
        [F_LS(:,i), M_LS(:,i)] = NE_one_body(-F_LT(:,i), -M_LT(:,i),model.R_Shank.mass,model.R_Shank.I,...
            model.R_Shank.r_in,model.R_Shank.r_out, LS_v(:,i), LS_a(:,i));
        [F_GrndL(:,i), M_GrndL(:,i)] = NE_one_body(-F_LS(:,i), -M_LS(:,i),model.R_Foot.mass,model.R_Foot.I,...
            model.R_Foot.r_in,model.R_Foot.r_out, LF_v(:,i), LF_a(:,i));
        F_Grnd(:,i) = F_GrndL(:,i) + F_GrndR(:,i);
        M_Grnd(:,i) = M_GrndL(:,i) + M_GrndR(:,i);
    else %d(i)-do >= 0, single support
        %% Right foot
        Fin = [0;0;0]; Min=[0;0;0];
        RF_rin = model.L_Foot.r_out;
        RF_rout = model.L_Foot.r_in;
        
        [F_RF(:,i), M_RF(:,i)] = NE_one_body(Fin,Min,model.L_Foot.mass,model.L_Foot.I,...
           RF_rin,RF_rout, RF_v(:,i), RF_a(:,i));
        RS_rin = model.L_Shank.r_out;
        RS_rout = model.L_Shank.r_in;
        [F_RS(:,i), M_RS(:,i)] = NE_one_body(-F_RF(:,i), -M_RF(:,i),model.L_Shank.mass,model.L_Shank.I,...
            RS_rin,RS_rout, RS_v(:,i), RS_a(:,i));
        RT_rin = model.L_Thigh.r_out;
        RT_rout = model.L_Thigh.r_in;
        [F_RT(:,i), M_RT(:,i)] = NE_one_body(-F_RS(:,i), -M_RS(:,i),model.L_Thigh.mass,model.L_Thigh.I,...
            RT_rin,RT_rout, RT_v(:,i), RT_a(:,i));
        
        %%%%%%%%%%%%%%%%%
        
        %% Lower trunk -- hip joints
        Fin = [-F_umTr(:,i),-F_RT(:,i)];
        Min = [-M_umTr(:,i),-M_RT(:,i)];
        r_in = [model.lowerTrunk.r_in,model.lowerTrunk.r_out(:,1)]; %note: right leg become another input 
        %to the lowerTrunk
        r_out = [model.lowerTrunk.r_out(:,2)];%the only output of lowerTrunk is left leg
        [F_LTr(:,i), M_LTr(:,i)] = NE_one_body(Fin,Min,model.lowerTrunk.mass,model.lowerTrunk.I,...
            r_in, r_out, LTr_v(:,i), LTr_a(:,i));
        
        
        
        
        %% Ground reaction force and Moment
        %left foot
        Fin = -F_LTr(:,i);
        Min = -M_LTr(:,i);
        [F_LT(:,i), M_LT(:,i)] = NE_one_body(Fin,Min,model.R_Thigh.mass,model.R_Thigh.I,...
            model.R_Thigh.r_in,model.R_Thigh.r_out, LT_v(:,i), LT_a(:,i));
        [F_LS(:,i), M_LS(:,i)] = NE_one_body(-F_LT(:,i), -M_LT(:,i),model.R_Shank.mass,model.R_Shank.I,...
            model.R_Shank.r_in,model.R_Shank.r_out, LS_v(:,i), LS_a(:,i));
        [F_GrndL(:,i), M_GrndL(:,i)] = NE_one_body(-F_LS(:,i), -M_LS(:,i),model.R_Foot.mass,model.R_Foot.I,...
            model.R_Foot.r_in,model.R_Foot.r_out, LF_v(:,i), LF_a(:,i));
        F_Grnd(:,i) = F_GrndL(:,i);
        M_Grnd(:,i) = M_GrndL(:,i); 
    end
    
end
%low pass filtering 
% F_Grnd = lowpass(F_Grnd, fpass, 1/dt);
% M_Grnd = lowpass(M_Grnd, fpass, 1/dt);

%% test plot
% k = size(F_Grnd,2) - 100;
% f = 1/60
% t = 1.2+ (f:f:(f*k));
% 
% figure(123);
% subplot(3,1,1); 
% plot(t,F_Grnd(1,1:k));title('computed Fx');hold on;
% subplot(3,1,2); 
% plot(t,F_Grnd(2,1:k));title('computed Fy');hold on;
% subplot(3,1,3); 
% plot(t,F_Grnd(3,1:k));title('computed Fz');hold on;
% 
% figure(456);
% subplot(3,1,1); 
% plot(t,M_Grnd(1,1:k));title('computed Mx');hold on;
% subplot(3,1,2); 
% plot(t,M_Grnd(2,1:k));title('computed My');hold on;
% subplot(3,1,3); 
% plot(t,M_Grnd(3,1:k));title('computed Mz');hold on;

