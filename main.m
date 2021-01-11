%Thanh Nguyen 10/12/2020
% MOdified by Aarsh 17/12/2020

% For MATLAB 2020 go to motions/read.m line 175 change it to :
% save(Motion+"_q",'motion'); 

%%% What you can do from here

% You can define the type of motion you want to use
% You can control animation 
% You can control if you want to see graphs for Force and moments by NE
% You can control comparision between Human and Inverted pendulum
% You can control plotting of Inverted pendulum with harmonic motion
% If possible do not try to run 2 animation at time

clc;
close all;
global animation Motion M freq  
%% add path
addpath('NE');
addpath('animation');
addpath('motions');
addpath('Hanavan_model');

%% main 

animation           = false; % true/false
Human_Pendulum      = true; % true/false
Harmonic_pendulum   = false; % true/false
Plot_NE             = false; % true/false

% Avilable Motions : 
% custom motion: "custom" , "custom2" , "customL" , 
% hand-waving: "fastArm",  "slowArm" , "mediumArm" , "mediumArmNOSTOMP"
% one leg kicking: "fastKick" , "fastKickArm" , "mediumKick" , "mediumKickArm" , "slowKick" , "slowKickArm"
% vertical jumping: "maxJump" , "maxJump2" ,  "medJump" , "quickJump", "jumpFeetUp"
Motion = "medJump" ;

M = 807.5/9.81; % mass of person derived from some staic pose (Fz/g)

%increase the number for quick animation of Hanavan model
freq = 20; % it decreases the reading for animation



run('Run.m');
run('mix_support.m');

if Plot_NE==true
    run('plot_ground_reaction.m');
end
if Human_Pendulum==true
    run('Human_pendulum.m');
end
if Harmonic_pendulum==true
    run('Pendulum.m');
end
%% Notes
%the frequency of motion capture is f = 60Hz, it is used to compute the
%ground reaction in double_support.m and mix_support.m

%F_z is the most important to look, and should be used to matching

%F_x, F_y are often influtuated, this is normal, have to explain in the
%report.