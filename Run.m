% Aarsh - December 17 

addpath('animation')
addpath('motions');
addpath('Hanavan_model');

global Motion M Hanavan animation

% Variables to define in this file
% Motion = ""
% M = mass in Kg
% i = instance
% animation = true/false
% generate = true/false
% freq = frequency to read data for motion
%
% Avilable Motions : 
% custom motion: "custom" , "custom2" , "customL" , 
% hand-waving: "fastArm",  "slowArm" , "mediumArm" , "mediumArmNOSTOMP"
% one leg kicking: "fastKick" , "fastKickArm" , "mediumKick" , "mediumKickArm" , "slowKick" , "slowKickArm"
% vertical jumping: "maxJump" , "maxJump2" ,  "medJump" , "quickJump", "jumpFeetUp"
% sitting: "slowSit" no force data for slowSit 


%animation = true ; % if animation == true

i = 1;
generate = true;

% checking for needed motion file in motions folder
% In case file is not avilable, it will generate the file 
% Now you can go and grab a cup of coffee ;)
% It may take upto 8-10 minutes if your file is not there
% To get more details go to motions\read.m

if isfile("motions\"+ Motion + "_q.mat")==true
    load(Motion + "_q.mat");
else
    run('read.m');
    pause(5);
end
% loading P.mat from motions folder
load('P.mat');

% generates data at instance i
% To get more details go to Hanavan_model\hanavan.m
% generate animation
% To get more details go to animation\anim.m
if animation==true
    Hanavan = hanavan(P,M,i,motion);
    run('anim.m');
end
