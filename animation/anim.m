% written by Aarsh on December 4,2020
% modified on December 8,2020
% IN CASE YOU FIND ANY MISTAKE OR YOU HAVE DOUBT PLEASE PING ME
%
%*************************** USAGE *****************************
% To be used in animation 
% Nothing much to explain

% defining the hanavan model for animation
% no need of mass here
global Hanavan freq
animHand = Hanavan.R_Hand; 
animForearm = Hanavan.R_Forearm;
animUpperarm = Hanavan.R_Upperarm;
animFoot = Hanavan.R_Foot;
animShank = Hanavan.R_Shank;
animThigh = Hanavan.R_Thigh;
animHead = Hanavan.Head;
animupperTrunk = Hanavan.upperTrunk;
animmiddleTrunk = Hanavan.middleTrunk;
animlowerTrunk = Hanavan.lowerTrunk;
 
figure(1);
axis equal
xlim([-1000,1000]);
ylim([-1000,1000]);
zlim([0,2000]);
v = [152.0058 21.6421];
view(v);

% since there is lots of data, here taking interval of 20
for i = 1:freq:length(motion(1).J)
    hold on
    
    % For details of how this class works go to animation\build.m
    uppertrunk = build(animupperTrunk.a0,animupperTrunk.b0,animupperTrunk.a1,animupperTrunk.b1,[motion(3).J(1:3,i)],[motion(2).J(1:3,i)],animupperTrunk.L,NaN,NaN,NaN);
    elliptical(uppertrunk);
    
    middletrunk = build(animmiddleTrunk.a0,animmiddleTrunk.b0,animmiddleTrunk.a1,animmiddleTrunk.b1,[motion(2).J(1:3,i)],[motion(3).J(1:3,i)],animmiddleTrunk.L,NaN,NaN,NaN);
    elliptical(middletrunk);
    
    lowertrunk = build(animlowerTrunk.a0,animlowerTrunk.b0,animlowerTrunk.a1,animlowerTrunk.b1,[motion(2).J(1:3,i)],[motion(1).J(1:3,i)],animlowerTrunk.L,NaN,NaN,NaN);
    elliptical(lowertrunk);
    
    r_thigh = build(animThigh.a0,animThigh.b0,animThigh.a1,animThigh.b1,[motion(13).J(1:3,i)],[motion(15).J(1:3,i)],animThigh.L,NaN,NaN,NaN);
    elliptical(r_thigh);
    
    l_thigh = build(animThigh.a0,animThigh.b0,animThigh.a1,animThigh.b1,[motion(14).J(1:3,i)],[motion(16).J(1:3,i)],animThigh.L,NaN,NaN,NaN);
    elliptical(l_thigh);
    
    r_shank = build(animShank.a0,animShank.b0,animShank.a1,animShank.b1,[motion(15).J(1:3,i)],[motion(17).J(1:3,i)],animShank.L,NaN,NaN,NaN);
    elliptical(r_shank);
    
    l_shank = build(animShank.a0,animShank.b0,animShank.a1,animShank.b1,[motion(16).J(1:3,i)],[motion(18).J(1:3,i)],animShank.L,NaN,NaN,NaN);
    elliptical(l_shank);
    
    r_foot = build(animFoot.a0,animFoot.b0,animFoot.a1,animFoot.b1,[motion(17).J(1:3,i)],[motion(19).J(1:3,i)],animFoot.L,NaN,NaN,NaN);
    elliptical(r_foot);
    
    l_foot = build(animFoot.a0,animFoot.b0,animFoot.a1,animFoot.b1,[motion(18).J(1:3,i)],[motion(20).J(1:3,i)],animFoot.L,NaN,NaN,NaN);
    elliptical(l_foot);
    
    r_upperarm = build(animUpperarm.a0,animUpperarm.b0,animUpperarm.a1,animUpperarm.b1,[motion(5).J(1:3,i)],[motion(7).J(1:3,i)],animUpperarm.L,NaN,NaN,NaN);
    elliptical(r_upperarm);
    
    l_upperarm = build(animUpperarm.a0,animUpperarm.b0,animUpperarm.a1,animUpperarm.b1,[motion(6).J(1:3,i)],[motion(8).J(1:3,i)],animUpperarm.L,NaN,NaN,NaN);
    elliptical(l_upperarm);
    
    r_forearm = build(animForearm.a0,animForearm.b0,animForearm.a1,animForearm.b1,[motion(7).J(1:3,i)],[motion(9).J(1:3,i)],animForearm.L,NaN,NaN,NaN);
    elliptical(r_forearm);
    
    l_forearm = build(animForearm.a0,animForearm.b0,animForearm.a1,animForearm.b1,[motion(8).J(1:3,i)],[motion(10).J(1:3,i)],animForearm.L,NaN,NaN,NaN);
    elliptical(l_forearm);
    
    head = build(NaN,NaN,NaN,NaN,motion(4).J(1:6,i),motion(3).J(1:6,i),NaN,animHead.a,animHead.b,animHead.c);
    ellipsoid_(head);

    R_hand = build(NaN,NaN,NaN,NaN,motion(11).J(1:6,i),motion(9).J(1:6,i),NaN,animHand.a,animHand.b,animHand.c);
    ellipsoid_(R_hand);
    
    L_hand = build(NaN,NaN,NaN,NaN,motion(12).J(1:6,i),motion(10).J(1:6,i),NaN,animHand.a,animHand.b,animHand.c);
    ellipsoid_(L_hand);
    
    drawnow
    cla

end



