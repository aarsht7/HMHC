function model = hanavan(P,M,i,motion)
%
% Install robotic toolbox from here:
% https://petercorke.com/toolboxes/robotics-toolbox/
%
% Note that lengths in hanavan parameters are in meters
% While lengths used to calculate mass from model are in centimeters
% And parker data are in milimeters
% To use this function you can create the variable whih is struct
% model = hanavan(P,M,i,motion)
%
%***************** INPUT DATA *********************************
% P = Hanavan parameters
% M = mass of body (Fz/g)
% i = instance at which data required
% motion = motion data for which this data are extracting
% To get motion you need to run motion data file "slowArm_q.mat"
% which will give you structure witn name "motion" and to get values for
% 1st instance you take i=1
%
%**************** KNOWN ISSUE ********************************
% Apperently I noticed that the RIGHT side data are for LEFT side 
% and vice-versa. But you can use it by defining as shown.
% I am really sorry for this mistake but there has to be lot of change
% to modify it. Please use it accordingly. Thank you.
%
%************** Read_Me if you want to know the code***********************
%
% Some other things you might be intrested to know before using the code
%
% for ellipsoid body (Head,Hand)
% You have avilable properties : a,b,c,mass,volume,CoM,orient,I,r_in,r_out
% where a,b,c are rasious of ellipsoid in x,y,z direction (in  milimeters)
% mass is the mass of the segment comming from hanavan model (in Kg)
% volume is the volume of ellipsoid (in m^3)
% CoM is the position of CoM for specific body in Global frame (in milimeter)
% orient is the orientation of body in global frame (in Degrees)
% I is inertia matrix for a specific body at its CoM (in Kg.m^2)
% r_in is the measurement matrix from all input force point(markers) to the CoM
% r_out is measurement matrix from all output force point to the CoM
% To get more details on Ellipsoid shape, go to Hanavan_model\Ellipsoid.m
%
% for elliptical Solid or ES (all other parts)
% You have avilable properties : a0,b0,a1,b1,L,mass,volume,CoM,orient,I,r_in,r_out
% a0,b0 are radious of upper ellips in ES (in  milimeters)
% a1,b1 are radious of lower ellips in ES (in  milimeters)
% L is lenght of ES (in  milimeters)
% mass is the mass of the segment comming from hanavan model (in Kg)
% volume is the volume of ellipsoid (in m^3)
% CoM is the position of CoM for specific body in Global frame (in milimeter)
% orient is the orientation of body in global frame (in Degrees)
% (There is no orientation between Upper-trunk and middle-trunk)
% I is inertia matrix for a specific body at its CoM (in Kg.m^2)
% r_in is the measurement matrix from all input force point(markers) to the CoM (in milimeter)
% r_out is measurement matrix from all output force point to the CoM (in milimeter)
% To get more details on Elliptical shape, go to Hanavan_model\ES.m
%
%***************** USAGE **************************************
%
% L_Hand = model.R_Hand; 
% R_Hand = model.L_Hand; 
% L_Forearm = model.R_Forearm;
% R_Forearm = model.L_Forearm;
% L_Upperarm = model.R_Upperarm;
% R_Upperarm = model.L_Upperarm;
% L_Foot = model.R_Foot;
% R_Foot = model.L_Foot;
% L_Shank = model.R_Shank;
% R_Shank = model.L_Shank;
% L_Thigh = model.R_Thigh;
% R_Thigh = model.L_Thigh;
% Head = model.Head;
% upperTrunk = model.upperTrunk;
% middleTrunk = model.middleTrunk
% lowerTrunk = model.lowerTrunk;
% umTrunk = model.umTrunk; (combined data on upper and middle trunk)
%
% 
% To get specific property
% You can use either 
% body_part_property = model.body_part.property_name  (Hand_mass = model.Hand.mass)
% OR
% body_part = model.body_part;
% (Hand = model.Hand;)
% body_part_property = body_part.property;
% (Hand_mass = Hand.mass;)
% 
%
%************************************* Warning *************************
% 
% CHECK UNITS OF DATA IN Read_Me SECTION BEFORE USING IT
%
% written by Aarsh on November 25,2020
% modified on December 4,2020
% modified on December 8,2020
% IN CASE YOU FIND ANY MISTAKE OR YOU HAVE DOUBT PLEASE PING ME
%  



    % Hand modelling
    
    R_Hand = body;
    R_Hand.name = 'R_Hand';
    R_Hand.a = 1000*P(14)/(2*pi);
    R_Hand.b = R_Hand.a;
    R_Hand.c = 1000*P(2)/2;
    R_Hand.mass = 0.038*P(15)*100 + 0.08*P(32)*100 - 0.66;
    R_Hand.type = Ellipsoid(P(14)/(2*pi),P(14)/(2*pi),P(2)/2,R_Hand.mass);
    R_Hand.volume = R_Hand.type.get_volume();
    R_Hand.CoM = get_com(R_Hand.type,motion(11).J(1:3,i));
    R_Hand.I = R_Hand.type.get_inertiaMat();
    R_Hand.r_in = [0;0;0];
    R_Hand.r_out = motion(9).J(1:3,i) - R_Hand.CoM;
    R_Hand.orient = motion(9).J(4:6,i) - motion(11).J(4:6,i);
    model.R_Hand = R_Hand;
    
    L_Hand = body;
    L_Hand.name = 'L_Hand';
    L_Hand.a = 1000*P(14)/(2*pi);
    L_Hand.b = L_Hand.a;
    L_Hand.c = 1000*P(2)/2;
    L_Hand.mass = 0.038*P(15)*100 + 0.08*P(32)*100 - 0.66;
    L_Hand.type = Ellipsoid(P(14)/(2*pi),P(14)/(2*pi),P(2)/2,L_Hand.mass);
    L_Hand.volume = L_Hand.type.get_volume();
    L_Hand.CoM = L_Hand.type.get_com(motion(12).J(1:3,i));
    L_Hand.I = L_Hand.type.get_inertiaMat();
    L_Hand.r_in = [0;0;0];
    L_Hand.r_out = motion(10).J(1:3,i) - L_Hand.CoM;
    L_Hand.orient = motion(12).J(4:6,i) - motion(10).J(4:6,i);
    model.L_Hand = L_Hand;
    
    
    % Forearm modelling
    
    R_Forearm = body;
    R_Forearm.name = 'R_Forearm';
    R_Forearm.a0 = 1000*P(17)/(2*pi);
    R_Forearm.b0 = 1000*P(17)/(2*pi);
    R_Forearm.a1 = 1000*P(15)/(2*pi);
    R_Forearm.b1 = 1000*P(15)/(2*pi);
    R_Forearm.L = 1000*P(3);
    R_Forearm.mass = 0.081*M + 0.052*P(16)*100 - 1.650 ;
    R_Forearm.type = ES(P(17)/(2*pi),P(17)/(2*pi),P(15)/(2*pi),P(15)/(2*pi),P(3),R_Forearm.mass);
    R_Forearm.volume = R_Forearm.type.get_volume();
    R_Forearm.CoM = R_Forearm.type.get_com(motion(7).J(1:3,i),motion(9).J(1:3,i));
    R_Forearm.I = R_Forearm.type.get_inertiaMat();
    R_Forearm.r_in = motion(9).J(1:3,i) - R_Forearm.CoM;
    R_Forearm.r_out = motion(7).J(1:3,i) - R_Forearm.CoM;
    R_Forearm.orient = motion(7).J(4:6,i) - motion(9).J(4:6,i);
    model.R_Forearm = R_Forearm;
    
    L_Forearm = body;
    L_Forearm.name = 'L_Forearm';
    L_Forearm.a0 = 1000*P(17)/(2*pi);
    L_Forearm.b0 = 1000*P(17)/(2*pi);
    L_Forearm.a1 = 1000*P(15)/(2*pi);
    L_Forearm.b1 = 1000*P(15)/(2*pi);
    L_Forearm.L = 1000*P(3);
    L_Forearm.mass = 0.081*M + 0.052*P(16)*100 - 1.650 ;
    L_Forearm.type = ES(P(17)/(2*pi),P(17)/(2*pi),P(15)/(2*pi),P(15)/(2*pi),P(3),L_Forearm.mass);
    L_Forearm.volume = L_Forearm.type.get_volume();
    L_Forearm.CoM = L_Forearm.type.get_com(motion(10).J(1:3,i),motion(8).J(1:3,i));
    L_Forearm.I = L_Forearm.type.get_inertiaMat();
    L_Forearm.r_in = motion(10).J(1:3,i) - L_Forearm.CoM;
    L_Forearm.r_out = motion(8).J(1:3,i) - L_Forearm.CoM;
    L_Forearm.orient = motion(8).J(4:6,i) - motion(10).J(4:6,i);
    model.L_Forearm = L_Forearm;
    
    % Upperarm modelling
    
    R_Upperarm = body;
    R_Upperarm.name = 'R_Upperarm';
    R_Upperarm.a0 = 1000*(P(18)/(2*pi));
    R_Upperarm.b0 = 1000*(P(18)/(2*pi));
    R_Upperarm.a1 = 1000*(P(17)/(2*pi));
    R_Upperarm.b1 = 1000*(P(17)/(2*pi));
    R_Upperarm.L = 1000*P(5);
    R_Upperarm.mass = 0.007*M + 0.092*P(18)*100 +0.050*P(5)*100 -3.101;
    R_Upperarm.type = ES(P(18)/(2*pi),P(18)/(2*pi),P(17)/(2*pi),P(17)/(2*pi),P(5),R_Upperarm.mass);
    R_Upperarm.volume = R_Upperarm.type.get_volume();
    R_Upperarm.CoM = R_Upperarm.type.get_com(motion(5).J(1:3,i),motion(7).J(1:3,i));
    R_Upperarm.I = R_Upperarm.type.get_inertiaMat();
    R_Upperarm.r_in = motion(7).J(1:3,i) - R_Upperarm.CoM;
    R_Upperarm.r_out = motion(5).J(1:3,i) - R_Upperarm.CoM;
    R_Upperarm.orient = motion(5).J(4:6,i) - motion(7).J(4:6,i);
    model.R_Upperarm = R_Upperarm;
    
    L_Upperarm = body;
    L_Upperarm.name = 'L_Upperarm';
    L_Upperarm.a0 = 1000*(P(18)/(2*pi));
    L_Upperarm.b0 = 1000*(P(18)/(2*pi));
    L_Upperarm.a1 = 1000*(P(17)/(2*pi));
    L_Upperarm.b1 = 1000*(P(17)/(2*pi));
    L_Upperarm.L = 1000*P(5);
    L_Upperarm.mass = 0.007*M + 0.092*P(18)*100 +0.050*P(5)*100 -3.101;
    L_Upperarm.type = ES(P(18)/(2*pi),P(18)/(2*pi),P(17)/(2*pi),P(17)/(2*pi),P(5),L_Upperarm.mass);
    L_Upperarm.volume = L_Upperarm.type.get_volume();
    L_Upperarm.CoM = L_Upperarm.type.get_com(motion(8).J(1:3,i),motion(6).J(1:3,i));
    L_Upperarm.I = L_Upperarm.type.get_inertiaMat();
    L_Upperarm.r_in = motion(8).J(1:3,i) - L_Upperarm.CoM;
    L_Upperarm.r_out = motion(6).J(1:3,i) - L_Upperarm.CoM;
    L_Upperarm.orient = motion(6).J(4:6,i) - motion(8).J(4:6,i);
    model.L_Upperarm = L_Upperarm;
    
    % Foot modelling
    
    R_Foot = body;
    R_Foot.name = 'R_Foot';
    R_Foot.a0 = 1000*P(19)/(2*pi);
    R_Foot.b0 = 1000*P(19)/(2*pi);
    R_Foot.a1 = 1000*(P(33)+P(34))/4;
    R_Foot.b1 = 1000*(P(20)+P(21))/(2*pi);
    R_Foot.L = 1000*P(6);
    R_Foot.mass = 0.003*M + 0.048*P(22)*100 +0.027*P(6)*100 -0.869;
    R_Foot.type = ES(P(19)/(2*pi),P(19)/(2*pi),(P(33)+P(34))/4,(P(20)+P(21))/(2*pi),P(6),R_Foot.mass);
    R_Foot.volume = R_Foot.type.get_volume();
    R_Foot.CoM = R_Foot.type.get_com(motion(17).J(1:3,i),motion(19).J(1:3,i));
    R_Foot.I = R_Foot.type.get_inertiaMat();
    R_Foot.r_in = motion(19).J(1:3,i) - R_Foot.CoM;
    R_Foot.r_out = motion(17).J(1:3,i) - R_Foot.CoM;
    R_Foot.orient = motion(17).J(4:6,i) - motion(19).J(4:6,i);
    model.R_Foot = R_Foot;
    
    L_Foot = body;
    L_Foot.name = 'L_Foot';
    L_Foot.a0 = 1000*P(19)/(2*pi);
    L_Foot.b0 = 1000*P(19)/(2*pi);
    L_Foot.a1 = 1000*(P(33)+P(34))/4;
    L_Foot.b1 = 1000*(P(20)+P(21))/(2*pi);
    L_Foot.L = 1000*P(6);
    L_Foot.mass = 0.003*M + 0.048*P(22)*100 +0.027*P(6)*100 -0.869;
    L_Foot.type = ES(P(19)/(2*pi),P(19)/(2*pi),(P(33)+P(34))/4,(P(20)+P(21))/(2*pi),P(6),L_Foot.mass);
    L_Foot.volume = L_Foot.type.get_volume();
    L_Foot.CoM = L_Foot.type.get_com(motion(18).J(1:3,i),motion(20).J(1:3,i));
    L_Foot.I = L_Foot.type.get_inertiaMat();
    L_Foot.r_in = motion(20).J(1:3,i) - L_Foot.CoM;
    L_Foot.r_out = motion(18).J(1:3,i) - L_Foot.CoM;
    L_Foot.orient = motion(18).J(4:6,i) - motion(20).J(4:6,i);
    model.L_Foot = L_Foot;
    
    % Shank modelling
    
    R_Shank = body;
    R_Shank.name = 'R_Shank';
    R_Shank.a0 = 1000*P(24)/(2*pi);
    R_Shank.b0 = 1000*P(24)/(2*pi);
    R_Shank.a1 = 1000*P(22)/(2*pi);
    R_Shank.b1 = 1000*P(22)/(2*pi);
    R_Shank.L = 1000*P(7);
    R_Shank.mass = 0.135*P(23)*100 -1.318;
    R_Shank.type = ES(P(24)/(2*pi),P(24)/(2*pi),P(22)/(2*pi),P(22)/(2*pi),P(7),R_Shank.mass);
    R_Shank.volume = R_Shank.type.get_volume();
    R_Shank.CoM = R_Shank.type.get_com(motion(15).J(1:3,i),motion(17).J(1:3,i));
    R_Shank.I = R_Shank.type.get_inertiaMat();
    R_Shank.r_in = motion(15).J(1:3,i) - R_Shank.CoM;
    R_Shank.r_out = motion(17).J(1:3,i) - R_Shank.CoM;
    R_Shank.orient = motion(15).J(4:6,i) - motion(17).J(4:6,i);
    model.R_Shank = R_Shank;
    
    L_Shank = body;
    L_Shank.name = 'L_Shank';
    L_Shank.a0 = 1000*P(24)/(2*pi);
    L_Shank.b0 = 1000*P(24)/(2*pi);
    L_Shank.a1 = 1000*P(22)/(2*pi);
    L_Shank.b1 = 1000*P(22)/(2*pi);
    L_Shank.L = 1000*P(7);
    L_Shank.mass = 0.135*P(23)*100 -1.318;
    L_Shank.type = ES(P(24)/(2*pi),P(24)/(2*pi),P(22)/(2*pi),P(22)/(2*pi),P(7),L_Shank.mass);
    L_Shank.volume = L_Shank.type.get_volume();
    L_Shank.CoM = L_Shank.type.get_com(motion(16).J(1:3,i),motion(18).J(1:3,i));
    L_Shank.I = L_Shank.type.get_inertiaMat();
    L_Shank.r_in = motion(16).J(1:3,i) - L_Shank.CoM;
    L_Shank.r_out = motion(18).J(1:3,i) - L_Shank.CoM;
    L_Shank.orient = motion(16).J(4:6,i) - motion(18).J(4:6,i);
    model.L_Shank = L_Shank;
    
    % Thigh modelling
    
    R_Thigh = body;
    R_Thigh.name = 'R_Thigh';
    R_Thigh.a0 = 1000*(P(25)/pi) - 1000*(P(35)/2);
    R_Thigh.b0 = 1000*P(35)/2;
    R_Thigh.a1 = 1000*P(24)/(2*pi);
    R_Thigh.b1 = 1000*P(24)/(2*pi);
    R_Thigh.L = 1000*P(8);
    R_Thigh.mass = 0.074*M + 0.138*P(25)*100 -4.641;
    R_Thigh.type = ES((P(25)/pi - P(35))/2,P(35)/2,P(24)/(2*pi),P(24)/(2*pi),P(8),R_Thigh.mass);
    R_Thigh.volume = R_Thigh.type.get_volume();
    R_Thigh.CoM = R_Thigh.type.get_com(motion(13).J(1:3,i),motion(15).J(1:3,i));
    R_Thigh.I = R_Thigh.type.get_inertiaMat();
    R_Thigh.r_in = motion(13).J(1:3,i) - R_Thigh.CoM;
    R_Thigh.r_out = motion(15).J(1:3,i) - R_Thigh.CoM;
    R_Thigh.orient = motion(13).J(4:6,i) - motion(15).J(4:6,i);
    model.R_Thigh = R_Thigh;
    
    L_Thigh = body;
    L_Thigh.name = 'L_Thigh';
    L_Thigh.a0 = 1000*(P(25)/pi) - 1000*(P(35)/2);
    L_Thigh.b0 = 1000*P(35)/2;
    L_Thigh.a1 = 1000*P(24)/(2*pi);
    L_Thigh.b1 = 1000*P(24)/(2*pi);
    L_Thigh.L = 1000*P(8);
    L_Thigh.mass = 0.074*M + 0.138*P(25)*100 -4.641;
    L_Thigh.type = ES((P(25)/pi - P(35))/2,P(35)/2,P(24)/(2*pi),P(24)/(2*pi),P(8),L_Thigh.mass);
    L_Thigh.volume = L_Thigh.type.get_volume();
    L_Thigh.CoM = L_Thigh.type.get_com(motion(14).J(1:3,i),motion(16).J(1:3,i));
    L_Thigh.I = L_Thigh.type.get_inertiaMat();
    L_Thigh.r_in = motion(14).J(1:3,i) - L_Thigh.CoM;
    L_Thigh.r_out = motion(16).J(1:3,i) - L_Thigh.CoM;
    L_Thigh.orient = motion(14).J(4:6,i) - motion(16).J(4:6,i);
    model.L_Thigh = L_Thigh;
    
    % Head modelling
    
    Head = body;
    Head.name = 'Head';
    Head.a = 1000*P(26)/(2*pi);
    Head.b = 1000*P(26)/(2*pi);
    Head.c = 1000*P(9)/2;
    Head.mass = 0.104*P(26)*100 + 0.015*M -2.189;
    Head.type = Ellipsoid(P(26)/(2*pi),P(26)/(2*pi),P(9)/2,Head.mass);
    Head.volume = Head.type.get_volume();
    Head.CoM = Head.type.get_com(motion(4).J(1:3,i));
    Head.I = Head.type.get_inertiaMat();
    Head.r_in = [0;0;0];
    Head.r_out = motion(3).J(1:3,i) - Head.CoM;
    Head.orient = motion(3).J(4:6,i) - motion(4).J(4:6,i);
    model.Head = Head;
    
    % Trunk modelling
    
    upperTrunk = body;
    middleTrunk = body;
    lowerTrunk = body;
    
    upperTrunk.name = 'upperTrunk';
    upperTrunk.a0 = 1000*(P(36)+P(37))/4;
    upperTrunk.b0 = 1000*(P(27)+P(28))/(2*pi) - upperTrunk.a0;
    upperTrunk.a1 = 1000*(P(36)+P(37))/4;
    upperTrunk.b1 = 1000*(P(27)+P(28))/(2*pi) - upperTrunk.a0;
    upperTrunk.L = 1000*P(11);
    upperTrunk.type = ES((P(36)+P(37))/4,(P(27)+P(28))/2*pi - (P(36)+P(37))/4,(P(36)+P(37))/4,(P(27)+P(28))/2*pi - (P(36)+P(37))/4, P(11),NaN);
    upperTrunk.volume = upperTrunk.type.get_volume();
    
    
    middleTrunk.name = 'middleTrunk';
    middleTrunk.a0 = 1000*P(37)/2;
    middleTrunk.b0 = 1000*P(28)/pi - 1000*P(37)/2;
    middleTrunk.a1 = 1000*P(38)/2;
    middleTrunk.b1 = 1000*P(29)/pi - 1000*P(38)/2;
    middleTrunk.L = 1000*P(12);
    middleTrunk.type = ES(P(37)/2,P(28)/pi - P(37)/2,P(38)/2,P(29)/pi - P(38)/2, P(12),NaN);
    middleTrunk.volume = middleTrunk.type.get_volume();
    
    
    lowerTrunk.name = 'lowerTrunk';
    lowerTrunk.a0 = 1000*(P(38)+P(39))/4;
    lowerTrunk.b0 = ((P(29)+P(30))/(2*pi))*1000 - lowerTrunk.a0;
    lowerTrunk.a1 = 1000*(P(38)+P(39))/4;
    lowerTrunk.b1 = ((P(29)+P(30))/(2*pi))*1000 - lowerTrunk.a0;
    lowerTrunk.L = 1000*P(13);
    lowerTrunk.type = ES((P(38)+P(39))/4,(P(29)+P(30))/2*pi - (P(38)+P(39))/4,(P(38)+P(39))/4,(P(29)+P(30))/2*pi - (P(38)+P(39))/4, P(13),NaN);
    lowerTrunk.volume = lowerTrunk.type.get_volume();
    
    m_wt = 0.349*M + 0.423*P(41)*100 + 0.229*P(27)*100 - 35.460;
    sf = m_wt/(0.92*upperTrunk.volume+1.01*(middleTrunk.volume+lowerTrunk.volume));
    
    upperTrunk.mass = 0.92*sf*upperTrunk.volume;
    middleTrunk.mass = 1.01*sf*middleTrunk.volume;
    lowerTrunk.mass = 1.01*sf*lowerTrunk.volume;
    
    upperTrunk.type.mass = upperTrunk.mass;
    middleTrunk.type.mass = middleTrunk.mass;
    lowerTrunk.type.mass = lowerTrunk.mass;
    
    upperTrunk.CoM = upperTrunk.type.get_com(motion(3).J(1:3,i),motion(2).J(1:3,i));
    upperTrunk.I = upperTrunk.type.get_inertiaMat();
    model.upperTrunk = upperTrunk;
    
    middleTrunk.CoM = middleTrunk.type.get_com(motion(2).J(1:3,i),motion(3).J(1:3,i));
    middleTrunk.I = middleTrunk.type.get_inertiaMat();
    model.middleTrunk = middleTrunk;
    
    lowerTrunk.CoM = lowerTrunk.type.get_com(motion(2).J(1:3,i),motion(1).J(1:3,i));
    lowerTrunk.I = lowerTrunk.type.get_inertiaMat();
    lowerTrunk.r_in = motion(2).J(1:3,i) - lowerTrunk.CoM;
    % in order 13 - 14
    r_out1 = motion(13).J(1:3,i) - lowerTrunk.CoM;
    r_out2 = motion(14).J(1:3,i) - lowerTrunk.CoM;
    lowerTrunk.r_out = [r_out1,r_out2];
    lowerTrunk.orient = motion(1).J(4:6,i) - motion(2).J(4:6,i);
    model.lowerTrunk = lowerTrunk;   
    
    % Combining Uppertrunk and Middletrunk
    umTrunk = body;
    umTrunk.name = 'umTrunk';
    umTrunk.mass = upperTrunk.mass + middleTrunk.mass;
    umTrunk.volume = upperTrunk.volume + middleTrunk.volume;
    % newCoM = (CoM1*m1 + CoM2*m2)/(m1+m2);
    umTrunk.CoM = (upperTrunk.CoM*upperTrunk.mass + middleTrunk.CoM*middleTrunk.mass)/(upperTrunk.mass + middleTrunk.mass);
    % Transfer of axis theorm in I_new = I_old + m*d^2 ; d=dist btn axis
    % but in 3D
    % I_new = I_old + m * skew(d); d=distance vector btn CoM
    differance1 = (upperTrunk.CoM-umTrunk.CoM);
    differance2 = (middleTrunk.CoM-umTrunk.CoM);
    skew1 = [0 -differance1(3) differance1(2);...
            differance1(3) 0 -differance1(1);...
            -differance1(2) differance1(1) 0];
    skew2 = [0 -differance2(3) differance2(2);...
            differance2(3) 0 -differance2(1);...
            -differance2(2) differance2(1) 0];
    um_I1 =  upperTrunk.I - 1000*upperTrunk.mass*skew1;
    um_I2 =  middleTrunk.I - 1000*middleTrunk.mass*skew2;
    umTrunk.I = um_I1+um_I2;
    % In order  marker 3 - 5 - 6 
    r_in1 = motion(3).J(1:3,i) - umTrunk.CoM;
    r_in2 = motion(5).J(1:3,i) - umTrunk.CoM;
    r_in3 = motion(6).J(1:3,i) - umTrunk.CoM;
    umTrunk.r_in = [r_in1,r_in2,r_in3];
    umTrunk.r_out = motion(2).J(1:3,i) - umTrunk.CoM;
    umTrunk.orient = motion(2).J(4:6,i) - motion(3).J(4:6,i);
    model.umTrunk = umTrunk;
                        
end

