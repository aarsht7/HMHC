% written by Aarsh on December 4,2020
% modified on December 8,2020
% IN CASE YOU FIND ANY MISTAKE OR YOU HAVE DOUBT PLEASE PING ME
%
%*************************** USAGE *****************************
% Class to buils and plot Ellipsoid and Elliptical shape(ES)
%
% Class constructor 
% obj = build(a0,b0,a1,b1,marker_1,marker_2,length,a,b,c)
%
% a0 = radious for 1st ellips in X
% b0 = radious for 1st ellips in Y
% a1 = radious for 2nd ellips in X
% b1 = radious for 2nd ellips in Y
% marker_1 = Starting point of ES or center of ellipsoid
% market_2 = direction of ES (marker from next body)
% length = Length of ES
% a = radious for 1st ellipsoid in X
% b = radious for 1st ellipsoid in Y
% c = radious for 1st ellipsoid in Z
% 
% If body is ES 
% a = b = c = NaN;
% and to plot the body :
% elliptical(obj);
%
% If body is Ellipsoid 
% a0 = b0 = a1 = b1 =length = marker_2 = NaN 
% and to plot the body :
% ellipsoid_(obj);
%
%**************************************************************************

classdef build
    
    properties
        a_0     % = NaN for ellipsoid
        b_0     % = NaN for ellipsoid
        a_1     % = NaN for ellipsoid
        b_1     % = NaN for ellipsoid
        marker1 %
        marker2 % = NaN for ellipsoid
        L       % 
        a_      % = NaN for elliptical
        b_      % = NaN for elliptical
        c_      % = NaN for elliptical

    end
    properties(Constant)
        N_b = 20 % constant number of data points 
    end
    
    methods
        
    function obj = build(a0,b0,a1,b1,marker_1,marker_2,length,a,b,c)
        obj.a_0 = a0;   
        obj.b_0 = b0;
        obj.a_1 = a1;
        obj.b_1 = b1;
        obj.marker1 = marker_1;
        obj.marker2 = marker_2;
        obj.L = length;
        obj.a_ = a;
        obj.b_ = b;
        obj.c_ = c;

    end

    function s = elliptical(obj)
        a0 = obj.a_0;           % radious for 1st ellips in X
        b0 = obj.b_0;           % radious for 1st ellips in Y
        a1 = obj.a_1;           % radious for 2nd ellips in X
        b1 = obj.b_1;           % radious for 2nd ellips in Y
        marker_1 = obj.marker1; % Starting point of ES 
        marker_2 = obj.marker2; % Direction of ES (from one market to other)
        length = obj.L;         % Length of ES
        Nb = obj.N_b;           % number of data points defined on top
        
        if norm(marker_2-marker_1)<length
            length = norm(marker_2-marker_1);
        end
        
        % Elevation and Azimuth angle from marker_1 to marker_2
        % to define the relative direction
        El = atan2((marker_2(3)-marker_1(3)),norm([marker_2(1)-marker_1(1),marker_2(2)-marker_1(2)]));
        Az = atan2(marker_2(2)-marker_1(2),marker_2(1)-marker_1(1));

        % linespace from 0 to 2*pi with Nb data points
        the=linspace(0,2*pi,Nb);

        % X coordinates of ellips 1 and 2 respectively 
        X1=a0*cos(the);
        X2=a1*cos(the);
        
        % Y coordinates of ellips 1 and 2 respectively
        Y1=b0*sin(the);
        Y2=b1*sin(the);

        % Z coordinates assumed to be zero in starting for both
        Z1=zeros(1, Nb);
        Z2=zeros(1, Nb);

        % Rotaion matrix based on Elevation and Azimuth
        % Google it if you have doubt
        R = [sin(El)*cos(Az) , -sin(Az) , cos(El)*cos(Az) ;...
            sin(El)*sin(Az) ,  cos(Az) , cos(El)*sin(Az) ;...
             -cos(El)        ,     0    , sin(El)];


% Apply transformation
            for i=1:Nb
                
                xyzMat_1 = [X1(i);Y1(i);Z1(i)]; % create temp values
                temp_1 = R*xyzMat_1; 
                X1(i) = temp_1(1); 
                Y1(i) = temp_1(2);
                Z1(i) = temp_1(3);
    
                xyzMat_2 = [X2(i);Y2(i);Z2(i)]; % create temp values
                temp_2 = R*xyzMat_2; 
                X2(i) = temp_2(1); 
                Y2(i) = temp_2(2);
                Z2(i) = temp_2(3);
    
            end
% Apply offsets 
            % 1st ellips has center position of marker_1
            X1 = X1 + marker_1(1);
            Y1 = Y1 + marker_1(2);
            Z1 = Z1 + marker_1(3);
            
            % unit vector in direction of marker_2 from marker_1
            u = (marker_2-marker_1)/norm(marker_2-marker_1);
            % location of 2nd ellips in direction of u and 
            % with given length of respective part
            d = marker_1 + u*length;
            X2 = X2 + d(1);
            Y2 = Y2 + d(2);
            Z2 = Z2 + d(3);
            
            X = [X1;X2];
            Y = [Y1;Y2];
            Z = [Z1;Z2];
            
            % plotting the mesh from accuired data
            s = mesh(X,Y,Z,'FaceAlpha','0.5');colormap hsv;
            s.FaceColor = 'flat';

    end
    
    
    function s = ellipsoid_(obj)
        a = obj.a_;
        b = obj.b_;
        c = obj.c_;
        marker_1 = obj.marker1(1:3);

        [X,Y,Z] = ellipsoid(marker_1(1),marker_1(2),marker_1(3),a,b,c);
        s = mesh(X,Y,Z,'FaceAlpha','0.5');colormap hsv;
        s.FaceColor = 'flat';

    end
    end
end
