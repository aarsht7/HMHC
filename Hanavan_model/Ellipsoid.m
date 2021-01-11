
% written by Aarsh on November 25,2020
% modified on December 4,2020
% modified on December 8,2020
% IN CASE YOU FIND ANY MISTAKE OR YOU HAVE DOUBT PLEASE PING ME
%
%********************** USAGE *********************************
% object_name = Ellipsoid(a,b,c,mass)
% where a,b and c are radii along 3 axis
% mass is comming from hanavan parameters
% volume_object = object_name.get_volume()
% CoM_object = object_name.get_com()
% Ixx_object = object_name.get_Ixx()
% Iyy_object = object_name.get_Iyy()
% Izz_object = object_name.get_Izz()
% Ineria_matrix_object = object_name.get_inertiaMat()
%
%**************************************************************************

classdef Ellipsoid
    
    properties
        a
        b
        c
        mass
    end

    methods
        % class constructor
        function this = Ellipsoid(A,B,C,MASS)

            this.a = A;
            this.b = B;
            this.c = C;
            this.mass = MASS;

        end
        
        % Calculate the volume of ellipsoid
        function volume = get_volume(this)
           volume = 4*this.a*this.b*this.c*pi/3; 
        end
        
        % Calculate the position of CoM of Ellipsoid in Global frame
        function CoM= get_com(this,marker)
           CoM = marker; 
        end
        
        % Inertia of body in X directin in local frame at CoM
        function Ixx = get_Ixx(this)
            Ixx = 0.2*this.mass*(this.b^2+this.c^2);
        end
        
        % Inertia of body in Y directin in local frame at CoM
        function Iyy= get_Iyy(this)
            Iyy = 0.2*this.mass*(this.a^2+this.c^2);
        end
        
        % Inertia of body in Z directin in local frame at CoM
        function Izz= get_Izz(this)
            Izz = 0.2*this.mass*(this.a^2+this.b^2);
        end
        
        % Inertia matrix of body in local frame at CoM
        function I= get_inertiaMat(this)
            I = [get_Ixx(this),0,0; 0,get_Iyy(this),0; 0,0,get_Izz(this)];
        end
            
    end
end


       
