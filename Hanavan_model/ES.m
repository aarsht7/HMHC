
% written by Aarsh on November 25,2020
% modified on December 4,2020
% modified on December 8,2020
% IN CASE YOU FIND ANY MISTAKE OR YOU HAVE DOUBT PLEASE PING ME
%
%********************** USAGE *********************************
% object_name = ES(a0,b0,a1,b1,l,mass)
% where a0 and b0 are base radii, a1 and b1 are top radii
% l is length of segment mass is comming from hanavan parameters
% volume_object = object_name.get_volume()
% CoM_object = object_name.get_com()
% Ixx_object = object_name.get_Ixx()
% Iyy_object = object_name.get_Iyy()
% Izz_object = object_name.get_Izz()
% Ineria_matrix_object = object_name.get_inertiaMat()
%
%**************************************************************************

classdef ES
    
    properties
        a0
        b0
        a1
        b1
        l
        mass
    end
    
    properties (Dependent)
        B_ab1
        B_ab2 
        B_ab3 
        B_aaab4 
        B_aaab5
        B_aaab6
        B_aaab7
        B_aaab8
        B_abbb4
        B_abbb5
        B_abbb6
        B_abbb7
        B_abbb8
        A_ab1 
        A_ab2 
        A_ab3 
        A_aaab4 
        A_abbb4 
    end
    
    methods
        % Class constructor
        
        function this = ES(A0,B0,A1,B1,L,MASS)

            this.a0 = A0;
            this.a1 = A1;
            this.b0 = B0;
            this.b1 = B1;
            this.l = L;
            this.mass = MASS;

        end
        
        % Calculates the volume of Elliptical
        function volume = get_volume(this)
           volume = this.l*this.A_ab1*pi; 
        end
        
        % Calculates the position CoM of elliptical in global frame
        function CoM= get_com(this,marker_1,marker_2)
           u = (marker_1-marker_2)/norm(marker_2-marker_1);
           c = marker_1 - u*1000*(this.l*this.A_ab2/this.A_ab1); 
           CoM = c;
        end
        
        % Inertia of body in X directin in local frame at CoM
        function Ixx = get_Ixx(this)
            Ixx = 0.25*this.mass*(this.A_abbb4/this.A_ab1) + this.mass*(this.l^2)*(this.A_ab3/this.A_ab1) - this.mass*(this.l*this.A_ab2/this.A_ab1)^2;
        end
        
        % Inertia of body in X directin in local frame at CoM
        function Iyy= get_Iyy(this)
            Iyy = 0.25*this.mass*(this.A_aaab4/this.A_ab1) + this.mass*(this.l^2)*(this.A_ab3/this.A_ab1) - this.mass*(this.l*this.A_ab2/this.A_ab1)^2;
        end
        
        % Inertia of body in X directin in local frame at CoM
        function Izz= get_Izz(this)
            Izz = 0.25*this.mass*(this.A_aaab4+this.A_abbb4)/this.A_ab1;
        end
        
        % Inertia matrix of body in local frame at CoM
        function I= get_inertiaMat(this)
            I = [get_Ixx(this),0,0; 0,get_Iyy(this),0; 0,0,get_Izz(this)];
        end
            
    end
    
    
    % Calculating some internal parameters, You can find it ppt
    methods 
        
        function B_ab1 = get.B_ab1(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_ab1 = (A1-A0)*(B1-B0);
        end
        function B_ab2 = get.B_ab2(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_ab2 = A0*(B1-B0)+B0*(A1-A0);
        end
        function B_ab3 = get.B_ab3(obj)
            A0=obj.a0; B0=obj.b0;
            B_ab3 = A0*B0;
        end
        function B_aaab4 = get.B_aaab4(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_aaab4 = (A1-A0)*(A1-A0)*(A1-A0)*(B1-B0);
        end
        function B_aaab5 = get.B_aaab5(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_aaab5 = A0*(A1-A0)*(A1-A0)*(B1-B0) + A0*(A1-A0)*(A1-A0)*(B1-B0) + A0*(A1-A0)*(A1-A0)*(B1-B0) + B0*(A1-A0)*(A1-A0)*(A1-A0);
        end
        function B_aaab6 = get.B_aaab6(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_aaab6 = A0*A0*(A1-A0)*(B1-B0)+A0*A0*(A1-A0)*(B1-B0)+A0*B0*(A1-A0)*(A1-A0)+A0*A0*(A1-A0)*(B1-B0)+A0*B0*(A1-A0)*(A1-A0)+A0*B0*(A1-A0)*(A1-A0);
        end
        function B_aaab7 = get.B_aaab7(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_aaab7 = A0*A0*B0*(A1-A0) + A0*A0*B0*(A1-A0) + A0*A0*B0*(A1-A0) + A0*A0*A0*(B1-B0);
        end
        function B_aaab8 = get.B_aaab8(obj)
            A0=obj.a0; B0=obj.b0;
            B_aaab8 = A0*A0*A0*B0;
        end
        function B_abbb4  = get.B_abbb4 (obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_abbb4 = (A1-A0)*(B1-B0)*(B1-B0)*(B1-B0);
        end
        function B_abbb5 = get.B_abbb5(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_abbb5 = A0*(B1-B0)*(B1-B0)*(B1-B0) + B0*(A1-A0)*(B1-B0)*(B1-B0) + B0*(A1-A0)*(B1-B0)*(B1-B0) + B0*(A1-A0)*(B1-B0)*(B1-B0);
        end
        function B_abbb6 = get.B_abbb6(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_abbb6 = A0*B0*(B1-B0)*(B1-B0)+A0*B0*(B1-B0)*(B1-B0)+A0*B0*(B1-B0)*(B1-B0)+B0*B0*(A1-A0)*(B1-B0)+B0*B0*(A1-A0)*(B1-B0)+B0*B0*(A1-A0)*(B1-B0);
        end
        function B_abbb7 = get.B_abbb7(obj)
            A0=obj.a0; A1=obj.a1 ; B0=obj.b0; B1=obj.b1;
            B_abbb7 = B0*B0*B0*(A1-A0) + A0*B0*B0*(B1-B0) + A0*B0*B0*(B1-B0) + A0*B0*B0*(B1-B0);
        end
        function B_abbb8 = get.B_abbb8(obj)
            A0=obj.a0; B0=obj.b0;
            B_abbb8 = A0*B0*B0*B0;
        end
        function A_ab1 = get.A_ab1(obj)
            Bs_ab1=obj.B_ab1; Bs_ab2=obj.B_ab2 ; Bs_ab3=obj.B_ab3;
            A_ab1 = (Bs_ab1/3)+(Bs_ab2/2)+Bs_ab3;
        end
        function A_ab2 = get.A_ab2(obj)
            Bs_ab1=obj.B_ab1; Bs_ab2=obj.B_ab2 ; Bs_ab3=obj.B_ab3;
            A_ab2 = (Bs_ab1/4)+(Bs_ab2/3)+(Bs_ab3/2);
        end
        function A_ab3 = get.A_ab3(obj)
            Bs_ab1=obj.B_ab1; Bs_ab2=obj.B_ab2 ; Bs_ab3=obj.B_ab3;
            A_ab3 = (Bs_ab1/5)+(Bs_ab2/4)+(Bs_ab3/3);
        end
        function A_aaab4 = get.A_aaab4(obj)
            Bs_aaab4=obj.B_aaab4; Bs_aaab5=obj.B_aaab5; Bs_aaab6=obj.B_aaab6; Bs_aaab7=obj.B_aaab7; Bs_aaab8=obj.B_aaab8;
            A_aaab4 = (Bs_aaab4/5)+(Bs_aaab5/4)+(Bs_aaab6/3)+(Bs_aaab7/2)+Bs_aaab8;
        end
        function A_abbb4 = get.A_abbb4(obj)
            Bs_abbb4=obj.B_abbb4; Bs_abbb5=obj.B_abbb5; Bs_abbb6=obj.B_abbb6; Bs_abbb7=obj.B_abbb7; Bs_abbb8=obj.B_abbb8;
            A_abbb4 = (Bs_abbb4/5)+(Bs_abbb5/4)+(Bs_abbb6/3)+(Bs_abbb7/2)+Bs_abbb8;
        end
          
    end
end


       
