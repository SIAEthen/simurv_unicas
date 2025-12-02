%
% MARIS: Romeo + trident arm
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

fprintf('\n MARIS: Romeo + trident arm\n')
fprintf('\n Description of the model')

%------------------------
% vehicle parameters 
% here only the geometric data, the remaining in function
% that computes that vehicle dyanmics
%------------------------
PARAM.type = 'Romeo';

%  rotation matrix from the manipulator-base frame (zero) to the vehicle-fixed
R_0_B = [1   0   0
         0   1   0
         0   0   1];

r_B0_B = [0 0 0.5]'; % vector position from origin of frame B to the
             		 % origin of frame 0, expressed in frame B [m]
T_0_B = [R_0_B, r_B0_B; 0 0 0 1];
                                
% semi-axis for the ellipsoid/openframe representing the vehicle
L  = 1.3;   % [m]
a1 = L/2;		% vehicle-fixed x
a2 = .45;		% vehicle-fixed y
a3 = .5;    % vehicle-fixed z

PARAM.T_0_B = T_0_B;
PARAM.a1 = a1;
PARAM.a2 = a2;
PARAM.a3 = a3;

%------------------------
% manipulator parameters 
%------------------------

% D-H table
DH_a     = [0      0      0     0      0     0      0]';   % [m]
DH_alpha = [-pi/2   pi/2  -pi/2  pi/2  -pi/2  pi/2   0]';   % [rad]
DH_d     = [.213  0      .427  0      .42   0     .153]'; % [m]
DH_theta = [0 40 0 -10 0 -15 0]'/180*pi; % [rad]
DH = [DH_a DH_alpha  DH_d  DH_theta];

% remember (see file InverseDynamics_forces.png):
%   link i connects joint i with joint i+1
%   frame i-1 is positioned along joint i
%   frame i   is positioned along joint i+1
%   force/moment/torque i   are acting on joint i   -> origin of frame i-1
%   force/moment/torque i+1 are acting on joint i+1 -> origin of frame i

% r_ matrix 3xn with in column the vector position
%   from origin of frame i-1 to origin of frame i expressed in frame i.
r_ = zeros(3,length(DH_a));
for i=1:length(DH_a)
    r_(:,i) = [DH_a(i); 
               DH_d(i)*sin(DH_alpha(i)); 
               DH_d(i)*cos(DH_alpha(i))];
end

% r_c matrix 3xn with in column the vector position
%   from origin of frame i-1 to the center of mass of link i expressed in frame i.
r_c  = [0       0      0     0       0      0      0 
       -0.1     0     -0.15  0      -0.15   0      0
        0       0.03   0     0.03    0      0.03   0.15];      % m

% r_b matrix 3xn with in column the vector position
%   from origin of frame i-1 to the center of buoyancy of link i expressed in frame i.
r_b  = [0       0      0     0       0      0      0 
       -0.1     0     -0.25  0      -0.25   0      0
        0       0.03   0     0.03    0      0.03   0.15];      % m
      
% r_k matrix 3xn with in column the vector position
%   from origin of frame i to the center of mass of link i expressed in frame i.
r_k = r_c - r_;

g0 = [0 0 9.81]';   % gravity acceleration in inertial frame
rho = 1000;		    % water density [kg/m^3]

r_l = [.07 .06 .06 .06 .06 .06 .06];  % link radius [m]
L_l = [.21 .14 .29 .12 .29 .13 .05];  % link length [m]
delta = pi*(r_l.^2).*(L_l);   % link volumes [m^3]
m = [6 4.5 5.8 3.7 4.2 3.5 1.5];      % link masses [kg]

% each link is modeled as a cylinder so the 6x6 added matrix is diagonal
% the asimmetric term of the added masses and inertias is the direction
% of the cylinder length

% M is a 3x3xn matrix where M(:,:,i) is the mass of link i
M = zeros(3,3,7);
for i=1:7
    M(:,:,i) = m(i)*eye(3,3);
end
% Ma is a 3x3xn matrix where Ma(:,:,i) is the added mass of link i
Ma = zeros(3,3,7);
Ma(:,:,1) = diag([ rho*delta(1), 0.1*m(1), rho*delta(1)]);
Ma(:,:,2) = diag([ rho*delta(2), rho*delta(2), 0.1*m(2)]);
Ma(:,:,3) = diag([ rho*delta(3), 0.1*m(3), rho*delta(3)]);
Ma(:,:,4) = diag([ rho*delta(4), rho*delta(4), 0.1*m(4)]);
Ma(:,:,5) = diag([ rho*delta(5), 0.1*m(5), rho*delta(5)]);
Ma(:,:,6) = diag([ rho*delta(6), rho*delta(6), 0.1*m(6)]);
Ma(:,:,7) = diag([ rho*delta(7), rho*delta(7), 0.1*m(7)]);

% I is a 3x3xn matrix where I(:,:,i) is the inertia of link i
I = zeros(3,3,7);
I(:,:,1) = .5*m(1)*diag([.5*r_l(1)^2+L_l(1)^2/6, r_l(1)^2,               .5*r_l(1)^2+L_l(1)^2/6]);
I(:,:,2) = .5*m(2)*diag([.5*r_l(2)^2+L_l(2)^2/6, .5*r_l(2)^2+L_l(2)^2/6, r_l(2)^2]);
I(:,:,3) = .5*m(3)*diag([.5*r_l(3)^2+L_l(3)^2/6, r_l(3)^2,               .5*r_l(3)^2+L_l(3)^2/6]);
I(:,:,4) = .5*m(4)*diag([.5*r_l(4)^2+L_l(4)^2/6, .5*r_l(4)^2+L_l(4)^2/6, r_l(4)^2]);
I(:,:,5) = .5*m(5)*diag([.5*r_l(5)^2+L_l(5)^2/6, r_l(5)^2,               .5*r_l(5)^2+L_l(5)^2/6]);
I(:,:,6) = .5*m(6)*diag([.5*r_l(6)^2+L_l(6)^2/6, .5*r_l(6)^2+L_l(6)^2/6, r_l(6)^2]);
I(:,:,7) = .5*m(7)*diag([.5*r_l(7)^2+L_l(7)^2/6, .5*r_l(7)^2+L_l(7)^2/6, r_l(7)^2]);
% Huygens-Steiner
I(:,:,1) = I(:,:,1) + m(1)*r_l(1)^2*diag([1 0 1]);
I(:,:,2) = I(:,:,2) + m(2)*r_l(2)^2*diag([1 1 0]);
I(:,:,3) = I(:,:,3) + m(3)*r_l(3)^2*diag([1 0 1]);
I(:,:,4) = I(:,:,4) + m(4)*r_l(4)^2*diag([1 1 0]);
I(:,:,5) = I(:,:,5) + m(5)*r_l(5)^2*diag([1 0 1]);
I(:,:,6) = I(:,:,6) + m(6)*r_l(6)^2*diag([1 1 0]);
I(:,:,7) = I(:,:,7) + m(7)*r_l(7)^2*diag([1 1 0]);

% Ia is a 3x3xn matrix where Ia(:,:,i) is the added inertia of link i
Ia = zeros(3,3,7);
Ia(:,:,1) = diag([(pi*rho*r_l(1)^2*L_l(1)^2)/12, 0, (pi*rho*r_l(1)^2*L_l(1)^2)/12]);
Ia(:,:,2) = diag([(pi*rho*r_l(2)^2*L_l(2)^2)/12, (pi*rho*r_l(2)^2*L_l(2)^2)/12, 0]);
Ia(:,:,3) = diag([(pi*rho*r_l(3)^2*L_l(3)^2)/12, 0, (pi*rho*r_l(3)^2*L_l(3)^2)/12]);
Ia(:,:,4) = diag([(pi*rho*r_l(4)^2*L_l(4)^2)/12, (pi*rho*r_l(4)^2*L_l(4)^2)/12, 0]);
Ia(:,:,5) = diag([(pi*rho*r_l(5)^2*L_l(5)^2)/12, 0, (pi*rho*r_l(5)^2*L_l(5)^2)/12]);
Ia(:,:,6) = diag([(pi*rho*r_l(6)^2*L_l(6)^2)/12, (pi*rho*r_l(6)^2*L_l(6)^2)/12, 0]);
Ia(:,:,7) = diag([(pi*rho*r_l(7)^2*L_l(7)^2)/12, (pi*rho*r_l(7)^2*L_l(7)^2)/12, 0]);

% dry friction of the links
fric_dry = 0*[20 10 5  10 5 6 1];
% viscous friction of the links
fric_vis = [2 2 2 2 .5 1 .1];

Ds = 2*0.4;   % linear skin coefficient
CD = 2*0.6;   % quadratic drag coefficient
CL = 0;		% lift coefficient

zita_limit = [.2 .2 .2 .2 .2 .2 [20 20 20 20 20 20 20]/180*pi];
q_min = [-2.8 -1.6 -2.9 -1.6 -2.9 -1.44 -2.8];
q_max = [ 2.8  1.6  2.9  1.6  2.9  1.3   2.8];
q_nominal = [0 45 0 0 0 0 0]/180*pi;

PARAM.r_  = r_;
PARAM.r_c = r_c;
PARAM.r_b = r_b;
PARAM.r_k = r_k;
PARAM.rho = rho;
PARAM.g0  = g0;
PARAM.delta  = delta;
PARAM.m   = m;
PARAM.M   = M;
PARAM.Ma  = Ma;
PARAM.I   = I;
PARAM.Ia  = Ia;
PARAM.fric_dry = fric_dry;
PARAM.fric_vis = fric_vis;
PARAM.Ds = Ds;
PARAM.CD = CD;
PARAM.CL = CL;
PARAM.zita_limit = zita_limit;
PARAM.q_min = q_min;
PARAM.q_max = q_max;
PARAM.q_nominal = q_nominal;

clear i L rv_g rv_b a1 a2 a3 R_0_B r_B0_B T_B_0 rho g0
clear DH_a DH_alpha DH_d DH_theta r_k r_c r_b r_ m 
clear r_l L_l delta
clear M Ma I Ia 
clear fric_dry fric_vis Ds CD CL zita_limit q_nominal

PrintData(DH,PARAM);
	




