%
% Phoenix + 3 link planar on the vertical plane
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

fprintf('\n Phoenix + 3 link planar on the vertical plane\n')
fprintf('\n Description of the model')

%------------------------
% vehicle parameters 
% here only the geometric data, the remaining in function
% that computes that vehicle dyanmics
%------------------------
PARAM.type = 'parallelepiped';

%  rotation matrix from the manipulator-base frame (zero) to the vehicle-fixed
R_0_B = [1  0  0
         0  -1  0
         0  0  -1];

r_B0_B = [5.3/2 0 0]';	% vector position from origin of frame B to the
             		% origin of frame O, expressed in frame B [m]
T_0_B = [R_0_B, r_B0_B; 0 0 0 1];
                                
% semi-axis for the ellipsoid representing the vehicle
L  = 5.3;   % [m]
a1 = L/2;				% vehicle-fixed x
a2 = .6;        		% vehicle-fixed y
a3 = .5;        		% vehicle-fixed z

PARAM.T_B_0 = T_0_B;
PARAM.a1 = a1;
PARAM.a2 = a2;
PARAM.a3 = a3;

%------------------------
% manipulator parameters 
%------------------------

% D-H table
DH_a     = [1 .5 .5]';      % [m]
DH_alpha = [0 0 0]';        % [rad]
DH_d     = [0 0 0]';        % [m]
DH_theta = [0 0 0]'/180*pi; % [rad]
DH = [DH_a DH_alpha  DH_d  DH_theta];

% r_ matrix 3xn with in column the vector position
%   from origin of frame i-1 to origin of frame i expressed in frame i.
r_ = zeros(3,length(DH_a));
for i=1:length(DH_a)
    r_(:,i) = [DH_a(i); DH_d(i)*sin(DH_alpha(i)); DH_d(i)*cos(DH_alpha(i))];
end

% r_c matrix 3xn with in column the vector position
%   from origin of frame i-1 to the center of mass of link i expressed in frame i.
r_c  = [ .3  .2 .1 
          0   0   0  
          0   0   0];      % [m]

% r_b matrix 3xn with in column the vector position
%   from origin of frame i-1 to the center of buoyancy of link i expressed in frame i.
r_b  = [ .3  .2 .1 
          0   0   0  
          0   0   0];      % [m]
      
% r_k matrix 3xn with in column the vector position
%   from origin of frame i to the center of mass of link i expressed in frame i.
r_k = r_c - r_;

g0 = [0 0 9.81]';   % gravity acceleration in inertial frame [m/s^2]
rho = 1000;		    % water density [kg/m^3]

r_l = [.15 .1 .1];  % link radius [m]
L_l = [1  .5 .3];   %link length [m]
delta = pi*(r_l.^2).*(L_l);     % link volumes [m^3]
m = [80 30 20];     % link masses [kg]

% each link is modeled as a cylinder so the 6x6 added matrix is diagonal
% the asimmetric term of the added masses and inertias is the direction
% of the cylinder length

% M is a 3x3xn matrix where M(:,:,i) is the mass of link i
M = zeros(3,3,3);
for i=1:3
    M(:,:,i) = m(i)*eye(3,3);
end
% Ma is a 3x3xn matrix where Ma(:,:,i) is the added mass of link i
Ma = zeros(3,3,3);
Ma(:,:,1) = diag([ 0.1*m(1), rho*delta(1), rho*delta(1)]);
Ma(:,:,2) = diag([ 0.1*m(2), rho*delta(2), rho*delta(2)]);
Ma(:,:,3) = diag([ 0.1*m(3), rho*delta(3), rho*delta(3)]);

% I is a 3x3xn matrix where I(:,:,i) is the inertia of link i
I = zeros(3,3,3);
I(:,:,1) = diag([5 1.5 5]);
I(:,:,2) = diag([1 4 4]);
I(:,:,3) = diag([.1 .025 .1]);
% Ia is a 3x3xn matrix where Ia(:,:,i) is the added inertia of link i
Ia = zeros(3,3,3);
Ia(:,:,1) = diag([0, (pi*rho*r_l(1)^2*L_l(1)^2)/12, (pi*rho*r_l(1)^2*L_l(1)^2)/12]);
Ia(:,:,2) = diag([0, (pi*rho*r_l(2)^2*L_l(2)^2)/12, (pi*rho*r_l(2)^2*L_l(2)^2)/12]);
Ia(:,:,3) = diag([0, (pi*rho*r_l(3)^2*L_l(3)^2)/12, (pi*rho*r_l(3)^2*L_l(3)^2)/12]);

% dry friction of the links
fric_dry = 0*[20 10 5];
% viscous friction of the links
fric_vis = [30 20 5];

Ds = 0.4;   % linear skin coefficient
CD = 0.6;   % quadratic drag coefficient
CL = 0;		% lift coefficient

zita_limit = [.2 .2 .2 .2 .2 .2 [20 20 20]/180*pi];

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

clear i L rv_g rv_b a1 a2 a3 R_0_B r_B0_B T_B_0 rho g0
clear DH_a DH_alpha DH_d DH_theta r_k r_c r_b r_ m 
clear r_l L_l delta
clear M Ma I Ia 
clear fric_dry fric_vis Ds CD CL zita_limit


PrintData(DH,PARAM);
