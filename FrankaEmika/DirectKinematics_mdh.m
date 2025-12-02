function T = DirectKinematics_mdh(MDH)
%
% Computes the homogeneous transformation matrix 
%	from intertial frame to end-effector
%
% function T = DirectKinematics(eta,DH,T_0_B)
%
% input:
%       eta    dim 6x1     vehicle position/orientation
%       DH     dim nx4     Denavit-Hartenberg table
%       T_0_B  dim 4x4     Homogeneous transformation 
%                          matrix from vehicle to zero frame
%
% output:
%       T      dim 4x4    Homogeneous transformation 
%                         matrix from inertial to end-effector
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv



% from vehicle-fixed to zero
T = eye(4);
n = size(MDH,1);
% manipulator cycle
for i=1:n
    TT = TMDH(MDH(i,1),MDH(i,2),MDH(i,3),MDH(i,4));
    T = T*TT;
end
