function J = J18(eta_ee1d,eta,DH,T_0_B)
%
% Computes the Jacobian for the task function:
%   end-effector error position norm version 2
%
% function  J = J18(eta_ee1d,eta,DH,T_0_B)
%
% input:
%       eta_ee1d dim 3x1     end-effector desired position
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                            matrix from vehicle to zero frame
%
% output:
%       J        dim 1x6+n   Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta_ee1d = CheckVector(eta_ee1d);
eta      = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = TT(1:3,4);

J = Jacobian(eta,DH,T_0_B);
J = J(1:3,:);
J = - (eta_ee1d-eta_ee1)'*J;
