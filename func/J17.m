function J = J17(p_d,eta,DH,T_0_B)
%
% Computes the Jacobian for the task function:
%   end-effector field-of-view "cameraman" error
%   second version with spherical coordinates
%
% function  J = J17(a_d,eta,DH,T_0_B)
%
% input:
%       p_d      dim 3x1     point to be framed
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                            matrix from vehicle to zero frame
%
% output:
%       J        dim 2x6+n   Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

p_d = CheckVector(p_d);
eta = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = TT(1:3,4);
R_I_ee  = TT(1:3,1:3)';
r       = R_I_ee*(p_d - eta_ee1);
normr  = sqrt(r'*r);

Jdummy = Jacobian(eta,DH,T_0_B);
Jpos = Jdummy(1:3,:);
Jor  = Jdummy(4:6,:);

dummyx = (-1)/(r'*r*sqrt(1-r(1)/normr));
dummyy = (-1)/(r'*r*sqrt(1-r(2)/normr));

drdq = - R_I_ee*Jpos + S(r)*R_I_ee*Jor;

J(1,:) = dummyx*(normr*[1 0 0]*drdq - r(1)*r'*drdq/normr);
J(2,:) = dummyy*(normr*[0 1 0]*drdq - r(2)*r'*drdq/normr);
