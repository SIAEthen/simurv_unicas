function sigma_tilde = sigma_tilde17(p_d,eta,DH,T_0_B)
%
% Computes the task function:
%   end-effector field-of-view "cameraman" error
%   second version with spherical coordinates
%
% function sigma_tilde = sigma_tilde17(p_d,eta,DH,T_0_B)
%
% input:
%       p_d      dim 3x1     point to be framed
%       eta      dim 6x1     vehicle position/orientation
%       DH       dim nx4     Denavit-Hartenberg table
%       T_0_B    dim 4x4     Homogeneous transformation 
%                            matrix from vehicle to zero frame
%
% output:
%       sigma_tilde    dim 2x1     end-effector field-of-view error
%
% G. Antonelli, Simurv 4.0, 2013

p_d = CheckVector(p_d);
eta = CheckVector(eta);

TT = DirectKinematics(eta,DH,T_0_B);
eta_ee1 = TT(1:3,4);
r       = TT(1:3,1:3)'*(p_d - eta_ee1);
normr = sqrt(r'*r);

sigma_tilde(1,1) = pi/2 - acos(r(1)/normr);
sigma_tilde(2,1) = pi/2 - acos(r(2)/normr);
