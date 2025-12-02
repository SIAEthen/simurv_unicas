function [zita_d, sigma_tilde_a, sigma_tilde_b, sigma_tilde_c] = InverseKynematics_maris01(eta_ee1_d,...
    eta_ee_quat_d, eta, q, DH, PARAM)
%
% Computes the inverse kynematics for MARIS: algorithm 01
%
% function [zita_d, sigma_tilde_a, sigma_tilde_b, sigma_tilde_c] = InverseKynematics_maris01(eta_ee1_d,...
%    eta_ee_quat_d, eta, q, DH, PARAM)
%
% input:
%       eta_ee1_d     dim 3x1     desired ee position
%       eta_ee_quat_d dim 4x1     desired ee quaternion
%       eta           dim 6x1     vehicle position/orientation
%       q             dim nx1     joint position
%       DH            dim nx4     DH table (contain joint positions)
%       PARAM         struct      parameters for the dynamic simulation
%
% output:
%       zita_d        dim 6+nx1   system velocities
%       sigma_tilde_a dim max1    error for task a
%       sigma_tilde_b dim mbx1    error for task b
%       sigma_tilde_c dim mcx1    error for task c
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

k_a = 10;
k_b = 10;
k_c = diag([0 10 0 0 0 0 0]);
n = size(DH,1);
W = diag([50 50 50 25 25 25 1*ones(1,n)]);invW=inv(W);

eta_ee1_d     = CheckVector(eta_ee1_d);
eta_ee_quat_d = CheckVector(eta_ee_quat_d);
eta  = CheckVector(eta);
q    = CheckVector(q);

% first-priority task - end-effector configuration
sigma_tilde_a = sigma_tilde05(eta_ee1_d,eta_ee_quat_d,eta,[DH(:,1:3) q],PARAM.T_0_B);
J_a = J05(eta,[DH(:,1:3) q],PARAM.T_0_B);
pinvJ_a = mypinv(J_a,invW,'a');
N_a = eye(6+n,6+n)-pinvJ_a*J_a;
% second-priority task - null roll/pitch
sigma_tilde_b = sigma_tilde12([0 0]/180*pi,eta(4:6));
J_b = J12(eta(4:6),n);
pinvJ_b = mypinv(J_b,invW,'b');
% third-priority task 
J_ab = [J_a; J_b];
pinvJ_ab = mypinv(J_ab,invW);
N_ab = eye(6+n,6+n)-pinvJ_ab*J_ab;
sigma_tilde_c = sigma_tilde09(q,PARAM.q_nominal);
J_c = J09(q);
pinvJ_c = mypinv(J_c,invW,'c');

%-------------------
zita_d =      pinvJ_a*k_a*sigma_tilde_a + ...
          N_a*pinvJ_b*k_b*sigma_tilde_b +...
         N_ab*pinvJ_c*k_c*sigma_tilde_c;
zita_d = VectorSat(zita_d,PARAM.zita_limit);


