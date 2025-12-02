function [tau, gamma_v_dot, gamma_q_dot] = Control_maris03(eta, DH, zita, eta_d, q_d, zita_d, gamma_v, gamma_q, PARAM)
%
% Computes the controller for MARIS: algorithm 03
%
% function [tau, gamma_v_dot, gamma_q_dot] = Control_maris03(eta, DH, zita, eta_d, q_d, zita_d, gamma_v, gamma_q, PARAM)
%
% input:
%       eta           dim 6x1     vehicle position/orientation
%       DH            dim nx4     Denavit-Hartenberg table (include joint pos)
%       zita          dim 6+nx1   system velocities
%       eta_d         dim 6x1     desired vehicle position/orientation
%       q_d           dim nx1     desired joint position
%       zita_d        dim 6+nx1   desired system velocities
%       gamma_v       dim 9+6nx1  vehicle parameters
%       gamma_q       dim nx1     robot integral actions
%       PARAM         struct      kin/dyn parameters
%
% output:
%       tau           dim 6+nx1   generalized forces
%       gamma_v_dot   dim 9+6nx1  derivative of the controller parameters
%       gamma_q_dot   dim 9nx1    derivative of the robot integral actions
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta    = CheckVector(eta);
zita   = CheckVector(zita);
eta_d  = CheckVector(eta_d);
q_d    = CheckVector(q_d);
zita_d = CheckVector(zita_d);
gamma_v= CheckVector(gamma_v);
gamma_q= CheckVector(gamma_q);
q      = DH(:,4);
T_0_B  = PARAM.T_0_B;
R_0_B  = T_0_B(1:3,1:3);
r_B0_B = T_0_B(1:3,4);
r_  = PARAM.r_;

n = size(q,1);

% vehicle control parameters
Kd_v = diag([3*80 3*80 3*80 60 60 60]);
lambda = diag([.5 .5 .5 1 1 1]);
invKgamma = diag([20*ones(9,1); 2*ones(6*3,1); 1*ones(6*4,1)]);

% PID at joints
Kp = 200*diag([20 20 20 10 10 10 1]);
Ki = 5*diag([20 20 20 20 20 20 20]);
Kd = 1*diag([20 20 20 10 10 10 0.3]);

% Compute all the rotations with respect to the inertial frame
RBI = Rpy2Rot(eta(4:6));        % from vehicle-fixed to inertial
R0I = RBI*T_0_B(1:3,1:3);     % from zero to intertial
RiI = zeros(3,3,n);
RiI(:,:,1) = R0I*Rot_dh(DH(1,2),DH(1,4));
% manipulator cycle
for i=2:n
    RR = Rot_dh(DH(i,2),DH(i,4));
    RiI(:,:,i) = RiI(:,:,i-1)*RR;
end

% Compute all the consecutives U
U0B = U_mat(R_0_B, r_B0_B);
Y = zeros(6,9+6*n);           % total regressor
Y(:,1:9) = Regressor_smart(RBI);
U_iB = U0B;
% building of the regressor
U_current = U_mat(Rot_dh(DH(1,2),DH(1,4)),[0 0 0]);
U_iB = U_iB*U_current;
Y(:,9+1:9+6) = U_iB*Regressor_smart2(RiI(:,:,1));
for i=2:n    
    % force transformation from consecutive links
    U_current = U_mat(Rot_dh(DH(i,2),DH(i,4)),r_(:,i-1));
    % build U from link i to body-fixed frame
    U_iB = U_iB*U_current;
    Y(:,9+1+6*(i-1):9+6*i) = U_iB*Regressor_smart2(RiI(:,:,i));
end

% vehicle controller
e   = Rpy2Quat(eta(4:6));
e_d = Rpy2Quat(eta_d(4:6));
e_o = e(4)*e_d(1:3) - e_d(4)*e(1:3) + cross(e_d(1:3),e(1:3));
eta_tilde_1 = eta_d(1:3) - eta(1:3);
nu_tilde = zita_d(1:6) - zita(1:6);
s = nu_tilde + lambda*[RBI'*eta_tilde_1;  e_o];
tau(1:6) = Kd_v*s + Y*gamma_v;
gamma_v_dot = invKgamma*Y'*s;


% arm controller (PID at joints)
tau(7:6+n) = Kp*(q_d - q) + Kd*(zita_d(7:6+n)-zita(7:6+n)) + Ki*gamma_q;
gamma_q_dot = (q_d - q);

