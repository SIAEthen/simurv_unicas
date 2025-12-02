function [tau, gamma_v_dot, gamma_q_dot] = Control_maris02(eta, DH, zita, eta_d, q_d, zita_d, gamma_v, gamma_q, PARAM)
%
% Computes the controller for MARIS: algorithm 02
%
% function [tau, gamma_v_dot, gamma_q_dot] = Control_maris02(eta, q, zita, eta_d, q_d, zita_d, gamma_v, gamma_q, PARAM)
%
% input:
%       eta           dim 6x1     vehicle position/orientation
%       DH            dim nx4     Denavit-Hartenberg table (include joint pos)
%       zita          dim 6+nx1   system velocities
%       eta_d         dim 6x1     desired vehicle position/orientation
%       q_d           dim nx1     desired joint position
%       zita_d        dim 6+nx1   desired system velocities
%       gamma_v       dim 9(n+1)x1vehicle parameters
%       gamma_q       dim nx1     robot integral actions
%       PARAM         struct      kin/dyn parameters
%
% output:
%       tau           dim 6+nx1   generalized forces
%       gamma_v_dot   dim 9(n+1)x1derivative of the controller parameters
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
Kd_v = 200*diag([4 4 4 3 3 3]);
lambda = diag([.2 .2 .2 1 1 1]);
invKgamma = diag([240*ones(9,1); 24*ones(9*3,1); 2.4*ones(9*4,1)]);

% PID at joints
Kp = 100*diag([10 10 20 10 10 10 1]);
Ki = diag([2 2 2 2 2 2 2]);
Kd = 1*diag([22 12 15 10 10 10 0.3]);

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
Y = zeros(6,9*(n+1));           % total regressor
Y(:,1:9) = Regressor_smart(RBI);
U_iB = U0B;
% building of the regressor
U_current = U_mat(Rot_dh(DH(1,2),DH(1,4)),[0 0 0]);
U_iB = U_iB*U_current;
Y(:,1+9*1:9*(1+1)) = U_iB*Regressor_smart(RiI(:,:,1));
for i=2:n    
    % force transformation from consecutive links
    U_current = U_mat(Rot_dh(DH(i,2),DH(i,4)),r_(:,i-1));
    % build U from link i to body-fixed frame
    U_iB = U_iB*U_current;
    Y(:,1+9*i:9*(i+1)) = U_iB*Regressor_smart(RiI(:,:,i));
end

% vehicle controller
e   = Rpy2Quat(eta(4:6));
e_d = Rpy2Quat(eta_d(4:6));
e_o = e(4)*e_d(1:3) - e_d(4)*e(1:3) + cross(e_d(1:3),e(1:3));
eta_tilde_1 = eta_d(1:3) - eta(1:3);
nu_tilde = zita_d(1:6) - zita(1:6);
%RBI = Quat2Rot(e);
s = nu_tilde + lambda*[RBI'*eta_tilde_1;  e_o];
%phi_p = Regressor_smart(R);
%tau(1:6) = Kd_v*s + phi_p*gamma_v;
tau(1:6) = Kd_v*s + Y*gamma_v;

gamma_v_dot = invKgamma*Y'*s;


% arm controller (PID at joints)
tau(7:6+n) = Kp*(q_d - q) + Kd*(zita_d(7:6+n)-zita(7:6+n)) + Ki*gamma_q;
gamma_q_dot = (q_d - q);

