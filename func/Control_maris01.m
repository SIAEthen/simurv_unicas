function [tau, gamma_v_dot, gamma_q_dot] = Control_maris01(eta, q, zita, eta_d, q_d, zita_d, gamma_v, gamma_q)
%
% Computes the controller for MARIS: algorithm 01
%
% function [tau, gamma_v_dot, gamma_q_dot] = Control_maris01(eta, q, zita, eta_d, q_d, zita_d, gamma_v, gamma_q)
%
% input:
%       eta           dim 6x1     vehicle position/orientation
%       q             dim nx1     joint position
%       zita          dim 6+nx1   system velocities
%       eta_d         dim 6x1     desired vehicle position/orientation
%       q_d           dim nx1     desired joint position
%       zita_d        dim 6+nx1   desired system velocities
%       gamma_v       dim 9x1     vehicle parameters
%       gamma_q       dim nx1     robot integral actions
%
% output:
%       tau           dim 6+nx1   generalized forces
%       gamma_v_dot   dim 9x1     derivative of the vehicle parameters
%       gamma_q_dot   dim nx1     derivative of the robot integral actions
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta    = CheckVector(eta);
q      = CheckVector(q);
zita   = CheckVector(zita);
eta_d  = CheckVector(eta_d);
q_d    = CheckVector(q_d);
zita_d = CheckVector(zita_d);
gamma_v= CheckVector(gamma_v);
gamma_q= CheckVector(gamma_q);

n = size(q,1);

% vehicle paameters
Kd_v = diag([3*80 3*80 3*80 60 60 60]);
lambda = diag([.5 .5 .5 1 1 1]);
invKgamma = 20*eye(9,9);

% PID at joints
Kp = 200*diag([20 20 20 10 10 10 1]);
Ki = 5*diag([20 20 20 20 20 20 20]);
Kd = 1*diag([20 20 20 10 10 10 0.3]);


e   = Rpy2Quat(eta(4:6));
e_d = Rpy2Quat(eta_d(4:6));
e_o = e(4)*e_d(1:3) - e_d(4)*e(1:3) + cross(e_d(1:3),e(1:3));
eta_tilde_1 = eta_d(1:3) - eta(1:3);
nu_tilde = zita_d(1:6) - zita(1:6);
R = Quat2Rot(e);
s = lambda*[R'*eta_tilde_1;  e_o] + nu_tilde;
phi_p = [zeros(3,3)    R'            zeros(3,3);
         S(R'*[0;0;1])  zeros(3,3)   R'];
tau(1:6) = Kd_v*s + phi_p*gamma_v;

gamma_v_dot = invKgamma*phi_p'*s;

tau(7:6+n) = Kp*(q_d - q) + Kd*(zita_d(7:6+n)-zita(7:6+n)) + Ki*gamma_q;
gamma_q_dot = (q_d - q);

