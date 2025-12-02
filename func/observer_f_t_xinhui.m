function [tau_e_new,sigma_observer] = observer_f_t_xinhui(eta, hat_theta,tau_e,acc_lin,vel_ang,tau,sigma_observer,Ts)
%OBSERVER_F_T_XINHUI Summary of this function goes here
%   Detailed explanation goes here
eta = CheckVector(eta);
hat_theta = CheckVector(hat_theta);
hat_f = CheckVector(tau_e(1:3));
hat_t = CheckVector(tau_e(4:6));

acc_lin = CheckVector(acc_lin);
vel_ang = CheckVector(vel_ang);

tau_lin = CheckVector(tau(1:3));
tau_ang = CheckVector(tau(4:6));
Mass_matrix = 100 * eye(3);
I_matrix = 100 * eye(3);


K_lin = diag([10,10,10]);
K_ang = diag([10,10,10]);


e   = Rpy2Quat(eta(4:6));
R = Quat2Rot(e);
Phi = [zeros(3,3)    R'            zeros(3,3);
         S(R'*[0;0;1])  zeros(3,3)   R'];
Phi_lin = Phi(1:3,:);
Phi_ang = Phi(4:6,:);

hat_f_dot = -K_lin * hat_f + K_lin*(Mass_matrix*acc_lin + Phi_lin * hat_theta - tau_lin);
hat_f_new = hat_f + hat_f_dot * Ts;


% assume P_0 is 0
sigma_observer = sigma_observer + Ts*(tau_ang + hat_t - Phi_ang * hat_theta);
hat_t_new = K_ang * (I_matrix*vel_ang - sigma_observer);

tau_e_new = [hat_f_new;hat_t_new];
end

