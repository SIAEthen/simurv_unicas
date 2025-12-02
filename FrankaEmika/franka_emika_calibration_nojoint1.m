get_emika_dh_params
get_emika_data
% tau_inertia_frame = 0*tau_body_frame;
% % transform tau into inertia frame
% for i=1:size(tau_inertia_frame,2)
%     eta_i = eta_i(:,i);
%     R_B_I = Quat2Rot(eta(4:7));
%     U_B_I = U_mat(R_B_I,[0,0,0]');
%     tau_inertia_frame(:,i) = U_B_I * tau_body_frame(:,i);
% end
num_of_data_fmincon = 20;

Y = zeros(6*num_of_data_fmincon,24);
meas = zeros(6*num_of_data_fmincon,1);
for i=1:num_of_data_fmincon
    gressor = G_regressor_emika(pos(1:7,i));
    Y(6*i-5:6*i,:) = gressor(2:7,5:28);
    meas(6*i-5:6*i,:) = tau(2:7,i);
end


% 代价函数句柄
fun = @(theta) 0.5 * norm(Y * theta - meas)^2;


% use fmincon to solve problem
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
theta0 = zeros(24,1);
theta_fmincon = fmincon(fun, theta0, A, b, Aeq, beq, lb, ub, [], options);

clear mm YY
YY = [];
mm = [];
for i=num_of_data_fmincon:40
    j = i-num_of_data_fmincon+1;
    gressor = G_regressor_emika(pos(1:7,i));
    YY = [YY; gressor(2:7,5:28)];
    mm = [mm;tau(2:7,i)];
end
error = YY*theta_fmincon - mm;
error'*error
theta_fmincon