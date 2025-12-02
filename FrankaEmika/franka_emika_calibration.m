get_emika_dh_params
get_emika_data

PARAM.DH = emika_dh_parameters;
PARAM.T_0_B = eye(4);

% tau_inertia_frame = 0*tau_body_frame;
% % transform tau into inertia frame
% for i=1:size(tau_inertia_frame,2)
%     eta_i = eta_i(:,i);
%     R_B_I = Quat2Rot(eta(4:7));
%     U_B_I = U_mat(R_B_I,[0,0,0]');
%     tau_inertia_frame(:,i) = U_B_I * tau_body_frame(:,i);
% end
num_of_data_fmincon = 20;
Y = zeros(7*num_of_data_fmincon,28);
meas = zeros(7*num_of_data_fmincon,1);
for i=1:num_of_data_fmincon
    Y(7*i-6:7*i,:) = G_regressor_emika(pos(1:7,i));
    meas(7*i-6:7*i,:) = tau(1:7,i);
end


% 代价函数句柄
fun = @(theta) 0.5 * norm(Y * theta - meas)^2;


% use fmincon to solve problem
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
A = [];
b = [];
Aeq = [];
beq = [];
lb =    -[0,10,10,10, ...
        0,10,10,10, ...
        0,10,10,10, ...
        0,10,10,10, ...
        0,10,10,10, ...
        0,10,10,10, ...
        0,10,10,10, ...
        ];
ub =    [5,10,10,10, ...
        5,10,10,10, ...
        10,10,10,10, ...
        10,10,10,10, ...
        10,10,10,10, ...
        10,10,10,10, ...
        10,10,10,10, ...
                ];
theta0 = zeros(28,1);
theta_fmincon = fmincon(fun, theta0, A, b, Aeq, beq, lb, ub, [], options);

clear mm YY
for i=num_of_data_fmincon:40
    j = i-num_of_data_fmincon+1;
    YY(7*j-6:7*j,:) = G_regressor_emika(pos(1:7,i));
    mm(7*j-6:7*j,:) = tau(1:7,i);
end
com = [YY*theta_fmincon mm];
error = YY*theta_fmincon - mm;
error'*error
for i = 1:4:length(theta_fmincon)
    fprintf("$[%.2f~%.2f~%.2f~%.2f]^T$,\n", theta_fmincon(i:i+3));
end
syms q1 q2 q3 q4 q5 q6 q7 real
q = [q1 q2 q3 q4 q5 q6 q7]';
g = G_regressor_emika(q)*theta_fmincon;


% Y = simplify(g,'Steps',50);

% g(abs(g) < 1e-6) = 0;

% Generate function from a symbolic expression for the regressor
matlabFunction(g,'File','FrankaEmika/get_emika_gravity',...
               'Vars',{q});