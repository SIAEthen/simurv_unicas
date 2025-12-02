close all
% start calculate
calculate_sdh
% load data
data = load("Girona500/out.txt");
% data = load("Girona500/out_lower_velocity.txt");
q = data(:,8:11)';
tau_body_frame = data(:,12:17)';
% for fix bugs
tau_body_frame(4:6,:) = -1 * tau_body_frame(4:6,:);

eta = data(:,18:24)';
deta = data(:,25:30);
num_of_test = 3370;
num_of_data = size(data,1)-num_of_test;

tau_inertia_frame = 0*tau_body_frame;
% transform tau into inertia frame
for i=1:size(tau_inertia_frame,2)
    eta_i = eta(:,i);
    R_B_I = Quat2Rot(eta_i(4:7));
    U_B_I = U_mat(R_B_I,[0,0,0]');
    tau_inertia_frame(:,i) = U_B_I * tau_body_frame(:,i);
end

Y = zeros(6*num_of_data,20);
meas = zeros(6*num_of_data,1);

weight_matrix = diag([1,1,1,1,1,1]);
for i=1:num_of_data
    q_i = q(:,i);
    eta_i = eta(:,i);
    Y_i = Regressor_Girona500_inertia_frame(eta_i,q_i,GironaPARAMS) ;% with fixed regressor
    meas_i = tau_inertia_frame(:,i);
    % plus weight matrix
%     Y_i = weight_matrix*Y_i;
    meas_i = weight_matrix*meas_i;
    Y(1+6*(i-1):6*i,:) = Y_i;
    meas(1+6*(i-1):6*i,:) = meas_i;
end

% theta = -1.0*pinv(Y)*meas
% 
% Y_arms = Y(:,5:20);
% rank(Y_arms)
% Y_arms2 = Y(:,1:6);
% rank(Y_arms2)

% base parameters of vehicle and each link calculation
rho = 1000; % water density
g = 9.8;
mass = [144.681+3.825  0.845 5.734 1.516 0.308+0.579]'; % kg
volum = [147417.3+3824.7 844.7 5733.7 1515.9 307.5+579.3]'/100.0/100.0/100.0;% cm3
theta_0_calculate = g*(mass - rho*volum); % -26.8128 0.0029 0.0029 0.0010 0.0020


% 代价函数句柄
fun = @(theta) 0.5 * norm(Y * theta - meas)^2;

theta0 = zeros(20,1);
theta0(1,1) = theta_0_calculate(1,1); % net bouyant > 0, W-B <0
theta0(4,1) = 150; % net bouyant > 0, W-B <0
theta0(5,1) = theta_0_calculate(2,1);
theta0(9,1) = theta_0_calculate(3,1);
theta0(13,1) = theta_0_calculate(4,1);
theta0(17,1) = theta_0_calculate(5,1);
% use fmincon to solve problem
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
% lb = theta0 - [5,100,100,100, 5,100,100,100, 5,100,100,100, 5,100,100,100, 5,100,100,100]';
% ub = theta0 + [5,100,100,100, 5,100,100,100, 5,100,100,100, 5,100,100,100, 5,100,100,100]';
% lb = theta0 - [50,20,20,20,50,20,20,20,50,20,20,20,50,20,20,20,50,20,20,20]';
% ub = theta0 + [50,20,20,20,50,20,20,20,50,20,20,20,50,20,20,20,50,20,20,20]';
% lb = theta0 - [10,150,150,150,   10,50,50,50,  10,50,50,50, 10,50,50,50, 10,50,50,50]';
% ub = theta0 + [10,150,150,150, 10,50,50,50,  10,50,50,50, 10,50,50,50, 10,50,50,50]';
lb = theta0 - [10,50,50,500,    10,20,20,20,  10,20,20,20, 10,20,20,20,    10,20,20,20]';
ub = theta0 + [10,50,50,500,    10,20,20,20,  10,20,20,20, 10,20,20,20,    10,20,20,20]';
theta_est = fmincon(fun, theta0, A, b, Aeq, beq, lb, ub, [], options);

predict = zeros(6,num_of_test);
measure = zeros(6,num_of_test);
error = zeros(6,num_of_test);
error_relative = zeros(6,num_of_test);
for i=num_of_data+1:size(data,1)
    q_i = q(:,i);
    eta_i = eta(:,i);
    Y_i = Regressor_Girona500_inertia_frame(eta_i,q_i,GironaPARAMS) ;% with fixed regressor
    
    meas_i = tau_inertia_frame(:,i);
    pred_i = Y_i*theta_est;
    error_i = pred_i - meas_i;
    error(:,i-num_of_data) = error_i;
    predict(:,i-num_of_data) = pred_i;
    error_relative(:,i-num_of_data) = error_i./meas_i;
    measure(:,i-num_of_data) = (meas_i);
end

title_list = ["X","Y","Z","Rx","Ry","Rz"];
y_list = ["Force (N)","Force (N)","Force (N)","Torque (Nm)","Torque (Nm)","Torque (Nm)"];
x_list = ["Simulations","Simulations","Simulations","Simulations","Simulations","Simulations"];
for i=1:6
    figure
    x = 1:size(predict,2);
    plot(x,measure(i,:),x,predict(i,:));
    legend(["simulation","predict"]);
    title(title_list(i))
    xlabel(x_list(i))
    ylabel(y_list(i))
end

title_list = ["X","Y","Z","Rx","Ry","Rz"];
x_list = ["Force (N)","Force (N)","Force (N)","Torque (Nm)","Torque (Nm)","Torque (Nm)"];
for i=1:6
    figure
    data = predict(i,:)-measure(i,:);
    histogram(data,100)
    title("Predict error distribution in "+title_list(i))
    xlabel(x_list(i))
    ylabel("Number")
end

title_list = ["X","Y","Z","Rx","Ry","Rz"];
for i=3:5
    figure
    data = 100 * error_relative(i,:);
    histogram(data,100)
    title("Relative error distribution in " + title_list(i))
    xlabel("Error percentage (%)")
end

figure
histogram(tau_inertia_frame(3,:),100)
title("Distribution of collected data in Z")
xlabel("Force (N)")
ylabel("Number")
f = gcf;
exportgraphics(f,'Distribution_Z.png','Resolution',300)

figure
histogram(tau_inertia_frame(4,:),100)
title("Distribution of collected data in Rx")
xlabel("Force (N)")
ylabel("Number")
f = gcf;
exportgraphics(f,'Distribution_Rx.png','Resolution',300)


theta_0_calibrate = [theta_est(1,1) theta_est(5,1) theta_est(9,1) theta_est(13,1) theta_est(17,1)]';
fprintf("The net bouyant params (five parameters) abosolute error \n ");
theta_0_error = theta_0_calibrate-theta_0_calculate
fprintf("The net bouyant params (five parameters) relative errors \n ");
theta_0_relative = theta_0_error./theta_0_calculate

theta_est
% % 
% q_i = [0,0,0,0]';
% eta_i = [0,0,0, 0.1,0.2,0.3];
% Y_i = Regressor_Girona500_inertia_frame(eta_i,q_i,GironaPARAMS);
% tau_0 = Y_i*theta_est
% contribution_vehicle = Y_i(:,1:4)*theta_est(1:4)
% contribution_L1 = Y_i(:,5:8)*theta_est(5:8)
% contribution_L2 = Y_i(:,9:12)*theta_est(9:12)
% contribution_L3 = Y_i(:,13:16)*theta_est(13:16)
% contribution_L4 = Y_i(:,17:20)*theta_est(17:20)
% 
% 
% q_i = [0,0,0,0]';
% eta_i = [0,0,0, 0,0.1,0];
% Y_i = Regressor_Girona500(eta_i,q_i,GironaPARAMS);
% tau_0 = Y_i*theta_est;
% tau_0'
% Phi = Regressor_stupid(Rpy2Rot(eta_i(4:6)))
% gv = Phi * theta_est(1:4)
% 
% q_i = [0,0,0,0]';
% eta_i = [0,0,0, 0,0.0,0];
% Y_i = Regressor_Girona500(eta_i,q_i,GironaPARAMS);
% tau_0 = Y_i*theta_est;
% tau_0'
% Phi = Regressor_stupid(Rpy2Rot(eta_i(4:6)))
% gv = Phi * theta_est(1:4)
% 
% q_i = [0,0.1,0,0]';
% eta_i = [0,0,0, 0.1,0.3,0.2];
% Y_i = Regressor_Girona500_inertia_frame(eta_i,q_i,GironaPARAMS);
% tau_0 = Y_i*theta_est;
% tau_0'
