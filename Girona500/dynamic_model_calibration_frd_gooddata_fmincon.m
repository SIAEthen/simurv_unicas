



% tau_inertia_frame = 0*tau_body_frame;
% % transform tau into inertia frame
% for i=1:size(tau_inertia_frame,2)
%     eta_i = eta_i(:,i);
%     R_B_I = Quat2Rot(eta(4:7));
%     U_B_I = U_mat(R_B_I,[0,0,0]');
%     tau_inertia_frame(:,i) = U_B_I * tau_body_frame(:,i);
% end

Y = zeros(6*num_of_data_fmincon,20);
meas = zeros(6*num_of_data_fmincon,1);

weight_matrix = diag([1,1,1,1,1,1]);
for i=1:num_of_data_fmincon
    q_i = q(:,i);
    eta_i = eta(:,i);
    Y_i = Regressor_Girona500(eta_i,q_i,GironaPARAMS) ;% with fixed regressor
    meas_i = tau_body_frame(:,i);
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
% lb = theta0 - [5,100,100,100, 5,100,100,100, 5,100,100,100, 5,100,100,100, 5,100,100,100]';
% ub = theta0 + [5,100,100,100, 5,100,100,100, 5,100,100,100, 5,100,100,100, 5,100,100,100]';
% lb = theta0 - [50,20,20,20,50,20,20,20,50,20,20,20,50,20,20,20,50,20,20,20]';
% ub = theta0 + [50,20,20,20,50,20,20,20,50,20,20,20,50,20,20,20,50,20,20,20]';
% lb = theta0 - [10,150,150,150,   10,50,50,50,  10,50,50,50, 10,50,50,50, 10,50,50,50]';
% ub = theta0 + [10,150,150,150, 10,50,50,50,  10,50,50,50, 10,50,50,50, 10,50,50,50]';
lb = theta0 - [50,50,50,500,    10,5,5,5,  10,5,5,5, 10,5,5,5,    10,5,5,5]';
ub = theta0 + [50,50,50,500,    10,5,5,5,  10,5,5,5, 10,5,5,5,    10,5,5,5]';
theta_fmincon = fmincon(fun, theta0, A, b, Aeq, beq, lb, ub, [], options);

% predict = zeros(6,num_of_test_fmincon);
% measure = zeros(6,num_of_test_fmincon);
% error = zeros(6,num_of_test_fmincon);
% error_relative = zeros(6,num_of_test_fmincon);
% for i=num_of_data_fmincon+1:size(data,1)
%     q_i = q(:,i);
%     eta_i = eta(:,i);
%     Y_i = Regressor_Girona500(eta_i,q_i,GironaPARAMS) ;% with fixed regressor
%     
%     meas_i = tau_body_frame(:,i);
%     pred_i = Y_i*theta_fmincon;
%     error_i = pred_i - meas_i;
%     error(:,i-num_of_data_fmincon) = error_i;
%     predict(:,i-num_of_data_fmincon) = pred_i;
%     error_relative(:,i-num_of_data_fmincon) = error_i./meas_i;
%     measure(:,i-num_of_data_fmincon) = (meas_i);
% end
% 
% title_list = ["X","Y","Z","Rx","Ry","Rz"];
% y_list = ["Force (N)","Force (N)","Force (N)","Torque (Nm)","Torque (Nm)","Torque (Nm)"];
% x_list = ["Simulations","Simulations","Simulations","Simulations","Simulations","Simulations"];
% for i=1:6
%     figure
%     x = 1:size(predict,2);
%     plot(x,predict(i,:), x,measure(i,:));
%     legend(["predict","simulation"]);
%     title(title_list(i))
%     xlabel(x_list(i))
%     ylabel(y_list(i))
% end
% 
% title_list = ["X","Y","Z","Rx","Ry","Rz"];
% x_list = ["Force (N)","Force (N)","Force (N)","Torque (Nm)","Torque (Nm)","Torque (Nm)"];
% for i=1:6
%     figure
%     data = predict(i,:)-measure(i,:);
%     histogram(data,100)
%     title("Predict error distribution in "+title_list(i))
%     xlabel(x_list(i))
%     ylabel("Number")
% end
% 
% title_list = ["X","Y","Z","Rx","Ry","Rz"];
% for i=1:6
%     figure
%     data = 100 * error_relative(i,:);
%     histogram(data,100)
%     title("Relative error distribution in " + title_list(i))
%     xlabel("Error percentage (%)")
% end

% figure
% histogram(tau_body_frame(3,:),100)
% title("Distribution of collected data in Z")
% xlabel("Force (N)")
% ylabel("Number")
% f = gcf;
% exportgraphics(f,'Distribution.png','Resolution',300)

theta_0_calibrate = [theta_fmincon(1,1) theta_fmincon(5,1) theta_fmincon(9,1) theta_fmincon(13,1) theta_fmincon(17,1)]';
fprintf("The net bouyant params (five parameters) abosolute error \n ");
theta_0_error = theta_0_calibrate-theta_0_calculate;
fprintf("The net bouyant params (five parameters) relative errors \n ");
theta_0_relative = theta_0_error./theta_0_calculate;

theta_fmincon

