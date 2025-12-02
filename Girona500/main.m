close all
% start calculate
calculate_sdh
%prepare init data
% base parameters of vehicle and each link calculation
rho = 1000; % water density
g = 9.8;
mass = [144.681+3.825  0.845 5.734 1.516 0.308+0.579]'; % kg
volum = [147417.3+3824.7 844.7 5733.7 1515.9 307.5+579.3]'/100.0/100.0/100.0;% cm3
theta_0_calculate = g*(mass - rho*volum); % -26.8128 0.0029 0.0029 0.0010 0.0020
theta0 = zeros(20,1);
theta0(1,1) = theta_0_calculate(1,1); % net bouyant > 0, W-B <0
theta0(4,1) = 300; % net bouyant > 0, W-B <0
% theta0(5,1) = theta_0_calculate(2,1);
% theta0(9,1) = theta_0_calculate(3,1);
% theta0(13,1) = theta_0_calculate(4,1);
% theta0(17,1) = theta_0_calculate(5,1);

% load data
data = load("Girona500/high_quality_data.txt");
q = data(:,8:11)';
tau_body_frame = data(:,12:17)';
eta = data(:,18:24)';
deta = data(:,25:30);
num_of_data_fmincon = 20;
num_of_data_nn = 1000;
num_of_validation = 1000; % use the last 1000 data for validation
fprintf("Vehicle mean abs velocity NED in 6 DOF (Units: m/s or rad/s) %.4f  %.4f  %.4f  %.4f  %.4f  %.4f \n",...
    mean(abs(deta(1,:))),mean(abs(deta(2,:))),mean(abs(deta(3,:))),...
    mean(abs(deta(4,:))),mean(abs(deta(5,:))),mean(abs(deta(6,:))) ...
    );
fprintf("Vehicle max abs velocity NED in 6 DOF (Units: m/s or rad/s) %.4f  %.4f  %.4f  %.4f  %.4f  %.4f \n",...
    max(abs(deta(1,:))),max(abs(deta(2,:))),max(abs(deta(3,:))),...
    max(abs(deta(4,:))),max(abs(deta(5,:))),max(abs(deta(6,:))) ...
    );


eta_valid = eta(:,size(eta,2)-1000:size(eta,2));
q_valid = q(:,size(eta,2)-1000:size(eta,2));
tau_body_frame_valid = tau_body_frame(:,size(eta,2)-1000:size(eta,2));

% nn method
rpy = 0*eta(1:3,:);
for i=1:size(rpy,2)
    rpy(:,i) = Quat2Rpy(eta(4:7,i));
end

input_all = [rpy(1:2,:); q]';
output_all = tau_body_frame';
X_train = input_all(1:num_of_data_nn,:);
Y_train = output_all(1:num_of_data_nn,:);

dynamic_model_calibration_frd_gooddata_fmincon

dynamic_model_calibration_frd_gooddata_NN2

cal_performance(theta0,theta_fmincon,net,eta_valid,q_valid,GironaPARAMS,tau_body_frame_valid);

% 测试
% Y_predict = predict(net, X_val);
% error = Y_predict - Y_val;
% for i =1:6
% figure
% histogram(error(:,i))
% end


