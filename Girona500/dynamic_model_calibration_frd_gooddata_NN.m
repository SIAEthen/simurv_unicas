close all
% load data
data = load("Girona500/high_quality_data.txt");
q = data(:,8:11)';
tau_body_frame = data(:,12:17)';
% for fix bugs
% tau_body_frame(4:6,:) = -1 * tau_body_frame(4:6,:);

eta = data(:,18:24)';
deta = data(:,25:30);

num_of_data = 100;
num_of_test = size(data,1)-num_of_data;
eta_new = 0 * eta;


% 假设我们有 1000 组样本
N = 500;

% 输入数据 (8×N)
X = rand(8, N);

% 输出目标 (6×N)
Y = [sin(sum(X(1:3,:)));  % 一些非线性关系
     cos(X(4,:) + X(5,:));
     X(6,:).*X(7,:);
     X(8,:).^2;
     X(1,:).*X(8,:);
     exp(-X(2,:))];

% 创建一个前馈神经网络
hiddenLayerSize = [32 16];   % 两层隐藏层
net = feedforwardnet(hiddenLayerSize);

% 设置网络参数
net.trainFcn = 'trainlm';     % Levenberg-Marquardt，适合小中规模数据
net.performFcn = 'custom_loss';       % 均方误差
net.divideParam.trainRatio = 0.7;
net.divideParam.valRatio = 0.15;
net.divideParam.testRatio = 0.15;

% 训练网络
net = train(net, X, Y);

% 测试网络
Y_pred = net(X);

% 计算误差
error = Y - Y_pred;
fprintf('平均绝对误差: %.4f\n', mean(abs(error(:))));
figure

% 绘制对比图（可视化第一个输出维度）
figure;
plot(Y(1,:), 'b'); hold on;
plot(Y_pred(1,:), 'r--');
legend('True','Predicted');
title('Output dimension 1');
