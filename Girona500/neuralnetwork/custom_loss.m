function perf = custom_loss(Y, T)
%CUSTOM_LOSS Summary of this function goes here
%   Detailed explanation goes here
% 自定义损失函数，支持 MATLAB 调用 defaultParam

    % === 处理 MATLAB 查询默认参数的情况 ===
    if nargin == 1 && ischar(Y) && strcmp(Y, 'defaultParam')
        perf = struct;   % 必须返回结构体
        return
    end

    % === 正常的损失计算 ===
%     perf = mean(abs(T - Y), 'all') + 0.5 * mean((T - Y).^2, 'all');
    perf = 0* mean((T - Y).^2);
end


