function rpy = Quat2Rpy(q)
% quat2rpy 将四元数 [x y z w] 转换为欧拉角 [roll pitch yaw]
%
% 输入:
%   q : 1x4 或 Nx4 四元数数组 [x y z w]
%
% 输出:
%   rpy : Nx3 欧拉角矩阵 [roll pitch yaw] (单位: 弧度)
%
% 欧拉角采用旋转顺序: ZYX (yaw-pitch-roll)

    % --- 检查输入 ---
    if size(q,1) ~= 4
        error('输入四元数必须为 4*n 或 4*1 形式 [x y z w]');
    end

    q = q';

    % --- 归一化四元数 ---
    q = q ./ vecnorm(q, 2, 2);

    x = q(:,1); y = q(:,2); z = q(:,3); w = q(:,4);

    % --- 欧拉角计算 ---
    % roll (x轴旋转)
    sinr = 2 .* (w .* x + y .* z);
    cosr = 1 - 2 .* (x.^2 + y.^2);
    roll = atan2(sinr, cosr);

    % pitch (y轴旋转)
    sinp = 2 .* (w .* y - z .* x);
    pitch = asin(max(min(sinp,1),-1)); % 防止数值超界

    % yaw (z轴旋转)
    siny = 2 .* (w .* z + x .* y);
    cosy = 1 - 2 .* (y.^2 + z.^2);
    yaw = atan2(siny, cosy);

    % --- 输出 ---
    rpy = [roll, pitch, yaw]';
end
