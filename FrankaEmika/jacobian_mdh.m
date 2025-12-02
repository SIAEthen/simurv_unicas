function J = jacobian_mdh(MDH)
    %JACOBIAN_MDH Summary of this function goes here
    %   Detailed explanation goes here
    n = size(MDH,1);
    % 预分配存储每个连杆坐标系的变换
    z = zeros(3, n);
    p = zeros(3, n+1);
    % 初始坐标系
    T_i_0 = eye(4);
    p(:,1) = [0;0;0];
    % ---- Forward Kinematics ----
    for i = 1:n
        TT = TMDH(MDH(i,1),MDH(i,2),MDH(i,3),MDH(i,4));
        T_i_0 = T_i_0 * TT;
        % 提取 zi-1 和 pi-1
        Ri0 = T_i_0(1:3,1:3);
        z(:,i) = Ri0(:,3);
        p(:,i+1) = T_i_0(1:3,4);
    end
    % 末端位置
    p_end = p(:,end);
    % ---- Compute Jacobian ----
    Jv = zeros(3,n);
    Jw = zeros(3,n);
    for i = 1:n
        % Revolute joint：
        Jv(:,i) = cross(z(:,i), p_end - p(:,i));
        Jw(:,i) = z(:,i);
    end
    J = [Jv; Jw];
end

