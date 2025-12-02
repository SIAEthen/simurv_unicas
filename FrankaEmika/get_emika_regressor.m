
% ------------------------------------------------------------
% 自动从 Panda 的 Modified DH 参数生成重力项 G(q)
% ------------------------------------------------------------

clear; clc;

%% ====== 1. 定义符号变量 ======
n = 7;  % Panda 7DoF
syms q1 q2 q3 q4 q5 q6 q7 real
q = [q1; q2; q3; q4; q5; q6; q7];

g0 = sym([0,0,-0.98]');

% params of one joint is [mi mxi myi mzi]

%% ====== 2. 填入 Panda 的 MDH 参数（请你替换成你自己版本） ======
% MDH ：alpha(i), a(i), theta(i), d(i)
% theta(i) = q(i) (转动关节), 固定偏置写入 offset
%
% 下表是常见 Panda MDH 参数（你需要确认你的版本是否一致）
%
%    i     alpha_i         a_i        d_i             theta_i
%  ---------------------------------------------------------------
%    1    -pi/2            0          0.333           q1
%    2     pi/2            0          0               q2
%    3     pi/2            0.0825     0               q3
%    4    -pi/2           -0.0825     0.316           q4
%    5     pi/2            0          0               q5
%    6     pi/2            0.088      0               q6
%    7     0               0          0.107           q7
%
% 如果你有自己的表，请直接替换这些数值 ↓↓↓

% alpha = [-pi/2,  pi/2,   pi/2,  -pi/2, pi/2,  pi/2,  0];
% a     = [0,      0,      0.0825, -0.0825, 0, 0.088, 0];
% d     = [0.333,  0,       0,      0.316,   0, 0,     0.107];
% theta = [q1, q2, q3, q4, q5, q6, q7];  % all revolute

a = [0,0,0,0.0825,-0.0825,0.0,0.088,0.0]';
alpha = [0,-pi/2,pi/2,pi/2,-pi/2,pi/2,pi/2,0.0]';
d = [0.333,0.0,0.316,0.0,0.384,0.0,0.0,0.107]';
theta = [q1, q2, q3, q4, q5, q6, q7]';

%% ====== 3. 定义各连杆的 COM（你需要填自己的） ======
% COM_i expressed in link i frame
% 以下数值仅示例，请用你自己的真实 Panda 参数替换！



%% ====== 4. MDH 变换
Ti = sym(zeros(4,4,n));
p_i_i = sym(zeros(3,n));
g0_i = sym(zeros(3,n));

T_i_0 = eye(4);
for i = 1:n
    T_i_im1 = TMDH(a(i),alpha(i),d(i),theta(i));
    T_i_0 = T_i_0 * T_i_im1;
    Ti(:,:,i) = T_i_0;
    Ri0 = T_i_0(1:3,1:3);
    p_i_i(:,i) = Ri0' * T_i_0(1:3,4);
    g0_i(:,i) = Ri0'  * g0;
end

%% ====== 5. 势能 U = Σ m_i * g * h_i
% g = [0,0,-g]


% params 4n x 1  m mx my mz
% beta_P n x 4n

beta_P = sym(zeros(n,4));
for i = 1:n
    beta_P(i,:) = [-g0_i(:,i)'*p_i_i(:,i), -g0_i(:,i)'];
end

regressor = sym(zeros(n,4*n));
for i=1:n
    for j=1:n
        yij = diff(beta_P(j,:),q(i));
        regressor(i,4*(j-1)+1:4*j) = yij;
    end
end





% Generate function from a symbolic expression for the regressor
matlabFunction(regressor,'File','FrankaEmika/G_regressor_emika',...
               'Vars',{q});
