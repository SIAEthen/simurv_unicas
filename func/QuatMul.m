function q = QuatMul(q1, q2)
% quatMultiply 计算两个四元数的乘积（q = q1 * q2）
% 四元数顺序: [x y z w]
%
% 输入:
%   q1, q2 - 1x4 向量 [x y z w]
%
% 输出:
%   q - 1x4 向量 [x y z w]

% 提取分量
x1 = q1(1); y1 = q1(2); z1 = q1(3); w1 = q1(4);
x2 = q2(1); y2 = q2(2); z2 = q2(3); w2 = q2(4);

% 四元数乘法（右乘：q = q1 * q2）
x =  w1*x2 + x1*w2 + y1*z2 - z1*y2;
y =  w1*y2 - x1*z2 + y1*w2 + z1*x2;
z =  w1*z2 + x1*y2 - y1*x2 + z1*w2;
w =  w1*w2 - x1*x2 - y1*y2 - z1*z2;

q = [x y z w];
q = q/norm(q,2);
end

