p = 0.5;

rpm = -1000*p;
thrust = 0.64;
D = 0.18;
coeff = 0.52;
rho = 1000;
n = rpm/60;

c1 = coeff*rho*D^4;
F = c1*n*abs(n)


Kq0 = 0.05;
T = Kq0 * rho *n*abs(n) * D^5

% rpy1 = [0,0.1,0.5*pi]';
% quatd = Rpy2Quat(rpy1);
% 
% rpy2 = [0,0.12,0.5*pi]';
% quatc = Rpy2Quat(rpy2);
% quatc_inv = [-1*quatc(1:3);quatc(4)];
% R_B_I = Rpy2Rot(rpy2);
% e = quatMultiply(quatd,quatc_inv)
% R_B_I' * e(1:3)'
