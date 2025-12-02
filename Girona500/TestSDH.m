
digits(5);  %将 MATLAB 的符号计算精度设置为 5 位有效数字
% a alpha d theta
a = [0.1065,0.23332,0.103,0]';
alpha = [-pi/2,0,pi/2,0]';
d = [0,0,0,0.201]';
theta = [0,0,0,0]';



%% DH

offset_i = theta;

L(1)=Link([theta(1),d(1),a(1),alpha(1),0,offset_i(1)],'standard');
L(2)=Link([theta(2),d(2),a(2),alpha(2),0,offset_i(2)],'standard');
L(3)=Link([theta(3),d(3),a(3),alpha(3),0,offset_i(3)],'standard');
L(4)=Link([theta(4),d(4),a(4),alpha(4),0,offset_i(4)],'standard');
a_i     = a';
alpha_i = alpha';
d_i     = d';
theta_i = theta';
Arm = SerialLink(L);
Arm.name = ("Big Arm");
Arm.teach()
