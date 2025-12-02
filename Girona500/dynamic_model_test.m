clear all
calculate_sdh

eta = zeros(6,1);
q = zeros(4,1);
Y = Regressor_Girona500(eta,q,GironaPARAMS) ;% with fixed regressor
Y(:,17:20)*[1,0,0,0]'
Y(:,13:16)*[1,0,0,0]'
Y(:,1:4)*[1,0,0,0]'



W = 10;
B = 40;
WL = 5;
BL = 0;

xg = [0,0,0]';
xb = [-0.001,0,-1]';

theta0 = zeros(20,1);
theta0(1,1) = W-B; % net bouyant > 0, W-B <0
theta0(2:4,1) = W*xg - B*xb;


theta0(5,1) = 0;
theta0(9,1) = 0;
theta0(13,1) = 0;
theta0(17,1) = 0;

eta = zeros(6,1);
q = zeros(4,1);
Y = Regressor_Girona500(eta,q,GironaPARAMS) ;% with fixed regressor
% gRB = Y*theta0
% gv = Y(:,1:4)*theta0(1:4)

Phi = Regressor_stupid(Rpy2Rot(eta(4:6)));
gv_2 = Phi*theta0(1:4)

eta = zeros(6,1);
eta(4:6) = [0,0.2,0]';
q = zeros(4,1);
Y = Regressor_Girona500(eta,q,GironaPARAMS) ;% with fixed regressor
% gRB = Y*theta0
% gv = Y(:,1:4)*theta0(1:4)

Phi = Regressor_stupid(Rpy2Rot(eta(4:6)));
gv_2 = Phi*theta0(1:4)