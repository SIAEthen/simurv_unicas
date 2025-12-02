function Y = Regressor_Girona500_inertia_frame(eta,q,GironaPARAMS)
%REGRESSOR_GIRONA500 Summary of this function goes here
%   Detailed explanation goes here
% eta = zeros(6,1);
% q = zeros(4,1);
n = size(q,1);
DH_raw = GironaPARAMS.DH;
DH = [DH_raw(:,1:3) DH_raw(:,4)+q];
T_0_B = GironaPARAMS.T_0_B;
R_0_B  = T_0_B(1:3,1:3);
r_B0_B = T_0_B(1:3,4);
if(size(eta,1)==7)
    RBI = Quat2Rot(eta(4:7));        % from vehicle-fixed to inertial
else
    RBI = Rpy2Rot(eta(4:6));        % from vehicle-fixed to inertial
end
R0I = RBI*T_0_B(1:3,1:3);     % from zero to intertial
RiI = (zeros(3,3,n));
RiI(:,:,1) = R0I*Rot_dh(DH(1,2),DH(1,4));
% manipulator cycle
for i=2:n
    RR = Rot_dh(DH(i,2),DH(i,4));
    RiI(:,:,i) = RiI(:,:,i-1)*RR;
end

% Compute all the consecutives U
% U 可以像旋转矩阵一样连着乘
% 
% U_B_I = U_mat(RBI,[0,0,0]');
% U_0_B = U_mat(R_0_B, r_B0_B);
% U_0_I = U_B_I * U_0_B;
% Y = (zeros(6,4*(n+1)));           % total regressor
% Y(:,1:4) = Regressor_stupid(RBI); % this force is already in body frame
% Y(:,1:4) = U_B_I * Y(:,1:4); % this force is already in body frame
% 
% U_i_I = U_0_I;
% for i = 1:n
%     T = Homogeneous_dh(DH(i,:));
%     UU = U_mat(T(1:3,1:3),T(1:3,4));
%     U_i_I = U_i_I*UU;
%     Y(:,1+4*i:4*(i+1)) = U_i_I*Regressor_stupid(RiI(:,:,i));
% end

U_B_I = U_mat(RBI,[0,0,0]');

U_0_B = U_mat(R_0_B, r_B0_B);
Y = (zeros(6,4*(n+1)));           % total regressor
Y(:,1:4) = Regressor_stupid(RBI); % this force is already in body frame


U_i_B = U_0_B;
for i = 1:n
    T = Homogeneous_dh(DH(i,:));
    UU = U_mat(T(1:3,1:3),T(1:3,4));
    U_i_B = U_i_B*UU;
    Y(:,1+4*i:4*(i+1)) = U_i_B*Regressor_stupid(RiI(:,:,i));
end

Y = U_B_I * Y;
end

