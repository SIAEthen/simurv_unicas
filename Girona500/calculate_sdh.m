data = readstruct("/home/sia/unicas_ws/src/cola2_stonefish/scenarios/girona500_eca5emicro.scn",'FileType','xml');
[j0,j1,j2,j3,j4] = data.robot.joint.origin;
joint_information = [j0,j1,j2,j3,j4];
xyz = zeros(3,5);
rpy = zeros(3,5);
for i=1:5
    xyz_i = str2num(joint_information(i).xyzAttribute);
    rpy_i = str2num(joint_information(i).rpyAttribute);
    xyz(:,i) = xyz_i';
    rpy(:,i) = rpy_i';
end
ratation_axis = zeros(3,4);
ratation_axis(3,1) = 1;
ratation_axis(2,2) = 1;
ratation_axis(2,3) = 1;
ratation_axis(3,4) = 1;
% rpy0 = str2num(j0.xyzAttribute)
% R_0_B = Rpy2Rot()

R_0_B = Rpy2Rot(rpy(:,1));
t_0_B = [0.74 -0.011 0.3856]';
% t_0_B = [1,0,0.2]';
T_0_B = [R_0_B,t_0_B;0,0,0,1];

% a alpha d theta
% a = [0,0.1065,0.23332,0.103]';
% alpha = [0,-pi/2,0,pi/2]';
% d = [0,0,0,0]';
% theta = [0,0,0,0]';

a = [0.1065,0.23332,0.103,0]';
alpha = [-pi/2,0,pi/2,0]';
d = [0,0,0,0.201]';
theta = [0,0,0,0]';

SDH = [a,alpha,d,theta];
% try to draw a picture of the vehicle, saved figure in this folder
test_DH = SDH;
figure
DrawUVMS(zeros(6,1),test_DH,PARAM) % here param sould be loaded by run simurv and select params
GironaPARAMS.DH = SDH;
GironaPARAMS.T_0_B = T_0_B;
clear SDH theta a alpha d ratation_axis j* J* data i R* r*  T* x*
