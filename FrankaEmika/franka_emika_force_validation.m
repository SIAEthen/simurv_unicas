get_emika_dh_params
mdh = emika_dh_parameters; %MDH
% 读取 CSV 数据（包括 header）
% path = "/home/sia/Franka_Emika_data/exp3_validation/xinhui_exp2.bag_csv/";
% path = "/home/sia/Franka_Emika_data/exp2_validation/xinhui_exp01.bag_csv/";
path = "/home/sia/Franka_Emika_data/xinhui_exp3/xinhui_exp3.bag_csv/"; % 20251201 data1
filename1 = path+ "_franka_state_controller_F_ext.txt";
filename2 = path+ "_joint_states.txt";

opts = detectImportOptions(filename1, 'Delimiter', ',');
T = readtable(filename1, opts);
t1 = (T.x_time-T.x_time(1))/1e9;
wrench_measured = [T.field_wrench_force_x,T.field_wrench_force_y,T.field_wrench_force_z,...
                    T.field_wrench_torque_x,T.field_wrench_torque_y,T.field_wrench_torque_z]';

opts = detectImportOptions(filename2, 'Delimiter', ',');
T2 = readtable(filename2, opts);
t2 = (T2.x_time-T2.x_time(1))/1e9;
q = [T2.field_position0,T2.field_position1,T2.field_position2,T2.field_position3,T2.field_position4,T2.field_position5,T2.field_position6]';
tau = [T2.field_effort0,T2.field_effort1,T2.field_effort2,T2.field_effort3,T2.field_effort4,T2.field_effort5,T2.field_effort6]';

num = size(q,2);

S.x_r    = [0; 0; 0];
S.dx_r   = [0; 0; 0];
S.ddx_r  = [0; 0; 0];      % will be overwritten
S.xd   = [0; 0; 0];
S.dxd  = [0; 0; 0];
S.dt   = 0.03;

F_e_mea = [];
F_e_mea_2 = [];
F_e_pre = [];
F_e_pre_2 = [];
for i=1:num
    q_i = q(:,i);
%     q_i(1) = 0;
%     q_i = rand(7,1);
    q_i(7) = q_i(7) - 0.776;
    if abs(q_i(2))>0.1
        mdh_i =  [mdh(1:7,1:3) mdh(1:7,4) + q_i];
        
        J3_i = jacobian_mdh(mdh_i(1:3,:));
        J4_i = jacobian_mdh(mdh_i(1:4,:));
        J5_i = jacobian_mdh(mdh_i(1:5,:));
        J6_i = jacobian_mdh(mdh_i(1:6,:));
        J7_i = jacobian_mdh(mdh_i(1:7,:));
        
        tau_mea = tau(1:7,i);
        g_i = get_emika_gravity(q_i(1:7)); % it is the same with same joint configuration
        tau_e = tau_mea - g_i;

        
        T_3_0 = DirectKinematics_mdh(mdh_i(1:3,:));
        T_4_0 = DirectKinematics_mdh(mdh_i(1:4,:));
        T_5_0 = DirectKinematics_mdh(mdh_i(1:5,:));
        T_6_0 = DirectKinematics_mdh(mdh_i(1:6,:));
        T_7_0 = DirectKinematics_mdh(mdh_i(1:7,:));

%         f_e_pre = T_3_0(1:3,1:3)' * pinv(J3_i(1:3,:)')*tau_e(1:3);
%         f_e_pre = pinv(J4_i(1:3,:)')*tau_e(1:4);
%         f_e_pre = pinv(J5_i(1:3,:)')*tau_e(1:5);
%         f_e_pre = pinv(J6_i(1:3,:)')*tau_e(1:6);
        
        f_e_mea = pinv(J7_i')*tau_e;
        tau_e_notorque = tau_e - J7_i(4:6,:)'*f_e_mea(4:6);
%         f_e_pre = pinv(J4_i(1:3,:)')*tau_e_notorque(1:4);
%         f_e_pre = pinv(J5_i(1:3,:)')*tau_e_notorque(1:5);
        f_e_pre = pinv(J7_i(1:3,1:4)')*tau_e_notorque(1:4);
        f_e_pre_2 = pinv(J7_i(1:3,1:4)')*tau_e(1:4);
        
        F_e_pre = [F_e_pre f_e_pre];
        F_e_pre_2 = [F_e_pre_2 f_e_pre_2];
        F_e_mea = [F_e_mea f_e_mea(1:3)];

        F_e_mea_2 = [F_e_mea_2 T_7_0(1:3,1:3) * wrench_measured(1:3,20*i)];
    end
     
end
close all
% error1 = F_e_mea(1:3,:)-F_e_pre;
% 
% error2 = F_e_mea(1:3,:)-F_e_pre_2;

prefix1 = "exp2";

figure
plot(t2,F_e_mea_2)
legend("1","2","3")
xlabel("time (s)")
ylabel("F/T (N,Nm)")
grid
title("F_e in base frame")
% ax = gcf;
% ax.Units = "centimeters";
% ax.Position = [20,10,8,6];
% print(ax, '-dpng', '-r300', prefix1+'_mea_wrench.png')

% legend("1","2","3")
figure
plot(t2,F_e_mea)
legend("1","2","3")
xlabel("time (s)")
ylabel("F/T (N,Nm)")
grid
ax = gcf;
ax.Units = "centimeters";
ax.Position = [20,10,8,6];
print(ax, '-dpng', '-r300', prefix1+'_mea_wrench.png')

figure
plot(t2,F_e_mea_2 - F_e_mea)
legend("1","2","3")
xlabel("time (s)")
ylabel("F/T (N,Nm)")
grid


% figure
% plot(t2,F_e_pre)
% legend("1","2","3")
% xlabel("time (s)")
% ylabel("F (N)")
% grid
% ax = gcf;
% ax.Units = "centimeters";
% ax.Position = [20,10,8,6];
% print(ax, '-dpng', '-r300', prefix1+'_pre_fe.png')
% 
% 
% figure
% plot(t2,F_e_mea(1:3,:)-F_e_pre)
% legend("1","2","3")
% xlabel("time (s)")
% ylabel("F (N)")
% grid
% ax = gcf;
% ax.Units = "centimeters";
% ax.Position = [20,10,8,6];
% print(ax, '-dpng', '-r300', prefix1+'_pre_fe_err.png')
% 
% figure
% plot(t2,F_e_pre_2)
% legend("1","2","3")
% xlabel("time (s)")
% ylabel("F (N)")
% grid
% ax = gcf;
% ax.Units = "centimeters";
% ax.Position = [20,10,8,6];
% print(ax, '-dpng', '-r300', prefix1+'_pre_fe2.png')
% 
% figure
% plot(t2,F_e_mea(1:3,:)-F_e_pre_2)
% legend("1","2","3")
% xlabel("time (s)")
% ylabel("F (N)")
% grid
% ax = gcf;
% ax.Units = "centimeters";
% ax.Position = [20,10,8,6];
% print(ax, '-dpng', '-r300', prefix1+'_pre_fe2_err.png')
% 

