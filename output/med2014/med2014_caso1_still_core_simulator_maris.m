% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

%------------------------
% simulation parameters 
%------------------------
KinOnly   = 0;  % kinematic simulation only, controller needs to provide reference velocities
                % if null the simulation is dynamic and the controller
                % outputs the forces/moments/torques
Graphics  = 2;	% provide or not graphics in the end of the simulation

t_f = 20;       % final simulation time [s]
Ts  = 0.001;     % sampling time [s]
t   = 0:Ts:t_f;	% time vector [s]
npti = length(t);

% Definition of the variables
n   = size(DH,1);
eta           = zeros(6,npti); %eta(6,1) = 15/180*pi;
q             = zeros(n,npti); q (:,1) = DH(:,4);
zita          = zeros(6+n,npti);
eta_ee1       = zeros(3,npti);
eta_ee_quat   = zeros(4,npti);
dzita = zeros(6+n,npti);
tau   = zeros(6+n,npti);

% ---------------
% task variables
% ---------------
ma = 6;     % dim of task a
mb = 2;     % dim of task b
mc = 7;     % dim of task c
sigma_tilde_a = zeros(ma,npti);
sigma_tilde_b = zeros(mb,npti);
sigma_tilde_c = zeros(mc,npti);

R_d = [1 0 0; 0 1 0; 0 0 1]; % desired end-effector orientation
eta2_quat_d = Rot2Quat([1 0 0; 0 1 0; 0 0 1]); % desired vehicle orientation
obj_pos  = [1.6 -0.2 2]';   % object position
obj_size = 0.2;             % object size
p_obst = [1.4 0 1.5]';    % obstacle position
ee_safety_dist = .2;      % end-effector safety distance from obstacle
%W = diag([50 50 50 25 25 25 1*ones(1,n)]);invW=inv(W);

% ---------------
% control variables
% ---------------
eta_d         = zeros(6,npti);
q_d           = zeros(n,npti); q_d (:,1) = DH(:,4);
zita_d        = zeros(6+n,npti);
eta_ee1_d     = zeros(3,npti);
eta_ee_quat_d = zeros(4,npti);
CONTROLLER_TYPE = 1;
if (CONTROLLER_TYPE==1)
    % simple adaptive at the vehicle + PID at joints
    fprintf('\nController: 1 simple adaptive at the vehicle + PID at joints');
    gamma_v       = zeros(9,npti);
    gamma_v(6,1)  = 157.9282;  % weight with manipulator vertical
    gamma_q       = zeros(n,npti);
elseif (CONTROLLER_TYPE==2)
    % arm-based-adaptive at the vehicle + PID at joints
    fprintf('\nController: 2 arm-based-adaptive at the vehicle + PID at joints');
    gamma_v       = zeros(9*(n+1),npti);
    gamma_v(6,1)  = 157.9282;  % weight with manipulator vertical
    gamma_q       = zeros(n,npti);
elseif (CONTROLLER_TYPE==3)
    % arm-based-adaptive at the vehicle + PID at joints
    % the momentum external disturbance only on the vehicle and not on the
    % single joints
    fprintf('\nController: 3 arm-based-adaptive at the vehicle + PID at joints (regressor version 2)');
    gamma_v          = zeros(9+6*n,npti);
    gamma_v(1:3,1)   = [0 0 -441.45]'; % first moment grav+buoy. vehicle
    gamma_v(6,1)     = 299.5;          % buoyancy of the vehicle alone
    gamma_v(10:12,1) = [0 2.7147 0]';  % link 1. first moment grav+buoy -9.81*(PARAM.m(i)*PARAM.r_c(:,i)-PARAM.rho*PARAM.delta(i)*PARAM.r_b(:,i))
    gamma_v(15,1)    = -27.1472;       % link 1. buoyancy -9.81*(PARAM.m(1)-PARAM.rho*PARAM.delta(1))
    gamma_v(16:18,1) = [0 0 -.8584]';  % link 2.
    gamma_v(21,1)    = -28.6122;       % link 2.
    gamma_v(22:24,1) = [0 .4909 0]';   % link 3.
    gamma_v(27,1)    = -24.7229;       % link 3.
    gamma_v(28:30,1) = [0 0 -.6895]';  % link 4.
    gamma_v(33,1)    = -22.9832;       % link 4.
    gamma_v(34:36,1) = [0 -1.8635 0]'; % link 5.
    gamma_v(39,1)    = -9.0269;        % link 5.
    gamma_v(40:42,1) = [0 0 -0.5974]'; % link 6.
    gamma_v(45,1)    = -19.9117;       % link 6.
    gamma_v(46:48,1) = [0 0 -1.3751]'; % link 7.
    gamma_v(51,1)    = -9.1676;        % link 7.
    gamma_v_min(1:9)   = [-500 -500 -500 -400 -400 -400 -400 -400 -400]';
    gamma_v_min(10:15) = [-3 -3 -3 -3 -3 -30]';
    gamma_v_min(16:21) = [-3 -3 -3 -3 -3 -30]';
    gamma_v_min(22:27) = [-3 -3 -3 -3 -3 -30]';
    gamma_v_min(28:33) = [-3 -3 -3 -3 -3 -30]';
    gamma_v_min(34:39) = [-3 -3 -3 -3 -3 -30]';
    gamma_v_min(40:45) = [-3 -3 -3 -3 -3 -30]';
    gamma_v_min(46:51) = [-3 -3 -3 -3 -3 -30]';
    gamma_v_max(1:9)   = [500 500 500 400 400 400 400 400 400]';
    gamma_v_max(10:15) = [ 3  3  3  3  3 -24]';
    gamma_v_max(16:21) = [ 3  3  3  3  3 -24]';
    gamma_v_max(22:27) = [ 3  3  3  3  3 -18]';
    gamma_v_max(28:33) = [ 3  3  3  3  3 -15]';
    gamma_v_max(34:39) = [ 3  3  3  3  3 -5]';
    gamma_v_max(40:45) = [ 3  3  3  3  3 -15]';
    gamma_v_max(46:51) = [ 3  3  3  3  3 -5]';
    gamma_q          = zeros(n,npti);
elseif (CONTROLLER_TYPE==9)
    % free floating system
end



DrawSnap(eta(:,1),[DH(:,1:3) q(:,1)],PARAM,2.5);
%DrawCube(obj_pos,obj_size);
%DrawSphere(p_obst,ee_safety_dist);
axis auto
drawnow

%------------------------
% main loop - start
%------------------------
t_start_sim = clock();
for i=1:npti
    % current end-effector configuration
    T = DirectKinematics(eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
    eta_ee1(:,i)     = T(1:3,4);
    eta_ee_quat(:,i) = Rot2Quat(T(1:3,1:3));
    % desired end-effector trajectory
    if (t(i)<=10)
        eta_ee1_d(:,i) = trapezoidal2(eta_ee1(:,1),eta_ee1(:,1)-[1.2 0 0]',[.2 .3 .3]',6,t(i));
    else
        eta_ee1_d(:,i) = trapezoidal2(eta_ee1(:,1)-[1.2 0 0]',eta_ee1(:,1),[.2 .3 .3]',6,t(i)-10);
    end
    eta_ee_quat_d(:,i) = GenerateDesQuat(eta_ee_quat(:,1),R_d,.45,12,t(i));
    eta_ee_quat(:,i) = AlignQuat(eta_ee_quat_d(:,i),eta_ee_quat(:,i));
    
    % controller
    if (KinOnly==1)
        [zita_d(:,i), sigma_tilde_a(:,i), sigma_tilde_b(:,i), sigma_tilde_c(:,i)] = ...
            InverseKynematics_maris02(eta_ee1_d(:,i),eta_ee_quat_d(:,i), eta(:,i), q(:,i), DH, PARAM);
    else
        [zita_d(:,i), sigma_tilde_a(:,i), sigma_tilde_b(:,i), sigma_tilde_c(:,i)] = ...
            InverseKynematics_maris02(eta_ee1_d(:,i),eta_ee_quat_d(:,i), eta_d(:,i), q_d(:,i), DH, PARAM);
        %[zita_d(:,i), sigma_tilde_a(:,i), sigma_tilde_b(:,i), sigma_tilde_c(:,i)] = ...
        %    InverseKynematics_maris01(eta_ee1_d(:,i),eta_ee_quat_d(:,i), eta(:,i), q(:,i), DH, PARAM);
        zita_d(:,i) = 0*zita_d(:,i);
        if i<npti
            Je           = J_e(eta(4:6,i));
            eta_d(:,i+1) = eta_d(:,i) + Ts*(Je\zita_d(1:6,i));
            q_d(:,i+1)   = q_d(:,i)   + Ts*zita_d(7:6+n,i);
        end
        
        if (CONTROLLER_TYPE==1)
            [tau(:,i), gamma_v_dot, gamma_q_dot] = ...
            Control_maris01(eta(:,i), q(:,i), zita(:,i), ...
            eta_d(:,i), q_d(:,i), zita_d(:,i), gamma_v(:,i), gamma_q(:,i));
            if i<npti
                gamma_v(:,i+1) = IntegrationSat(gamma_v_dot, gamma_v(:,i), -1000*ones(9,1), 1000*ones(9,1), Ts);
                gamma_q(:,i+1) = IntegrationSat(gamma_q_dot, gamma_q(:,i), -1000*ones(n,1), 1000*ones(n,1), Ts);
            end
        elseif (CONTROLLER_TYPE==2)
            [tau(:,i), gamma_v_dot, gamma_q_dot] = ...
            Control_maris02(eta(:,i), [DH(:,1:3) q(:,i)], zita(:,i), ...
            eta_d(:,i), q_d(:,i), zita_d(:,i), gamma_v(:,i), gamma_q(:,i), PARAM);
            if i<npti
                gamma_v(:,i+1) = IntegrationSat(gamma_v_dot, gamma_v(:,i), -1000*ones(9*(n+1),1), 1000*ones(9*(n+1),1), Ts);
                gamma_q(:,i+1) = IntegrationSat(gamma_q_dot, gamma_q(:,i), -1000*ones(n,1), 1000*ones(n,1), Ts);
            end
        elseif (CONTROLLER_TYPE==3)
            [tau(:,i), gamma_v_dot, gamma_q_dot] = ...
            Control_maris03(eta(:,i), [DH(:,1:3) q(:,i)], zita(:,i), ...
            eta_d(:,i), q_d(:,i), zita_d(:,i), gamma_v(:,i), gamma_q(:,i), PARAM);
            if i<npti
                gamma_v(:,i+1) = IntegrationSat(gamma_v_dot, gamma_v(:,i), gamma_v_min, gamma_v_max, Ts);
                gamma_q(:,i+1) = IntegrationSat(gamma_q_dot, gamma_q(:,i), -1000*ones(n,1), 1000*ones(n,1), Ts);
            end
        elseif (CONTROLLER_TYPE==9)
            tau(:,i) = zeros(6+n,1);
        end
    end
    
    % do not modify remaing lines of the main loop
    % integration
    if i<npti
        [dzita(:,i), zita(:,i+1), eta(:,i+1), q(:,i+1)] = Integration(KinOnly, eta(:,i), [DH(:,1:3) q(:,i)],...
            zita(:,i), zita_d(:,i), tau(:,i), PARAM, Ts);
    end
    EstimateEndSim(t_start_sim, i, npti);
end
%------------------------
% main loop - end
%------------------------

figure
DrawSnap(eta(:,npti),[DH(:,1:3) q(:,npti)],PARAM,2.5);
%DrawSphere(p_obst,ee_safety_dist);
plot3(eta_ee1_d(1,:),eta_ee1_d(2,:),eta_ee1_d(3,:),'g')
plot3(eta_ee1(1,:),eta_ee1(2,:),eta_ee1(3,:),'k')
%if (exist('obj_pos','var')==1)
%    DrawCube(obj_pos,obj_size);
%    plot3([eta_ee1(1,npti) obj_pos(1)],[eta_ee1(2,npti) obj_pos(2)],[eta_ee1(3,npti) obj_pos(3)])
%end
axis auto

%------------------------
% graphics
%------------------------
if Graphics == 1
    figure
    PlotEndEffector(t,eta_ee1_d,eta_ee1,eta_ee_quat_d,eta_ee_quat);
    figure
    PlotVelocities(t,zita);
    figure
    PlotAccelerations(t,dzita);
    figure
    PlotForces(t,tau);
    figure
    PlotConfiguration2(t,eta,q,eta_d,q_d);
    figure
    PlotTasks(t,sigma_tilde_a,sigma_tilde_b,sigma_tilde_c);
elseif  Graphics == 2
    Graphics_GUI;
end

clear i t_start_sim
clear J Je eta_dot


