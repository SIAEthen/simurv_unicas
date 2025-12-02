% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

%------------------------
% simulation parameters 
%------------------------
KinOnly   = 1;  % kinematic simulation only, controller needs to provide reference velocities
                % if null the simulation is dynamic and the controller
                % outputs the forces/moments/torques
Graphics  = 1;	% provide or not graphics in the end of the simulation

t_f = 15;       % final simulation time [s]
Ts  = 0.01;     % sampling time [s]
t   = 0:Ts:t_f;	% time vector [s]
npti = length(t);

% Definition of the variables
n   = size(DH,1);
eta           = zeros(6,npti);
q             = zeros(n,npti); q (:,1) = [0 -45 -45 0 0 0]/180*pi;
zita          = zeros(6+n,npti);
eta_ee1       = zeros(3,npti);
eta_ee_quat   = zeros(4,npti);
zita_d        = zeros(6+n,npti);
eta_ee1_d     = zeros(3,npti);
eta_ee_quat_d = zeros(4,npti);
dzita = zeros(6+n,npti);
tau   = zeros(6+n,npti);

% ---------------
% task variables
% ---------------
ma = 6;     % dim of task a
mb = 2;     % dim of task b
mc = 6;     % dim of task c
sigma_tilde_a = zeros(ma,npti);
sigma_tilde_b = zeros(mb,npti);
sigma_tilde_c = zeros(mc,npti);

R_d = [0 0 1; 0 -1 0; 1 0 0]; % desired end-effector orientation
eta2_quat_d = Rot2Quat([1 0 0; 0 1 0; 0 0 1]); % desired vehicle orientation
obj_pos  = [1.6 -0.2 2]';   % object position
obj_size = 0.2;             % object size
p_obst = [1.4 0 1.5]';    % obstacle position
ee_safety_dist = .2;      % end-effector safety distance from obstacle
W = diag([50 50 50 25 25 25 1*ones(1,n)]);invW=inv(W);


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
    eta_ee1_d(:,i) = trapezoidal2(eta_ee1(:,1),eta_ee1(:,1)+[.2 .2 .2]',[.2 .03 .3]',6,t(i));
    eta_ee_quat_d(:,i) = GenerateDesQuat(eta_ee_quat(:,1),R_d,.45,12,t(i));
    eta_ee_quat(:,i) = AlignQuat(eta_ee_quat_d(:,i),eta_ee_quat(:,i));
    
    % controller
    if (KinOnly==1)
        % first-priority task - end-effector configuration
        sigma_tilde_a(:,i) = sigma_tilde05(eta_ee1_d(:,i),eta_ee_quat_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        J_a = J05(eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        pinvJ_a = mypinv(J_a,invW,'a');
        N_a = eye(6+n,6+n)-pinvJ_a*J_a;
        k_a = 10;
        % second-priority task - null roll/pitch
        sigma_tilde_b(:,i) = sigma_tilde12([0 0]/180*pi,eta(4:6,i));
        J_b = J12(eta(4:6,i),n);
        pinvJ_b = mypinv(J_b,invW,'b');
        k_b = 10;
        % third-priority task 
        J_ab = [J_a; J_b];
        pinvJ_ab = mypinv(J_ab,invW);
        N_ab = eye(6+n,6+n)-pinvJ_ab*J_ab;
        sigma_tilde_c(:,i) = sigma_tilde09(q(:,i),PARAM.q_nominal);
        J_c = J09(q(:,i));
        k_c = diag([0 10 0 0 0 0]);
        pinvJ_c = mypinv(J_c,invW,'c');

        %-------------------
        zita_d(:,i) =      pinvJ_a*k_a*sigma_tilde_a(:,i) + ...
                       N_a*pinvJ_b*k_b*sigma_tilde_b(:,i) +...
                      N_ab*pinvJ_c*k_c*sigma_tilde_c(:,i);
        zita_d(:,i) = VectorSat(zita_d(:,i),PARAM.zita_limit);
    else
        tau(:,i) = zeros(6+n,1);
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
if Graphics
    figure
    PlotEndEffector(t,eta_ee1_d,eta_ee1,eta_ee_quat_d,eta_ee_quat);
    figure
    PlotVelocities(t,zita);
    figure
    PlotConfiguration(t,eta,q);
    figure
    PlotTasks(t,sigma_tilde_a,sigma_tilde_b,sigma_tilde_c);
end

clear i t_start_sim
clear J Je eta_dot


