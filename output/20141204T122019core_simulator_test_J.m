% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

%------------------------
% simulation parameters 
%------------------------
t_f = 15;        % final simulation time [s]
Ts  = 0.01;	% sampling time [s]
t   = 0:Ts:t_f;	% time vector [s]
npti = length(t);

KinOnly   = 1;  % kinematic simulation only, controller needs to provide reference velocities
                % if null the simulation is dynamic and the controller
                % outputs the forces/moments/torques
Graphics  = 1;	% provide or not graphics in the end of the simulation

% Definition of the variables
n   = size(DH,1);
eta         = zeros(6,npti);%eta(4:6,1) = 1.2*[pi pi/2 pi]';
q           = zeros(n,npti);%q(:,1) = [0 -45 -45 0 0 0]/180*pi';
zita        = zeros(6+n,npti);
eta_ee1     = zeros(3,npti);
eta_ee_quat = zeros(4,npti);
zita_d      = zeros(6+n,npti);
eta_ee1_d   = zeros(3,npti);
eta_ee_quat_d = zeros(4,npti);
dzita = zeros(6+n,npti);
tau   = zeros(6+n,npti);

R_d = [1 0 0; 0 1 0; 0 0 1]; % desired end-effector orientation
eta2_quat_d = Rot2Quat([1 0 0; 0 1 0; 0 0 1]); % desired vehicle orientation
%eta2_quat_d = Rot2Quat([0 0 1; 0 -1 0; 1 0 0]); % desired vehicle orientation
obj_pos  = [2*1.6 -2*0.2 2]';   % object position
obj_size = 0.2;             % object size
p_obst = [1.4 0 1.5]';    % obstacle position
ee_safety_dist = .2;      % end-effector safety distance from obstacle
target  = zeros(3,npti);  % used in the cameraman task


DrawSnap(eta(:,1),[DH(:,1:3) q(:,1)],PARAM,2.1);
DrawCube(obj_pos,obj_size);
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
    eta_ee1_d(:,i) = trapezoidal2(eta_ee1(:,1),obj_pos-[0 0 .5*obj_size]',[.3 .1 .6]',6,t(i));
    % desired end-effector orientation
    %eta_ee_quat_d(:,i) = Rot2Quat([0 0 1; 0 -1 0; 1 0 0]); 
	eta_ee_quat_d(:,i) = GenerateDesQuat(eta_ee_quat(:,1),R_d,.7,12,t(i));

    % controller
    if (KinOnly==1)
        % test per sigma/J 01 - end-effector error position norm
        sigma_tilde(:,i) = sigma_tilde01(eta_ee1_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        J = J01(eta_ee1_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        k = 10;
        % test per sigma/J 02 - end-effector obstacle avoidance
        %sigma_tilde(:,i) = sigma_tilde02(p_obst,ee_safety_dist,eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %J = J02(p_obst,eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %k = 10;
        % test per sigma/J 03 - end-effector position error
        %sigma_tilde(:,i) = sigma_tilde03(eta_ee1_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %J = J03(eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %k = 10;
        % test per sigma/J 04 - end-effector orientation error
        %sigma_tilde(:,i) = sigma_tilde04(eta_ee_quat_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %J = J04(eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %k = 10;
        % test per sigma/J 05- end-effector configuration error
        %sigma_tilde(:,i) = sigma_tilde05(eta_ee1_d(:,i),eta_ee_quat_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %J = J05(eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %k = 10;      
        % test per sigma/J 06 - end-effector field-of-view error
        %a_d = [0.2 -.1 1]';a_d = a_d/norm(a_d);
        %sigma_tilde(:,i) = sigma_tilde06(a_d,eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %J = J06(a_d,eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %k = 1;
        % test per sigma/J 07 - end-effector field-of-view "cameraman" error
        %target(:,i) = [1.6 -.2-.5 2]' + 1.5*[sin(.4*t(i)); cos(.4*t(i)); 0];
        %sigma_tilde(:,i) = sigma_tilde07(target(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %J = J07(target(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        %k = 5;
        % test per sigma/J 08 - single joint position
        %sigma_tilde(:,i) = sigma_tilde08(q(:,i),30/180*pi,2);
        %J = J08(q(:,i),2);
        %k = 1;
        % test per sigma/J 09 - robot nominal position
        %sigma_tilde(:,i) = sigma_tilde09(q(:,i),[10 10 10 10 10 10]/180*pi);
        %J = J09(q(:,i));
        %k = 10;
        % test per sigma/J 10 vehicle orientation
        %sigma_tilde(:,i) = sigma_tilde10(eta2_quat_d,eta(4:6,i));
        %J = J10(n,eta(4:6,i));
        %k = 5;
        % test per sigma/J 11 - vehicle yaw
        %sigma_tilde(:,i) = sigma_tilde11(-180/180*pi,eta(4:6,i));
        %J = J11(eta(4:6,i),n);
        %k = 1;
        % test per sigma/J 12 - vehicle roll&pitch
        %sigma_tilde(:,i) = sigma_tilde12([-10 20]/180*pi,eta(4:6,i));
        %J = J12(eta(4:6,i),n);
        %k = 1;
        % test per sigma/J 13 - vehicle error position norm
        %sigma_tilde(:,i) = sigma_tilde13([.5 0 .6],eta(:,i));
        %J = J13([.5 0 .6],eta(:,i),[DH(:,1:3) q(:,i)]);
        %k = 1;
        % test per sigma/J 14 - vehicle obstacle avoidance
        %sigma_tilde(:,i) = sigma_tilde14(p_obst,.3,eta(:,i));
        %J = J14(p_obst,eta(:,i),[DH(:,1:3) q(:,i)]);
        %k = 1;

        % test per sigma/J 17 - end-effector error position norm, version 2
        sigma_tilde(:,i) = sigma_tilde18(eta_ee1_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        J = J18(eta_ee1_d(:,i),eta(:,i),[DH(:,1:3) q(:,i)],PARAM.T_0_B);
        k = 100;
        
        %-------------------
        W = diag([50 50 50 25 25 25 1*ones(1,n)]);W1=inv(W);
        zita_d(:,i) = mypinv(J,W1)*k*sigma_tilde(:,i);%keyboard
        zita_d(:,i) = pinv(J)*k*sigma_tilde(:,i);
        %zita_d(:,i) = VectorSat(zita_d(:,i),PARAM.zita_limit);
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
DrawSnap(eta(:,npti),[DH(:,1:3) q(:,npti)],PARAM,2.1);
%DrawSphere(p_obst,ee_safety_dist);
plot3(eta_ee1_d(1,:),eta_ee1_d(2,:),eta_ee1_d(3,:),'g')
%plot3(target(1,:),target(2,:),target(3,:),'g')
plot3(eta_ee1(1,:),eta_ee1(2,:),eta_ee1(3,:),'k')
if (exist('obj_pos','var')==1)
    DrawCube(obj_pos,obj_size);
    plot3([eta_ee1(1,npti) obj_pos(1)],[eta_ee1(2,npti) obj_pos(2)],[eta_ee1(3,npti) obj_pos(3)])
end
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
    PlotTasks(t,sigma_tilde);
end

clear i t_start_sim
clear J Je T eta_dot


