% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

%------------------------
% simulation parameters 
%------------------------
t_f = 20;       % final simulation time [s]
Ts  = 0.005;	% sampling time [s]
t   = 0:Ts:t_f;	% time vector [s]
npti = length(t);

KinOnly   = 0;  % kinematic simulation only, controller needs to provide reference velocities
                % if null the simulation is dynamic and the controller
                % outputs the forces/moments/torques
Graphics  = 1;	% provide or not graphics in the end of the simulation

% Definition of the variables
n   = size(DH,1);
eta           = zeros(6,npti);
q             = zeros(n,npti);
zita          = zeros(6+n,npti);
eta_d         = zeros(6,npti);
q_d           = zeros(n,npti);
zita_d        = zeros(6+n,npti);
gamma         = zeros(9,npti);
gamma(1,1)    = 200;
gamma(6,1)    = -600;
gamma_q       = zeros(n,npti);
dzita = zeros(6+n,npti);
tau   = zeros(6+n,npti);


DrawSnap(eta(:,1),[DH(:,1:3) q(:,1)],PARAM,2.1);
axis auto
drawnow

%------------------------
% main loop - start
%------------------------
t_start_sim = clock();
for i=1:npti
    
    % controller
    if (KinOnly==1)
        fprintf('\n ------------------------------------------------------------------- ');
        fprintf('\n ERROR in core_simulator_demo2.m: this demo only runs with KinOnly=0 ');
        fprintf('\n ------------------------------------------------------------------- ');
        return;
    else
        zita_d(:,i) = 0*zita_d(:,i);
        if i<npti
            Je           = J_e(eta(4:6,i));
            eta_d(:,i+1) = eta_d(:,i) + Ts*(Je\zita_d(1:6,i));
            q_d(:,i+1)   = q_d(:,i)   + Ts*zita_d(7:6+n,i);
        end
        
        % "basic" controller, vehicle is implementing
        % an adaptive action ignoring the presence of the manipulator
        % then, PID at joints
        % vehicle controller
        Kd = 2*diag([3000 3000 3000 300 2500 900]);
        lambda = diag([.08 .08 .08 1 1 1]);
        invKtheta = diag([4 4 4 4 4 4 4 4 4]);

        e   = Rpy2Quat(eta(4:6,i));
        e_d = Rpy2Quat(eta_d(4:6,i));
        e_o = e(4)*e_d(1:3) - e_d(4)*e(1:3) + cross(e_d(1:3),e(1:3));
        eta_tilde_1 = eta_d(1:3,i) - eta(1:3,i);
        nu_tilde = zita_d(1:6,i) - zita(1:6,i);
        R = Quat2Rot(e);
        s = lambda*[R*eta_tilde_1;  e_o] + nu_tilde;
        phi_p = [zeros(3,3)    R'            zeros(3,3);
                 S(R'*[0;0;1])  zeros(3,3)   R'];
        tau(1:6,i) = Kd*s + phi_p*gamma(1:9,i);
        if i<npti
            gamma(1:9,i+1) = gamma(1:9,i) + invKtheta*phi_p'*s;
        end
        % PID at joints
        Kp = diag([2000 2000 2000 200 200 200]);
        Ki = diag([1 1 1 1 1 1]);
        Kd = diag([500 500 500 200 200 200]);
        
        tau(7:6+n,i) = Kp*(q_d(:,i)-q(:,i)) + Kd*(zita_d(7:6+n,i)-zita(7:6+n,i)) + Ki*gamma_q(:,i);
        if i<npti
            gamma_q(:,i+1) = gamma_q(:,i) + Ki*(q_d(:,i)-q(:,i));
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
DrawSnap(eta(:,npti),[DH(:,1:3) q(:,npti)],PARAM,2.1);
axis auto


%------------------------
% graphics
%------------------------
if Graphics
    figure
    PlotForces(t,tau);
    figure
    PlotAccelerations(t,dzita);
    figure
    PlotVelocities(t,zita);
    figure
    PlotConfiguration(t,eta,q);
end

clear i t_start_sim
clear J Je T eta_dot
clear eta_tilde_1 phi_p


