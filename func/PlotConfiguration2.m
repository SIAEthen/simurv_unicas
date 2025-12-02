function PlotConfiguration2(t,eta,q,eta_d,q_d)
%
% PlotConfiguration2(t,eta,q,eta_d,q_d)
%
% input:
%       t   dim nptix1      time vector
%       eta dim 6xnpti      vehicle configuration
%       q   dim nxnpti      joint position
%       eta dim 6xnpti      desired vehicle configuration
%       q   dim nxnpti      desired joint position
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv


subplot(321)
cla
plot(t,eta(1:3,:))
hold on
plot(t,eta_d(1:3,:),'--')
title('vehicle position (solid-real, dashed-desired)')
xlabel('t [s]'),ylabel('eta1 [m]')
grid
hold off

subplot(322)
cla
plot(t,eta_d(1:3,:)-eta(1:3,:))
title('vehicle position error')
xlabel('t [s]'),ylabel('eta1-tilde [m]')
grid
hold off

subplot(323)
cla
plot(t,eta(4:6,:)*180/pi)
hold on
plot(t,eta_d(4:6,:)*180/pi,'--')
title('vehicle orientation rpy (solid-real, dashed-desired)')
xlabel('t [s]'),ylabel('eta2 [deg]')
grid
hold off

subplot(324)
cla
plot(t,(eta_d(4:6,:)-eta(4:6,:))*180/pi)
title('vehicle orientation error rpy')
xlabel('t [s]'),ylabel('eta2-tilde [deg]')
grid
hold off

subplot(325)
cla
plot(t,q*180/pi)
hold on
plot(t,q_d*180/pi,'--')
title('joint position (solid-real, dashed-desired)')
xlabel('t [s]'),ylabel('q [deg]')
grid
hold off

subplot(326)
cla
plot(t,(q_d-q)*180/pi)
title('joint position error')
xlabel('t [s]'),ylabel('q-tilde [deg]')
grid
hold off