function PlotConfiguration(t,eta,q,q_min,q_max)
%
% PlotConfiguration(t,eta,q)
%
% input:
%       t     dim nptix1      time vector
%       eta   dim 6xnpti      vehicle configuration
%       q     dim nxnpti      joint position
%       q_min dim nx1         joint lower mechanical limit
%       q_max dim nx1         joint upper mechanical limit
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

cla

t     = CheckVector(t);
n = size(q,1);

subplot(311)
plot(t,eta(1:3,:))
title('vehicle position')
xlabel('t [s]'),ylabel('eta1 [m]')
grid

subplot(312)
plot(t,eta(4:6,:)*180/pi)
title('vehicle orientation rpy')
xlabel('t [s]'),ylabel('eta2 [deg]')
grid

subplot(313)
plot(t,q*180/pi)
title('joint position')
xlabel('t [s]'),ylabel('q [deg]')
if nargin==5
    q_min = CheckVector(q_min);
    q_max = CheckVector(q_max);
    hold on
    for i=1:n
        plot(t,q_min(i)*ones(size(t))*180/pi, 'color', [0.5 0.5 0.5])
        plot(t,q_max(i)*ones(size(t))*180/pi, 'color', [0.5 0.5 0.5])
    end
end
