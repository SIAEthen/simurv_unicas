%
% genera figure degli esperimenti per l'articolo xyz
%
%
clear
close all
clc
fsize = 16;


load('med2014_caso1_still_out.mat','-mat')
npti = length(tau);
norm_pos_err_caso1 = zeros(npti,1);
norm_or_err_caso1  = zeros(npti,1);
norm_force_caso1   = zeros(npti,1);
norm_moment_caso1  = zeros(npti,1);
for i=1:npti
    norm_pos_err_caso1(i) = norm(eta_d(1:3,i)-eta(1:3,i));
    norm_or_err_caso1(i)  = norm(eta_d(4:6,i)-eta(4:6,i));
    norm_force_caso1(i)   = norm(tau(1:3,i));
    norm_moment_caso1(i)  = norm(tau(4:6,i));
end

load('med2014_caso3_still_out.mat','-mat')
norm_pos_err_caso3 = zeros(npti,1);
norm_or_err_caso3  = zeros(npti,1);
norm_force_caso3   = zeros(npti,1);
norm_moment_caso3  = zeros(npti,1);
for i=1:npti
    norm_pos_err_caso3(i) = norm(eta_d(1:3,i)-eta(1:3,i));
    norm_or_err_caso3(i)  = norm(eta_d(4:6,i)-eta(4:6,i));
    norm_force_caso3(i)   = norm(tau(1:3,i));
    norm_moment_caso3(i)  = norm(tau(4:6,i));
end

h=figure;
set(gca,'FontSize',fsize)
plot(t,norm_moment_caso1,'b','LineWidth', 2);grid on; hold on
plot(t,norm_moment_caso3,'g','LineWidth', 2);
xlabel('t [s]');
ylabel('moment [Nm]');
print -depsc med2014/moment_still

h=figure;
set(gca,'FontSize',fsize)
plot(t,norm_force_caso1,'b','LineWidth', 2);grid on; hold on
plot(t,norm_force_caso3,'g','LineWidth', 2);
xlabel('t [s]');
ylabel('force [N]');
print -depsc med2014/force_still

h=figure;
set(gca,'FontSize',fsize)
plot(t,norm_or_err_caso1/pi*180,'b','LineWidth', 2);grid on; hold on
plot(t,norm_or_err_caso3/pi*180,'g','LineWidth', 2);
xlabel('t [s]');
ylabel('orerr [deg]');
print -depsc med2014/orerr_still

h=figure;
set(gca,'FontSize',fsize)
plot(t,norm_pos_err_caso1,'b','LineWidth', 2);grid on; hold on
plot(t,norm_pos_err_caso3,'g','LineWidth', 2);
xlabel('t [s]');
ylabel('poserr [m]');
print -depsc med2014/poserr_still

h=figure;
set(gca,'FontSize',fsize)
plot(t,eta_d(4,:)/pi*180,'m','LineWidth', 2);grid on; hold on
plot(t,eta_d(5,:)/pi*180,'k','LineWidth', 2);
plot(t,eta_d(6,:)/pi*180,'r','LineWidth', 2);
xlabel('t [s]');
ylabel('rpy [deg]');
print -depsc med2014/destraj2_still

h=figure;
set(gca,'FontSize',fsize)
plot(t,eta_d(1,:),'m','LineWidth', 2);grid on; hold on
plot(t,eta_d(2,:),'k','LineWidth', 2);
plot(t,eta_d(3,:),'r','LineWidth', 2);
xlabel('t [s]');
ylabel('eta1d [m]');
print -depsc med2014/destraj1_still


load('med2014_caso1out.mat','-mat')
npti = length(tau);
norm_pos_err_caso1 = zeros(npti,1);
norm_or_err_caso1  = zeros(npti,1);
norm_force_caso1   = zeros(npti,1);
norm_moment_caso1  = zeros(npti,1);
for i=1:npti
    norm_pos_err_caso1(i) = norm(eta_d(1:3,i)-eta(1:3,i));
    norm_or_err_caso1(i)  = norm(eta_d(4:6,i)-eta(4:6,i));
    norm_force_caso1(i)   = norm(tau(1:3,i));
    norm_moment_caso1(i)  = norm(tau(4:6,i));
end

load('med2014_caso3out.mat','-mat')
norm_pos_err_caso3 = zeros(npti,1);
norm_or_err_caso3  = zeros(npti,1);
norm_force_caso3   = zeros(npti,1);
norm_moment_caso3  = zeros(npti,1);
for i=1:npti
    norm_pos_err_caso3(i) = norm(eta_d(1:3,i)-eta(1:3,i));
    norm_or_err_caso3(i)  = norm(eta_d(4:6,i)-eta(4:6,i));
    norm_force_caso3(i)   = norm(tau(1:3,i));
    norm_moment_caso3(i)  = norm(tau(4:6,i));
end

h=figure;
set(gca,'FontSize',fsize)
plot(t,norm_moment_caso1,'b','LineWidth', 2);grid on; hold on
plot(t,norm_moment_caso3,'g','LineWidth', 2);
xlabel('t [s]');
ylabel('moment [Nm]');
print -depsc med2014/moment

h=figure;
set(gca,'FontSize',fsize)
plot(t,norm_force_caso1,'b','LineWidth', 2);grid on; hold on
plot(t,norm_force_caso3,'g','LineWidth', 2);
xlabel('t [s]');
ylabel('force [N]');
print -depsc med2014/force

h=figure;
set(gca,'FontSize',fsize)
plot(t,norm_or_err_caso1/pi*180,'b','LineWidth', 2);grid on; hold on
plot(t,norm_or_err_caso3/pi*180,'g','LineWidth', 2);
xlabel('t [s]');
ylabel('orerr [deg]');
print -depsc med2014/orerr

h=figure;
set(gca,'FontSize',fsize)
plot(t,norm_pos_err_caso1,'b','LineWidth', 2);grid on; hold on
plot(t,norm_pos_err_caso3,'g','LineWidth', 2);
xlabel('t [s]');
ylabel('poserr [m]');
print -depsc med2014/poserr

h=figure;
set(gca,'FontSize',fsize)
plot(t,eta_d(4,:)/pi*180,'m','LineWidth', 2);grid on; hold on
plot(t,eta_d(5,:)/pi*180,'k','LineWidth', 2);
plot(t,eta_d(6,:)/pi*180,'r','LineWidth', 2);
xlabel('t [s]');
ylabel('rpy [deg]');
print -depsc med2014/destraj2

h=figure;
set(gca,'FontSize',fsize)
plot(t,eta_d(1,:),'m','LineWidth', 2);grid on; hold on
plot(t,eta_d(2,:),'k','LineWidth', 2);
plot(t,eta_d(3,:),'r','LineWidth', 2);
xlabel('t [s]');
ylabel('eta1d [m]');
print -depsc med2014/destraj1


clear all

