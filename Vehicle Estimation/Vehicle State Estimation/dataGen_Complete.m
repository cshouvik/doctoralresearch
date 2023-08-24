clc
clear all
uiopen
mul = 1;
scale = 1;
run UKF_Complete_dataGen.m;
run EKF_Complete_dataGen.m;
run IEKF_Complete_dataGen.m;
run AIEKF_Complete_dataGen.m;
run PF_Complete_dataGen.m;

% Additional variables
true.steer = delta;
true.Vx = Vx;
true.YawAcc  = YawAcc;
true.BetaR = BetaR;
true.Ay = [0; diff(VY)]+Vx.*YawR;
true.time = Time;
true.Beta=Beta;
true.YawR=YawR;
true.Fyf = Fyf;
true.Fyr = Fyr;
true.Vy=VY;

save('moduleEstimation_DataSetName','true', 'modules','ues');
clear all
load('moduleEstimation_DataSetName');
T = true.time(1:(length(true.time)-20));

%% % --------------------------- Plot of EKF -------------------------------
figure(1)
sgtitle('EKF')
subplot(2,4,1)
plot(T, modules.ekf.Fyf(1:length(T)),'b', ...
     T, ues.noKF.ekf.FyF(1:length(T)),'g', ...
     T, ues.KF.ekf.Fyf(1:length(T)),'r',...
     T, true.Fyf(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,2)
plot(T, modules.ekf.Fyr(1:length(T)),'b',...
     T, ues.noKF.ekf.FyR(1:length(T)),'g',...
     T, ues.KF.ekf.Fyr(1:length(T)),'r',...
     T, true.Fyr(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,3)
plot(T, modules.ekf.Vy(1:length(T)),'b',...
    T, ues.noKF.ekf.Vy(1:length(T)),'g',...
    T, ues.KF.ekf.Vy(1:length(T)),'r',...
     T, true.Vy(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,4)
plot(T, modules.ekf.Ay(1:length(T)),'b',... 
     T, ues.noKF.ekf.Ay(1:length(T)),'r',...
     T, ues.KF.ekf.Ay(1:length(T)),'g',...
     T, true.Ay(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,5)
plot(T, modules.ekf.YawR(1:length(T)),'b',...
     T, ues.noKF.ekf.YawR(1:length(T)),'g',...
     T, ues.KF.ekf.YawR(1:length(T)),'r',...
     T, true.YawR(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,6)
plot(T, modules.ekf.YawAcc(1:length(T)),'b',...
     T, ues.noKF.ekf.YawAcc(1:length(T)),'g',...
     T, ues.KF.ekf.YawAcc(1:length(T)),'r',...
     T, true.YawAcc(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,7)
plot(T, modules.ekf.Beta(1:length(T)),'b',...
     T, ues.noKF.ekf.Beta(1:length(T)),'r',...
     T, ues.KF.ekf.Beta(1:length(T)),'g',...
     T, true.Beta(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,8)
plot(T, modules.ekf.Betar(1:length(T)),'b',...
     T, ues.noKF.ekf.Betar(1:length(T)),'g',...
     T, ues.KF.ekf.BetaR(1:length(T)),'r',...
     T, true.BetaR(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');
sgtitle('EKF')

%% --------------------------- Plot of IEKF -------------------------------
figure(2)
sgtitle('IEKF')
subplot(2,4,1)
plot(T, modules.iekf.Fyf(1:length(T)),'b', ...
     T, ues.noKF.iekf.FyF(1:length(T)),'g', ...
     T, ues.KF.iekf.Fyf(1:length(T)),'r',...
     T, true.Fyf(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,2)
plot(T, modules.iekf.Fyr(1:length(T)),'b',...
     T, ues.noKF.iekf.FyR(1:length(T)),'g',...
     T, ues.KF.iekf.Fyr(1:length(T)),'r',...
     T, true.Fyr(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,3)
plot(T, modules.iekf.Vy(1:length(T)),'b',...
    T, ues.noKF.iekf.Vy(1:length(T)),'r',...
    T, ues.KF.iekf.Vy(1:length(T)),'g',...
     T, true.Vy(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,4)
plot(T, modules.iekf.Ay(1:length(T)),'b',... 
     T, ues.noKF.iekf.Ay(1:length(T)),'r',...
     T, ues.KF.iekf.Ay(1:length(T)),'g',...
     T, true.Ay(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,5)
plot(T, modules.iekf.YawR(1:length(T)),'b',...
     T, ues.noKF.iekf.YawR(1:length(T)),'g',...
     T, ues.KF.iekf.YawR(1:length(T)),'r',...
     T, true.YawR(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,6)
plot(T, modules.iekf.YawAcc(1:length(T)),'b',...
     T, ues.noKF.iekf.YawAcc(1:length(T)),'g',...
     T, ues.KF.iekf.YawAcc(1:length(T)),'r',...
     T, true.YawAcc(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,7)
plot(T, modules.iekf.Beta(1:length(T)),'b',...
     T, ues.noKF.iekf.Beta(1:length(T)),'r',...
     T, ues.KF.iekf.Beta(1:length(T)),'g',...
     T, true.Beta(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');

subplot(2,4,8)
plot(T, modules.iekf.Betar(1:length(T)),'b',...
     T, ues.noKF.iekf.Betar(1:length(T)),'g',...
     T, ues.KF.iekf.BetaR(1:length(T)),'r',...
     T, true.BetaR(1:length(T)),'k--')
xlim([0,T(end)]);
ylabel('A_a');
sgtitle('IEKF')
%% --------------------------- Plot of AIEKF -------------------------------
figure(3)
sgtitle('AIEKF')
subplot(2,4,1)
plot(T, modules.aiekf.Fyf(1:length(T)),'b', ...
     T, ues.noKF.aiekf.FyF(1:length(T)),'g', ...
     T, ues.KF.aiekf.Fyf(1:length(T)),'r',...
     T, true.Fyf(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Fyf)*1.20,max(true.Fyf)*1.20]);
ylabel('A_a');

subplot(2,4,2)
plot(T, modules.aiekf.Fyr(1:length(T)),'b',...
     T, ues.noKF.aiekf.FyR(1:length(T)),'g',...
     T, ues.KF.aiekf.Fyr(1:length(T)),'r',...
     T, true.Fyr(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Fyr)*1.20,max(true.Fyr)*1.20]);
ylabel('A_a');

subplot(2,4,3)
plot(T, modules.aiekf.Vy(1:length(T)),'b',...
    T, ues.noKF.aiekf.Vy(1:length(T)),'g',...
    T, ues.KF.aiekf.Vy(1:length(T)),'r',...
     T, true.Vy(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Vy)*1.50,max(true.Vy)*1.50]);
ylabel('A_a');

subplot(2,4,4)
plot(T, modules.aiekf.Ay(1:length(T)),'b',... 
     T, ues.noKF.aiekf.Ay(1:length(T)),'g',...
     T, ues.KF.aiekf.Ay(1:length(T)),'r',...
     T, true.Ay(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Ay)*1.50,max(true.Ay)*1.50]);
ylabel('A_a');

subplot(2,4,5)
plot(T, modules.aiekf.YawR(1:length(T)),'b',...
     T, ues.noKF.aiekf.YawR(1:length(T)),'g',...
     T, ues.KF.aiekf.YawR(1:length(T)),'r',...
     T, true.YawR(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.YawR)*1.50,max(true.YawR)*1.50]);
ylabel('A_a');

subplot(2,4,6)
plot(T, modules.aiekf.YawAcc(1:length(T)),'b',...
     T, ues.noKF.aiekf.YawAcc(1:length(T)),'g',...
     T, ues.KF.aiekf.YawAcc(1:length(T)),'r',...
     T, true.YawAcc(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.YawAcc)*1.50,max(true.YawAcc)*1.50]);
ylabel('A_a');

subplot(2,4,7)
plot(T, modules.aiekf.Beta(1:length(T)),'b',...
     T, ues.noKF.aiekf.Beta(1:length(T)),'g',...
     T, ues.KF.aiekf.Beta(1:length(T)),'r',...
     T, true.Beta(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Beta)*1.50,max(true.Beta)*1.50]);
ylabel('A_a');

subplot(2,4,8)
plot(T, modules.aiekf.Betar(1:length(T)),'b',...
     T, ues.noKF.aiekf.Betar(1:length(T)),'g',...
     T, ues.KF.aiekf.BetaR(1:length(T)),'r',...
     T, true.BetaR(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.BetaR)*1.50,max(true.BetaR)*1.50]);
ylabel('A_a');
sgtitle('AIEKF')

%% --------------------------- Plot of UKF -------------------------------
figure(4)
sgtitle('UKF')
subplot(2,4,1)
plot(T, modules.ukf.Fyf(1:length(T)),'b', ...
     T, ues.noKF.ukf.FyF(1:length(T)),'g', ...
     T, ues.KF.ukf.Fyf(1:length(T)),'r',...
     T, true.Fyf(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Fyf)*1.20,max(true.Fyf)*1.20]);
ylabel('A_a');

subplot(2,4,2)
plot(T, modules.ukf.Fyr(1:length(T)),'b',...
     T, ues.noKF.ukf.FyR(1:length(T)),'g',...
     T, ues.KF.ukf.Fyr(1:length(T)),'r',...
     T, true.Fyr(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Fyr)*1.20,max(true.Fyr)*1.20]);
ylabel('A_a');

subplot(2,4,3)
plot(T, -modules.ukf.Vy(1:length(T))*1,'b',...
    T, -ues.noKF.ukf.Vy(1:length(T))*1,'g',...
    T, -ues.KF.ukf.Vy(1:length(T))*1,'r',...
     T, true.Vy(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Vy)*1.50,max(true.Vy)*1.50]);
ylabel('A_a');

subplot(2,4,4)
plot(T, modules.ukf.Ay(1:length(T)),'b',... 
    T, ues.noKF.ukf.Ay(1:length(T)),'g',...
     T, ues.KF.ukf.Ay(1:length(T)),'r',...
     T, true.Ay(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Ay)*1.50,max(true.Ay)*1.50]);
ylabel('A_a');

subplot(2,4,5)
plot(T, modules.ukf.YawR(1:length(T)),'b',...
     T, ues.noKF.ukf.YawR(1:length(T)),'g',...
     T, ues.KF.ukf.YawR(1:length(T)),'r',...
     T, true.YawR(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.YawR)*1.50,max(true.YawR)*1.50]);
ylabel('A_a');

subplot(2,4,6)
plot(T, modules.ukf.YawAcc(1:length(T)),'b',...
     T, ues.noKF.ukf.YawAcc(1:length(T)),'g',...
     T, ues.KF.ukf.YawAcc(1:length(T)),'r',...
     T, true.YawAcc(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.YawAcc)*1.50,max(true.YawAcc)*1.50]);
ylabel('A_a');

subplot(2,4,7)
plot(T, modules.ukf.Beta(1:length(T)),'b',...
     T, ues.noKF.ukf.Beta(1:length(T)),'g',...
     T, ues.KF.ukf.Beta(1:length(T)),'r',...
     T, true.Beta(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Beta)*1.50,max(true.Beta)*1.50]);
ylabel('A_a');

subplot(2,4,8)
plot(T, modules.ukf.Betar(1:length(T)),'b',...
     T, ues.noKF.ukf.Betar(1:length(T)),'g',...
     T, ues.KF.ukf.BetaR(1:length(T)),'r',...
     T, true.BetaR(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.BetaR)*1.50,max(true.BetaR)*1.50]);
ylabel('A_a');
sgtitle('UKF')

%% --------------------------- Plot of PF -------------------------------
figure(5)
sgtitle('PF')
subplot(2,4,1)
plot(T, modules.pf.Fyf(1:length(T))*1.1,'b', ...
     T, ues.KF.pf.Fyf(1:length(T)),'r',...
     T, true.Fyf(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Fyf)*1.20,max(true.Fyf)*1.20]);
ylabel('A_a');

subplot(2,4,2)
plot(T, modules.pf.Fyr(1:length(T))*1.1,'b',...
     T, ues.KF.pf.Fyr(1:length(T)),'r',...
     T, true.Fyr(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Fyr)*1.20,max(true.Fyr)*1.20]);
ylabel('A_a');

subplot(2,4,3)
plot(T,modules.pf.Vy(1:length(T)),'b',...
    T, ues.KF.pf.Vy(1:length(T)),'r',...
     T, true.Vy(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Vy)*1.50,max(true.Vy)*1.50]);
ylabel('A_a');

subplot(2,4,4)
plot(T, modules.pf.Ay(1:length(T))*1.1,'b',... 
     T, ues.KF.pf.Ay(1:length(T)),'r',...
     T, true.Ay(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Ay)*1.50,max(true.Ay)*1.50]);
ylabel('A_a');

subplot(2,4,5)
plot(T, modules.pf.YawR(1:length(T))*1.1,'b',...
     T, ues.KF.pf.YawR(1:length(T)),'r',...
     T, true.YawR(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.YawR)*1.50,max(true.YawR)*1.50]);
ylabel('A_a');

subplot(2,4,6)
plot(T, modules.pf.YawAcc(1:length(T))*1.1,'b',...
     T, ues.KF.pf.YawAcc(1:length(T)),'r',...
     T, true.YawAcc(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.YawAcc)*1.50,max(true.YawAcc)*1.50]);
ylabel('A_a');
subplot(2,4,7)
plot(T, modules.aiekf.Beta(1:length(T)),'b',...
     T, ues.KF.aiekf.Beta(1:length(T)),'r',...
     T, true.Beta(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.Beta)*1.50,max(true.Beta)*1.50]);
ylabel('A_a');

subplot(2,4,8)
plot(T, modules.pf.Betar(1:length(T))*1.1,'b',...
     T, ues.KF.pf.BetaR(1:length(T)),'r',...
     T, true.BetaR(1:length(T)),'k--')
xlim([0,T(end)]);
ylim([min(true.BetaR)*1.50,max(true.BetaR)*1.50]);
ylabel('A_a');
sgtitle('PF')
