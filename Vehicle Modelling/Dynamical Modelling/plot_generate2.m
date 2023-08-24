figure(13)
subplot(4,2,1)
%subplot(1,3,1)
plot(Time,Y_Target,'k','linewidth',1.25)
hold on
plot(Time, YCG_TM,'--','linewidth',1.5)
hold off
legend('Target Path', 'Actual Path')
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it path (m)', 'fontname', 'Mathjax_Main', 'fontsize',12)
%title('Path')
axis([0 Time(length(Time)) min(YCG_TM)*1.10 max(YCG_TM)*1.10])

subplot(4,2,2)
plot(Time, delta, 'linewidth', 1.25)
axis([0 Time(length(Time)) min(delta)*1.10 max(delta)*1.10])
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it \delta (rad)', 'fontname', 'Mathjax_Main', 'fontsize',12)
%title('Steering Ange')

subplot(4,2,3)
plot(Time, VxTarget, 'k', 'linewidth', 1.25)
hold on
plot(Time, Vx*18/5, '--', 'linewidth',1.5)
hold off
legend('Target velocity','Actual Velocity')
axis([0 Time(length(Time)) min(Vx*18/5) max(Vx*18/5)])
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it v_x (m/s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
%title('Longitudinal Velocity')

%%
%figure(2)
subplot(4,2,4)
plot(Time, Fyf,'k','linewidth',1.25)
hold on
plot(Time, FyF, '--', 'linewidth', 1.5)
hold off
legend('CarSim','Model')
axis([0 Time(length(Time)) min(FyF)*1.10 max(FyF)*1.10])
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it F_{yf} (N)', 'fontname', 'Mathjax_Main', 'fontsize',12)

%title('Front Tire Lateral Force')

subplot(4,2,5)
plot(Time, Fyr,'k','linewidth',1.25)
hold on
plot(Time, FyR, '--', 'linewidth', 1.5)
hold off
legend('CarSim','Model')
axis([0 Time(length(Time)) min(FyR)*1.10 max(FyR)*1.10])
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it F_{yr} (N)', 'fontname', 'Mathjax_Main', 'fontsize',12)
%title('Rear Tire Lateral Force')

%%
%figure(3)
subplot(4,2,6)
plot(Time,-aLatCalc, 'k','linewidth',1.5)%Ay*9.8
hold on
plot(Time,ALat,'linewidth', 1.25)
hold on
plot(Time,ALat_model,'--','linewidth', 1.5)
hold off
legend('CarSim Output','STVM w/ CarSim Tire Model','STVM w/ Dyn. Tire Forces')
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it a_y (m/s^2)', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) min(ALat)*1.10 max(ALat)*1.10])

subplot(4,2,7)
plot(Time,YawAcc, 'k','linewidth',1.5)
hold on
plot(Time,YAwAcc, 'linewidth', 1.25)
hold on
plot(Time,YAwAcc_model,'--','linewidth',1.5)
hold off
legend('CarSim Output','STVM w/ CarSim Tire Model','STVM w/ Dyn. Tire Forces')
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it $\dot{r}\ (rad/s^2)$', 'fontname', 'Mathjax_Main', 'fontsize',12,'interpreter','latex')
axis([0 Time(length(Time)) min(YAwAcc)*1.10 max(YAwAcc)*1.10])

subplot(4,2,8)
plot(Time,BetaR,'k','linewidth',1.5)
hold on,
plot(Time,Betar, 'linewidth', 1.25)
hold on
plot(Time,Betar_model,'--','linewidth',1.5)
hold off
legend('CarSim Output','STVM w/ CarSim Tire Model','STVM w/ Dyn. Tire Forces')
xlabel('\it time (sec)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it $\dot{\beta}$ (rad/s)', 'fontname', 'Mathjax_Main', 'fontsize',12,'interpreter','latex')
axis([0 Time(length(Time)) min(Betar)*1.10 max(Betar)*1.10])

NRMSE.Fyf=goodnessOfFit(FyF,Fyf,'NRMSE');
NRMSE.Fyr=goodnessOfFit(FyR,Fyr,'NRMSE');
NRMSE.Ay=goodnessOfFit(ALat,-[0; diff(Vy)]/Time(2),'NRMSE');
NRMSE.AyMod=goodnessOfFit(ALat_model',-[0; diff(Vy)]/Time(2),'NRMSE');
NRMSE.YawAcc=goodnessOfFit(YAwAcc,YawAcc,'NRMSE');
NRMSE.YawAccMod=goodnessOfFit(YAwAcc_model',YawAcc,'NRMSE');
NRMSE.BetaR=goodnessOfFit(Betar,BetaR,'NRMSE');
NRMSE.BetaRMod=goodnessOfFit(Betar_model',BetaR,'NRMSE');

%fprintf('Table of Normalized Error for RUN- %2.0f',run);
T_NRMSE=struct2table(NRMSE)

NMSE.Fyf=goodnessOfFit(FyF,Fyf,'NMSE');
NMSE.Fyr=goodnessOfFit(FyR,Fyr,'NMSE');
NMSE.Ay=goodnessOfFit(ALat,-[0; diff(Vy)]/Time(2),'NMSE');
NMSE.AyMod=goodnessOfFit(ALat_model',-[0; diff(Vy)]/Time(2),'NMSE');
NMSE.YawAcc=goodnessOfFit(YAwAcc,YawAcc,'NMSE');
NMSE.YawAccMod=goodnessOfFit(YAwAcc_model',YawAcc,'NMSE');
NMSE.BetaR=goodnessOfFit(Betar,BetaR,'NMSE');
NMSE.BetaRMod=goodnessOfFit(Betar_model',BetaR,'NMSE');

%fprintf('Table of Normalized Error for RUN- %2.0f',run);
T_NMSE=struct2table(NMSE)

MAPE.Fyf = (mean(abs(FyF-Fyf))/mean(abs(Fyf)))*100;
MAPE.FyR = (mean(abs(FyR-Fyr))/mean(abs(Fyr)))*100;
MAPE.Ay = (mean(abs(ALat+[0; diff(Vy)]/Time(2)))/mean(abs(-[0; diff(Vy)]/Time(2))))*100;
MAPE.AyMod = (mean(abs(ALat_model'+[0; diff(Vy)]/Time(2)))/mean(abs(-[0; diff(Vy)]/Time(2))))*100;
MAPE.YawAcc = (mean(abs(YAwAcc-YawAcc))/mean(abs(YawAcc)))*100;
MAPE.YawAccMod = (mean(abs(YAwAcc_model'-YawAcc))/mean(abs(YawAcc)))*100;
MAPE.BetaR = (mean(abs(Betar-BetaR))/mean(abs(BetaR)))*100;
MAPE.BetaRMod = (mean(abs(Betar_model'-BetaR))/mean(abs(BetaR)))*100;

T_NMSE=struct2table(MAPE)