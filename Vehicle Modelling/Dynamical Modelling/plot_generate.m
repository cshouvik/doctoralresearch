figure(10)
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


%figure(3)
subplot(4,2,6)
plot(Time,Ay*9.8, 'k','linewidth',1.5)
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
%%
NRMSE.Fyf=goodnessOfFit(FyF,Fyf,'NRMSE');
NRMSE.Fyr=goodnessOfFit(FyR,Fyr,'NRMSE');
NRMSE.Ay=goodnessOfFit(ALat,AY,'NRMSE');
NRMSE.AyMod=goodnessOfFit(ALat_model',AY,'NRMSE');
NRMSE.YawAcc=goodnessOfFit(YAwAcc,YawAcc,'NRMSE');
NRMSE.YawAccMod=goodnessOfFit(YAwAcc_model',YawAcc,'NRMSE');
NRMSE.BetaR=goodnessOfFit(Betar,BetaR,'NRMSE');
NRMSE.BetaRMod=goodnessOfFit(Betar_model',BetaR,'NRMSE');

%fprintf('Table of Normalized Error for RUN- %2.0f',run);
T_NRMSE=struct2table(NRMSE)

NMSE.Fyf=goodnessOfFit(FyF,Fyf,'NMSE');
NMSE.Fyr=goodnessOfFit(FyR,Fyr,'NMSE');
NMSE.Ay=goodnessOfFit(ALat,AY,'NMSE');
NMSE.AyMod=goodnessOfFit(ALat_model',AY,'NMSE');
NMSE.YawAcc=goodnessOfFit(YAwAcc,YawAcc,'NMSE');
NMSE.YawAccMod=goodnessOfFit(YAwAcc_model',YawAcc,'NMSE');
NMSE.BetaR=goodnessOfFit(Betar,BetaR,'NMSE');
NMSE.BetaRMod=goodnessOfFit(Betar_model',BetaR,'NMSE');

%fprintf('Table of Normalized Error for RUN- %2.0f',run);
T_NMSE=struct2table(NMSE)

MAPE.Fyf = (mean(abs(FyF)-abs(Fyf))/mean(abs(Fyf)))*100;
MAPE.FyR = (mean(abs(FyR)-abs(Fyr))/mean(abs(Fyr)))*100;
MAPE.Ay = (mean(abs(ALat)-abs(AY))/mean(abs(AY)))*100;
MAPE.AyMod = (mean(abs(ALat_model')-abs(AY))/mean(abs(AY)))*100;
MAPE.YawAcc = (mean(abs(YAwAcc)-abs(YawAcc))/mean(abs(YawAcc)))*100;
MAPE.YawAccMod = (mean(abs(YAwAcc_model')-abs(YawAcc))/mean(abs(YawAcc)))*100;
MAPE.BetaR = (mean(abs(Betar)-abs(BetaR))/mean(abs(BetaR)))*100;
MAPE.BetaRMod = (mean(abs(Betar_model')-abs(BetaR))/mean(abs(BetaR)))*100;

T_MAPE=struct2table(MAPE)
%%
MSE.Fyf=goodnessOfFit(FyF,Fyf,'MSE');
MSE.Fyr=goodnessOfFit(FyR,Fyr,'MSE');
MSE.Ay=goodnessOfFit(ALat,AY,'MSE');
MSE.AyMod=goodnessOfFit(ALat_model',AY,'MSE');
MSE.YawAcc=goodnessOfFit(YAwAcc,YawAcc,'MSE');
MSE.YawAccMod=goodnessOfFit(YAwAcc_model',YawAcc,'MSE');
MSE.BetaR=goodnessOfFit(Betar,BetaR,'MSE');
MSE.BetaRMod=goodnessOfFit(Betar_model',BetaR,'MSE');

%fprintf('Table of Normalized Error for RUN- %2.0f',run);
T_MSE=struct2table(MSE)
%%
[RMSE.Fyf, MSE.Fyf, AvgEr.Fyf, pAvgEr.Fyf]=rmseCalc(FyF,Fyf);
[RMSE.Fyr, MSE.Fyr, AvgEr.Fyr, pAvgEr.Fyr]=rmseCalc(FyR,Fyr);
[RMSE.Ay, MSE.Ay, AvgEr.Ay, pAvgEr.Ay]=rmseCalc(ALat,AY);
[RMSE.AyMod, MSE.AyMod, AvgEr.AyMod, pAvgEr.AyMod]=rmseCalc(ALat_model',AY);
[RMSE.YawAcc, MSE.YawAcc, AvgEr.YawAcc, pAvgEr.YawAcc]=rmseCalc(YAwAcc,YawAcc);
[RMSE.YawAccMod, MSE.YawAccMod, AvgEr.YawAccMod, pAvgEr.YawAccMod]=rmseCalc(YAwAcc_model',YawAcc);
[RMSE.BetaR, MSE.BetaR, AvgEr.BetaR, pAvgEr.BetaR]=rmseCalc(Betar,BetaR);
[RMSE.BetaRMod, MSE.BetaRMod, AvgEr.BetaRMod, pAvgEr.BetaRMod]=rmseCalc(Betar_model',BetaR);

%fprintf('Table of Normalized Error for RUN- %2.0f',run);
T_RMSE=struct2table(RMSE)
T_MSE=struct2table(MSE)
T_AvgErr=struct2table(AvgEr)
T_pAvgErr=struct2table(pAvgEr)
%% Drive and Brake Torque : for braking and acceleration tests
figure(12)
plot(Time, My_Bk_L1);
hold on;
plot(Time, My_Bk_L2);
plot(Time, My_Bk_R1);
plot(Time, My_Bk_R2);
hold off;
legend('Brake Torque Front Left Tire', 'Brake Torque Front Right Tire','Brake Torque Rear Left Tire', 'Brake Torque Rear Right Tire' )
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it brake torque (Nm)', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) -1500 300])

figure(13)
plot(Time, My_Dr_L1);
hold on;
plot(Time, My_Dr_L2);
plot(Time, My_Dr_R1);
plot(Time, My_Dr_R2);
hold off;
legend('Drive Torque Front Left Tire', 'Drive Torque Front Right Tire','Drive Torque Rear Left Tire', 'Drive Torque Rear Right Tire' )
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it drive torque (Nm)', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) -200 700])

figure(14)
subplot(2,2,1)
plot(Time, MuX_L1, Time, MuX_L2)
legend('Front Left Tire', 'Rear Left Tire' )
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it longitudinal friction', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) 0 0.6])

subplot(2,2,2)
plot(Time, MuX_R1, Time, MuX_R2)
legend('Front Right Tire', 'Rear Rigth Tire' )
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it longitudinal friction', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) 0 0.6])

subplot(2,2,3)
plot(Time, MuY_L1, Time, MuY_L2)
legend('Front Left Tire', 'Rear Left Tire' )
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it lateral friction', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) 0.1 0.6])

subplot(2,2,4)
plot(Time, MuY_L1, Time, MuY_L2)
legend('Front Right Tire', 'Rear Rigth Tire' )
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it lateral friction', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) 0.1 0.6])

figure(15)
subplot(1,2,1)
plot(Time, MuX_L1, Time, MuX_L2, Time, MuX_R1, Time, MuX_R2)
legend('Front Left Tire', 'Rear Left Tire','Front Right Tire', 'Rear Rigth Tire' )
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it longitudinal friction', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) 0.1 0.7])

subplot(1,2,2)
plot(Time, MuY_L1, Time, MuY_L2,Time, MuY_L1, Time, MuY_L2)
legend('Front Left Tire', 'Rear Left Tire','Front Right Tire', 'Rear Rigth Tire' )
xlabel('\it time (s)', 'fontname', 'Mathjax_Main', 'fontsize',12)
ylabel ('\it lateral friction', 'fontname', 'Mathjax_Main', 'fontsize',12)
axis([0 Time(length(Time)) 0.1 0.7])
