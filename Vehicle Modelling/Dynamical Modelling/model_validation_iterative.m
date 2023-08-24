clc
clear all
% load('D:\Research\CarSim Generated Data\EClass Sedan DLC\MatLab Files\Sampling Time 0.025\DLC_Tight_Mu_0_85.mat')
uiopen
% run=input('Enter the dataset=>RUN');
rad=pi/180;
deg=180/pi;
Vx=Vx.*5/18;
Vy=Vy.*5/18;
yaw=Yaw;
delta=(1/2.*(Steer_L1+Steer_R1))*rad;
Fyf=(1.*(Fy_L1+Fy_R1));
Fyr=(1.*(Fy_L2+Fy_R2));
Fxf=(Fx_L1+Fx_R1);
Fxr=(Fx_L2+Fx_R2);
Alphaf=(1/2.*(Alpha_L1+Alpha_R1));
Alphar=(1/2.*(Alpha_L2+Alpha_R2));
YawR=AVz*rad;
YawAcc=AAz;
BetaR=BetaR*rad;
Beta=Beta*rad;
% Iz=input('Enter moment of inertia Iz(Nm): ');
% lf=input('Enter Value of lf(mm): ')/1000;
% lr=input('Enter Value of lr(mm): ')/1000;
% ms=input('Enter sprung mass of vehicle ms(kg): ');
% mu=input('Enter unsprung mass of vehicle mu(kg): ');
lr=(3.05-1.4);
lf=1.4;
m=1650;
Iz=3234;
l=lf+lr;
lw = 1.6;
g=9.8;
t=1:length(Time);
AY=Ay*9.8;
phi=Roll*rad;
Tsample=Time(2)-Time(1);
%% Calcultaing Myf and Myr
TStop=Time(length(Time));
FYF=[Time Fyf];
FYR=[Time Fyr];
options=simset('Solver','FixedStepDiscrete','FixedStep','Tsample');
[x,y]=sim('myf_myr_extract',TStop,simset('Solver','ode1','FixedStep','Tsample'));
%% Initialize Buffer
FyF=zeros(length(Time),1);
FyR=zeros(length(Time),1);
MyF=zeros(length(Time),1);
MyR=zeros(length(Time),1);
YAwAcc=[YawAcc(1); zeros(length(size(Time)-1),1)];
ALat=[AY(1); zeros((length(Time)-1),1)];
ALat2=[zeros((length(Time)),1)];
Betar=[BetaR(1); zeros((length(Time)-1),1)];
VLat=[Vy(1); zeros((length(Time)-1),1)];
RYaw=[YawR(1); zeros((length(Time)-1),1)];
BETA=[Beta(1); zeros((length(Time)-1),1)];

alpha_1=lr.*m.*Vx(1)./(l.*cos(delta(1)));
alpha_2=lf.*m.*Vx(1)./l;
FyF(1)=alpha_1.*BetaR(1)+alpha_1.*YawR(1)+alpha_1.*(Iz./(lr.*m.*Vx(1))).*YawAcc(1);
FyR(1)=alpha_2.*BetaR(1)+alpha_2.*YawR(1)-(Iz./l).*YawAcc(1);
%%
for i=2:length(Time)

alpha_1=lr.*m.*Vx(i-1)./(l.*cos(delta(i-1)));

alpha_2=lf.*m.*Vx(i-1)./l;

alpha_3=(1/m)*(alpha_2-alpha_1.*cos(delta(i-1)));

FyF(i)=alpha_1.*Betar(i-1)+alpha_1.*YawR(i-1)+alpha_1.*(Iz./(lr.*m.*Vx(i-1))).*YAwAcc(i-1);

MyF(i)=MyF(i-1)+Tsample*FyF(i-1);

FyR(i)=alpha_2.*Betar(i-1)+alpha_2.*YawR(i-1)-(Iz./l).*YAwAcc(i-1); 

MyR(i)=MyR(i-1)+Tsample*FyR(i-1);

ALat(i)=(1/m)*(Fxf(i)*sin(delta(i))-Fyf(i-1).*cos(delta(i-1))+Fyf(i-1)+m*YawR(i-1).*Vx(i-1));

%ALat2(i)=((((AY(i-1)-g*sin(phi(i-1))./Vx(i-1)))./cos(phi(i-1)))-Vx(i-1).*YawR(i-1));

VLat(i)=VLat(i-1)+Tsample*((((AY(i-1)-g*sin(phi(i-1))./Vx(i-1)))./cos(phi(i-1)))-Vx(i-1).*YawR(i-1));

YAwAcc(i)=(1/Iz)*(lf*Fxf(i)*sin(delta(i))+lf*Fyf(i-1).*cos(delta(i-1))-lr.*Fyr(i-1));
 
RYaw(i)=RYaw(i-1)+Tsample*YAwAcc(i-1);

Betar(i)=(1./(m*Vx(i-1))).*(Fxf(i)*sin(delta(i))+Fyf(i-1).*cos(delta(i-1))+Fyr(i-1)+m*g*sin(-phi(i-1)))-YawR(i-1); 

BETA(i)=BETA(i-1)+Tsample*((1./(m*Vx(i-1))).*(Fyf(i-1).*cos(delta(i-1))+Fyr(i-1)+m*g*sin(-phi(i-1)))-YawR(i-1));

ALat_model(i)=(1/m)*(Fxf(i)*sin(delta(i))-FyF(i).*cos(delta(i))+FyR(i)+m*YawR(i).*Vx(i));

YAwAcc_model(i)=(1/Iz)*(lf*(Fxf(i)*sin(delta(i))+FyF(i).*cos(delta(i)))-lr.*FyR(i));

Betar_model(i)=(1./(m*Vx(i))).*(Fxf(i)*sin(delta(i))+FyF(i-1).*cos(delta(i-1))+FyR(i-1)+m*g*sin(-phi(i-1)))-YawR(i-1); 
end

aLatCalc = [0;diff(Vy)]/Time(2);
%%
% figure(1)
% %subplot(121)
% plot(Time,FyF','k-',Time,Fyf,'kx')
% title('Front Tire Lateral Force')
% xlabel('Time(in sec)')
% ylabel('Fyf(in N)')
% legend('Model Output','CarSim Output','location','northeast')
% axis([0 Time(length(Time)) min(Fyf)*1.10 max(Fyf)*1.10])
% 
% %subplot(122)
% figure(9)
% plot(Time,FyR','k-',Time,Fyr,'kx')
% title('Rear Tire Lateral Force')
% xlabel('Time(in sec)')
% ylabel('Fyr(in N)')
% legend('Model Output','CarSim Output','location','northeast')
% axis([0 Time(length(Time)) min(Fyr)*1.10 max(Fyr)*1.10])
% 
% % figure(8)
% % %subplot(121)
% % plot(Time,MyF','k-',Time,Myf,'kx')
% % title('Front Tire Lateral Momentum')
% % xlabel('Time(in sec)')
% % ylabel('Myf(in kg m/s)')
% % legend('Model Output','CarSim Output','location','northeast')
% % axis([0 Time(length(Time)) min(Myf)*1.10 max(Myf)*1.10])
% 
% %subplot(122)
% % figure(10)
% % plot(Time,MyR','k-',Time,Myr,'kx')
% % title('Rear Tire Lateral Momentum')
% % xlabel('Time(in sec)')
% % ylabel('Myr(in kg m/s)')
% % legend('Model Output','CarSim Output','location','northeast')
% % axis([0 Time(length(Time)) min(Myr)*1.10 max(Myr)*1.10])
% 
% % figure(2)
% % plot(Time,VLat,'k-',Time,Vy,'kx')
% % title('Lateral Velocity')
% % xlabel('Time(in sec)')
% % ylabel('Vy(in m/sec)')
% % legend('Model Output','carsim','location','northeast')
% % axis([0 Time(length(Time)) min(Vy)*1.10 max(Vy)*1.10])
% 
% figure(3)
% plot(Time,ALat,'k-',Time,ALat_model,'b-',Time,Ay*9.8,'kx')
% title('Lateral Acceleration')
% xlabel('Time(in sec)')
% ylabel('Ay(in m/s^2)')
% legend('Model Output w/o tire force model','Model Output w/ tire force model','CarSim Output','location','northeast')
% axis([0 Time(length(Time)) min(Ay*9.8)*1.10 max(Ay*9.8)*1.10])
% 
% % figure(6)
% % plot(Time,RYaw,'k-',Time,YawR,'kx')
% % title('Yaw Rate')
% % xlabel('Time(in sec)')
% % ylabel('Yaw Rate(in rad/s)')
% % legend('Model Output','CarSim Output','location','northeast')
% % axis([0 Time(length(Time)) min(YawR)*1.20 max(YawR)*1.30])
% 
% figure(4)
% plot(Time,YAwAcc,'k-',Time,YAwAcc_model,'b-',Time,YawAcc,'kx')
% title('Yaw Acceleration')
% xlabel('Time(in sec)')
% ylabel('Yaw Acceleration(in rad/sec^2)')
% legend('Model Output w/o tire force model','Model Output w/ tire force model','CarSim Output','location','northeast')
% axis([0 Time(length(Time)) min(YawAcc)*1.20 max(YawAcc)*1.30])
% 
% figure(5)
% plot(Time,Betar,'k-',Time,Betar_model,'b-',Time,BetaR,'kx')
% title('Slip Rate(Vehicle Body Slip Angle)')
% xlabel('Time(in sec)')
% ylabel('Slip Rate(in rad/s)')
% legend('Model Output w/o tire force model','Model Output w/ tire force model','CarSim Output','location','northeast')
% axis([0 Time(length(Time)) min(BetaR)*1.10 max(BetaR)*1.10])
% 
% % figure(7)
% % plot(Time,BETA,'k-',Time,Beta,'kx')
% % title('Vehicle Body Slip Angle')
% % xlabel('Time(in sec)')
% % ylabel('Slip Angle(in rad)')
% % legend('Model Output','CarSim Output','location','northeast')
% % axis([0 Time(length(Time)) min(Beta)*1.20 max(Beta)*1.30])
% 
% % figure(8)
% % plot(t,ALat2,'red',t,[0;diff(Vy)]/Tsample,'blue')   %0.025=>Sampling Period
% % title('Lateral Acceleration(D^2y)')
% % xlabel('iterations')
% % ylabel('Ay(in m/s^2)')1
% % legend('Model','Carsim','location','northeast')
% 
% %%%%%%---------Display Result
% % error_Fyf = (Fyf-FyF).^2;
% % RMSE_Fyf  = error_Fyf.^0.5;
% % mean_RMSE_Fyf = mean(RMSE_Fyf);
% % NMSE.Fyf=mean_RMSE_Fyf/(max(Fyf)-min(Fyf));
% % fprintf('Modelling Error RMSE in Fyf : %2.3f\n\n',mean_RMSE_Fyf/(max(Fyf)-min(Fyf)));
% % 
% % error_Fyr = (Fyr-FyR).^2;
% % RMSE_Fyr  = error_Fyr.^0.5;
% % mean_RMSE_Fyr = mean(RMSE_Fyr);
% % NMSE.Fyr=mean_RMSE_Fyr/(max(Fyr)-min(Fyr));
% % fprintf('Modelling Error RMSE in Fyr : %2.3f\n\n',mean_RMSE_Fyr/(max(Fyr)-min(Fyr)));
% % 
% % error_Myf = (Myf-MyF).^2;
% % RMSE_Myf  = error_Myf.^0.5;
% % mean_RMSE_Myf = mean(RMSE_Myf);
% % NMSE.Myf=mean_RMSE_Myf/(max(Myf)-min(Myf));
% % fprintf('Modelling Error RMSE in Myf : %2.3f\n\n',mean_RMSE_Myf/(max(Myf)-min(Myf)));
% % 
% % error_Myr = (Myr-MyR).^2;
% % RMSE_Myr  = error_Myr.^0.5;
% % mean_RMSE_Myr = mean(RMSE_Myr);
% % NMSE.Myr=mean_RMSE_Myr/(max(Myr)-min(Myr));
% % fprintf('Modelling Error RMSE in Myr : %2.3f\n\n',mean_RMSE_Myr/(max(Myr)-min(Myr)));
% % 
% % error_Ay = (Ay*9.8-ALat).^2;
% % RMSE_Ay  = error_Ay.^0.5;
% % mean_RMSE_Ay = mean(RMSE_Ay);
% % NMSE.Ay=mean_RMSE_Ay/(max(AY)-min(AY));
% % fprintf('Modelling Error RMSE in Ay : %2.3f\n\n',mean_RMSE_Ay/(max(AY)-min(AY)));
% % 
% % error_YawAcc = (YawAcc-YAwAcc).^2;
% % RMSE_YawAcc  = error_YawAcc.^0.5;
% % mean_RMSE_YawAcc = mean(RMSE_YawAcc);
% % NMSE.YawAcc=mean_RMSE_YawAcc/(max(YawAcc)-min(YawAcc));
% % fprintf('Modelling Error RMSE in YawAcc : %2.3f\n\n',mean_RMSE_YawAcc/(max(YawAcc)-min(YawAcc)));
% % 
% % error_BetaR = (BetaR-Betar).^2;
% % RMSE_BetaR  = error_BetaR.^0.5;
% % mean_RMSE_BetaR = mean(RMSE_BetaR);
% % NMSE.BetaR=mean_RMSE_BetaR/(max(BetaR)-min(BetaR));
% % fprintf('Modelling Error RMSE in BetaR : %2.3f\n\n',mean_RMSE_BetaR/(max(BetaR)-min(BetaR)));
% % 
% % error_Vy = (Vy-VLat).^2;
% % RMSE_Vy  = error_Vy.^0.5;
% % mean_RMSE_Vy = mean(RMSE_Vy);
% % NMSE.Vy=mean_RMSE_Vy/(max(Vy)-min(Vy));
% % fprintf('Modelling Error RMSE in Vy : %2.3f\n\n',mean_RMSE_Vy/(max(Vy)-min(Vy)));
% % 
% % error_YawR = (YawR-RYaw).^2;
% % RMSE_YawR  = error_YawR.^0.5;
% % mean_RMSE_YawR = mean(RMSE_YawR);
% % NMSE.YawR=mean_RMSE_YawR/(max(YawR)-min(YawR));
% % fprintf('Modelling Error RMSE in YawR : %2.3f\n\n',mean_RMSE_YawR/(max(YawR)-min(YawR)));
% % 
% % error_Beta = (Beta-BETA).^2;
% % RMSE_Beta  = error_Beta.^0.5;
% % mean_RMSE_Beta = mean(RMSE_Beta);
% % NMSE.Beta=mean_RMSE_Beta/(max(Beta)-min(Beta));
% % fprintf('Modelling Error RMSE in Beta : %2.3f\n\n',mean_RMSE_Beta/(max(Beta)-min(Beta)));
% 
% % NMSE.Fyf=goodnessOfFit(FyF,Fyf,'NMSE');
% % NMSE.Fyr=goodnessOfFit(FyR,Fyr,'NMSE');
% % NMSE.Myf=goodnessOfFit(MyF,Myf,'NMSE');
% % NMSE.Myr=goodnessOfFit(MyR,Myr,'NMSE');
% % NMSE.Ay=goodnessOfFit(ALat,AY,'NMSE');
% % NMSE.Vy=goodnessOfFit(VLat,Vy,'NMSE');
% % NMSE.YawR=goodnessOfFit(RYaw,YawR,'NMSE');
% % NMSE.YawAcc=goodnessOfFit(YAwAcc,YawAcc,'NMSE');
% % NMSE.Beta=goodnessOfFit(BETA,Beta,'NMSE');
% % NMSE.BetaR=goodnessOfFit(Betar,BetaR,'NMSE');
% % 
% % %fprintf('Table of Normalized Error for RUN- %2.0f',run);
% % T=struct2table(NMSE)
% 
% % loc=input('Insert Row And Column(eg. B3):');
% % filename = 'model_performance.xlsx';
% % writetable(T,filename,'Sheet',1,'Range','B1')
% 
% %writetable(T,filename,'Sheet',1,'WriteVariableNames',false,'Range','B3')
% 
% % %%%%%%%%%--------Longitudinal Force Calculation
% % Fx_l1=(1/(0.298*1.25))*(My_Dr_L1-My_Bk_L1-AAy_L1);
% % Fx_l2=(1/(0.298*1.25))*(My_Dr_L2-My_Bk_L2-AAy_L2);
% % Fx_r1=(1/(0.298*1.25))*(My_Dr_R1-My_Bk_R1-AAy_R1);
% % Fx_r2=(1/(0.298*1.25))*(My_Dr_R2-My_Bk_R2-AAy_R2);
% % %ax=(1/(m*9.8)).*(Fxf.*cosd(steerf)+Fxr-Fyf.*sind(steerf)-m.*yawrate.*Vy);
% % figure(8)
% % plot(Time,FXF,'red',Time,Fxf,'blue')
% % title('Front Tire Longitudinal Force')
% % xlabel('Time(in sec)')
% % ylabel('Fyf(in N)')
% % legend('model','carsim','location','northeast')
