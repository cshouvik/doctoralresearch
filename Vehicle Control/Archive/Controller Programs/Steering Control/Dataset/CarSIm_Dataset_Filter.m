clc
clear all
uiopen
rad=pi/180;
deg=180/pi;
Vx=Vx.*5/18;
VY=Vy.*5/18;
Yaw=Yaw;
delta=(1/2.*(Steer_L1+Steer_R1))*rad;
Fyf=(1.*(Fy_L1+Fy_R1));
Fyr=(1.*(Fy_L2+Fy_R2));
Fxf=(Fx_L1+Fx_R1);
Fxr=(Fx_L2+Fx_R2);
Alphaf=(1/2.*(Alpha_L1+Alpha_R1))*rad;
Alphar=(1/2.*(Alpha_L2+Alpha_R2))*rad;
YawR=AVz*rad;
YawAcc=AAz;
BetaR=BetaR*rad;
Beta=Beta*rad;
% Iz=input('Enter moment of inertia Iz(Nm): ');
% lf=input('Enter Value of lf(mm): ')/1000;
% lr=input('Enter Value of lr(mm): ')/1000;
% ms=input('Enter sprung mass of vehicle ms(kg): ');
%mu=input('Enter unsprung mass of vehicle mu(kg): ');
Tbrake_L1=My_Bk_L1;
Tbrake_L2=My_Bk_L2;
Tbrake_R1=My_Bk_R1;
Tbrake_R2=My_Bk_R2;
Tdrive_L1=My_Dr_L1;
Tdrive_L2=My_Dr_L2;
Tdrive_R1=My_Dr_R1;
Tdrive_R2=My_Dr_R2;
lr=(3.05-1.4)*ones(length(Time),1);
lf=1.4*ones(length(Time),1);
m=1650*ones(length(Time),1);
Iz=3234*ones(length(Time),1);
l=lf+lr;
g=9.8;
t=1:length(Time);
AY=Ay*9.8;
Ax=Ax*9.8;
phi=Roll*rad;
Tsample=Time(2)-Time(1);
mu=0.85*ones(length(Time),1);
nn_in=[mu,lf,lr,m,Iz,delta,Tbrake_L1,Tbrake_R1,Tbrake_L2,Tbrake_R2,Tdrive_L1,Tdrive_R1,Tdrive_L2,Tdrive_R2];
nn_out=[Vx,Ax,Vy,AY,Yaw,YawR,Beta,BetaR,phi,Fyf,Fyr,Fxf,Fxr];

save('DLC_u85_v60_ESC_Filtered_and_NNData.mat','mu','lf','lr','m','Iz','delta',...
      'Tbrake_L1','Tbrake_R1','Tbrake_L2','Tbrake_R2','Tdrive_L1','Tdrive_R1','Tdrive_L2','Tdrive_R2',...
      'Vx','Ax','Vy','AY','Yaw','YawR','Beta','BetaR','phi','Fyf','Fyr','Fxf','Fxr','nn_in','nn_out');
  
% clear all
% load('DLC_u85_v60_ESC_Filtered_and_NNData.mat')