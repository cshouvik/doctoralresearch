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
g=9.8;
t=1:length(Time);
AY=Ay*9.8;
phi=Roll*rad;
Tsample=Time(2)-Time(1);
Cf=Fyf./Alphaf;
Cr=Fyr./Alphar;
Cf(isnan(Cf)) = 0;
Cf=mean(Cf);
Cr(isnan(Cr)) = 0;
Cr=mean(Cr);
Vx=mean(Vx);

%% STATE SPACE FORMULATION(3 States)

% 'A' MATRIX FORMULATION

a11 = -(Cf+Cr)/(m*Vx);
a12 = ((lr*Cr-lf*Cf)/(m*Vx))-Vx;
a13 = 0;

a21 = (lr*Cr-lf*Cf)/(Iz*Vx);
a22 = -(lr^2*Cr+lf^2*Cf)/(Iz*Vx);
a23 = 0;

a31 = 0;
a32 = ((lr*Cr-lf*Cf)/(m*Vx))-1;
a33 = -(Cf+Cr)/(m*Vx);

A = [-(Cf+Cr)/(m*Vx) ((lr*Cr-lf*Cf)/(m*Vx))-Vx 0;
     (lr*Cr-lf*Cf)/(Iz*Vx) -(lr^2*Cr+lf^2*Cf)/(Iz*Vx) 0;
     0 ((lr*Cr-lf*Cf)/(m*Vx))-1 -(Cf+Cr)/(m*Vx)];
 
% 'B' MATRIX FORMULATION

b1 = Cf/m;
b2 = lf*Cf/Iz;
b3 = Cf/(m*Vx);


B = [Cf/m;lf*Cf/Iz;Cf/(m*Vx)];

% 'C' MATRIX FORMULATION

C = eye(5,5);

% 'D' MATRIX FORMULATION 

D = [0];

% STATE SPACE OUTPUT

sys = ss(A,B,C,D);

% %% STATE SPACE FORMULATION(5 States)
% 
% % 'A' MATRIX FORMULATION
% 
% a11 = -(Cf+Cr)/(m*Vx);
% a12 = ((lr*Cr-lf*Cf)/(m*Vx))-Vx;
% a13 = 0;
% 
% a21 = (lr*Cr-lf*Cf)/(Iz*Vx);
% a22 = -(lr^2*Cr+lf^2*Cf)/(Iz*Vx);
% a23 = 0;
% 
% a31 = 0;
% a32 = ((lr*Cr-lf*Cf)/(m*Vx))-1;
% a33 = -(Cf+Cr)/(m*Vx);
% 
% A = [0 1 0 0 0; 0 a11 0 a12 0;
%     0 0 0 1 0; 0 a21 0 a22 0;
%     0 0 0 a32 a33];
%  
% % 'B' MATRIX FORMULATION
% 
% b1 = Cf/m;
% b2 = lf*Cf/Iz;
% b3 = Cf/(m*Vx);
% 
% 
% B = [0;b1;0;b2;b3];
% 
% % 'C' MATRIX FORMULATION
% 
% C = [1 0 0 0 0; 0 0 1 0 0];
% 
% % 'D' MATRIX FORMULATION 
% 
% D = [0;0];
% 
% % STATE SPACE OUTPUT
% 
% sys = ss(A,B,C,D);
