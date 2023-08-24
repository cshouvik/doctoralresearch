clear
clc
close all

load('Params.mat')
T=0.1;
t_final = length(steer)*T;
t=T:T:t_final;
%% ************************************************************************
%*************************** SYSTEM MODELLING *****************************
% Inputs:   params => [m, lf, lr, Iz] m-> Mass (kg), lf,lr-> distance between  
%                                     axles and CG (m), Iz = Yaw Moment of
%                                     inertia accross (Kg m^2)
%           steer  => delta (Steering angle)(rad)
%           Cf     => Cornering Stiffness of front tire
%           Cr     => Cornering Stiffness of rear tire
%           Vx     => Vx Longitudinal Velocity (m/s)
%           Vy     => Vy Lateral Velocity (m/s)
%           r      => Yaw Rate (rad/s)
%           p      => Yaw Angle (rad)
%           s      => Slip Angle (rad)
%  Outputs: State matrices (A,B,C,D)
% 
% 
%         [-(Cf+Cr)/(m*Vx)         ((lr*Cr-lf*Cf)/(m*Vx))-Vx        0          0] [Vy]    [   Cf/m  ]
% dX/dt = [(lr*Cr-lf*Cf)/(Iz*Vx)  -(lr^2*Cr+lf^2*Cf)/(Iz*Vx)        0          0]*[r ] +  [lf*Cf/iz ]*delta, 
%         [     0                  ((lr*Cr-lf*Cf)/(m*Vx))-1   -(Cf+Cr)/(m*Vx)  0] [s ]    [Cf/(m*Vx)]
%
%         [1    0     0     0] [Vy]
%     Y = [0    1     0     0]*[r ]
%         [0    0     1     0] [s ]
%**************************************************************************

% % CONSTANTS
% m = params(1);
% lf = params(2);
% lr = params(3);
% Iz = params(4);

%% ************************************************************************
%*************************** For Continous System *************************
% To track Position, Yaw and Yaw Rate
% STATE SPACE FORMULATION

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

A = [0 1 0 0 0; 0 a11 0 a12 0;
    0 0 0 1 0; 0 a21 0 a22 0;
    0 0 0 a32 a33];
 
% 'B' MATRIX FORMULATION

b1 = Cf/m;
b2 = lf*Cf/Iz;
b3 = Cf/(m*Vx);


B = [0;b1;0;b2;b3];

% 'C' MATRIX FORMULATION

C = [1 0 0 0 0; 0 0 0 1 0];

% 'D' MATRIX FORMULATION 

D = [0;0];

% STATE SPACE OUTPUT

sys = ss(A,B,C,D);
 %% Error Augmentation for Tracking
% At = [A zeros(5,2); C ones(2,2)];
% Bt =[B;D];
% 
% %% Defining Weights
% Qc=diag([10e-4 10e-4 10e-4 10e4 100 10000 10000]);Rc=[10000]; % same number as state variables
% %Rc = 100; % same number as control input
% 
% %% Running LQR Algorithm
% [Kc, Pc, Ec] = dlqr(At,Bt,Qc,Rc);
% 
% %% Calling Simulink For Validation
% Kc_lqr=Kc(1:5);
% Kic=Kc(6:7);

%% Using LQI Command
%solving for optimal gain K
%Q=diag([0.01 10 1000 10000 100 100000000 10000]);R=[10000000000];%Provides
%good position tracking
Q=diag([1/1000000 1 1/100 0.2/1000 0 1 1]);R=[10];%For Test
K_lqi=lqi(sys,Q,R);

%% ************************************************************************
%*************************** For Discrete System **************************
% %% STATE SPACE DISCRITIZATION
% sysd = c2d(ss(A,B,C,D),T);
% Ad = sysd.a;
% Bd = sysd.b;
% Cd = sysd.c;
% Dd = sysd.d;
% 
% %% Error Augmentation for Tracking
% At = [Ad zeros(3,1); Cd 1];
% Bt =[Bd;Dd];
% 
% %% Defining Weights
% wd_Vy = 1;
% wd_r =  1;
% wd_s = 10;
% %err_Vy = 10e6;
% errd_r = 10e6;
% %err_s = 10e6;
% Qd=diag([wd_Vy wd_r wd_s errd_r]); % same number as state variables
% Rd = 0.1; % same number as control input
% 
% %% Running LQR Algorithm
% [Kd, Pd, Ed] = dlqr(At,Bt,Qd,Rd);
% 
% %% New System
% %sys_new = ss((Ad-Bd*Kd(1:3)),Bd,Cd,0);
% %y=step(sys_new);
% 
% %% Calling Simulink For Validation
% K_lqr=Kd(1:3);
% Ki=Kd(4);
% 
% %% GENERATING REFERENCE
% %Vy_des=[t' zeros(length(steer),1)];
% 
% rdes=((Vx.*steer)/(lf+lr+((lr*Cr-lf*Cf)*m*Vx^2)/2*Cf*Cr*(lf+lr)));
% r_des=[t' rdes];
% 
% %s_des=[t' zeros(length(steer),1)];
% 
% %% CALLING SIMULINK MODEL
% sim('LQR_Reference_Tracking_Constant_Vx_Cf_Cr_Sim')
% 
% figure(2)
% subplot(2,2,1)
% plot(t,controlEffort(2:end))
% subplot(2,2,2)
% plot(t,r_track(2:end),'--',t,rdes)
% subplot(2,2,3)
% plot(t,Vy_track(2:end))
% subplot(2,2,4)
% plot(t,s_track(2:end))

