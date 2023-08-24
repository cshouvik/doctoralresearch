clear
clc
close all

load('Params.mat')
T=0.025;
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
%*************************** For Continous System **************************
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

A = [a11 a12 a13;
     a21 a22 a23;
     a31 a32 a33];
 
% 'B' MATRIX FORMULATION

b1 = Cf/m;
b2 = lf*Cf/Iz;
b3 = Cf/(m*Vx);


B = [b1;b2;b3];

% 'C' MATRIX FORMULATION

C = [0 1 0];

% 'D' MATRIX FORMULATION 

D = [0];

% STATE SPACE OUTPUT

sys = ss(A,B,C,D);
%% Error Augmentation for Tracking
At = [A zeros(3,1); C 1];
Bt =[B;D];

%% Defining Weights
wc_Vy = 1;
wc_r =  1;
wc_s = 10;
%err_Vy = 10e6;
errc_r = 10e6;
%err_s = 10e6;
Qc=diag([wc_Vy wc_r wc_s errc_r]); % same number as state variables
Rc = 0.1; % same number as control input

%% Running LQR Algorithm
[Kc, Pc, Ec] = dlqr(At,Bt,Qc,Rc);

%% Calling Simulink For Validation
Kc_lqr=Kc(1:3);
Kic=Kc(4);

%% Using LQI Command
%solving for optimal gain K
Q=diag([1,1,10,1]);R=[.1];
K_lqi=lqi(sys,Q,R);

%% ************************************************************************
%*************************** For Discrete System **************************
%% STATE SPACE DISCRITIZATION
sysd = c2d(ss(A,B,C,D),T);
Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;
Dd = sysd.d;

%% Error Augmentation for Tracking
At = [Ad zeros(3,1); Cd 1];
Bt =[Bd;Dd];

%% Defining Weights
wd_Vy = 1;
wd_r =  1;
wd_s = 10;
%err_Vy = 10e6;
errd_r = 10e6;
%err_s = 10e6;
Qd=diag([wd_Vy wd_r wd_s errd_r]); % same number as state variables
Rd = 0.1; % same number as control input

%% Running LQR Algorithm
[Kd, Pd, Ed] = dlqr(At,Bt,Qd,Rd);

%% New System
%sys_new = ss((Ad-Bd*Kd(1:3)),Bd,Cd,0);
%y=step(sys_new);

%% Calling Simulink For Validation
K_lqr=Kd(1:3);
Ki=Kd(4);

%% GENERATING REFERENCE
%Vy_des=[t' zeros(length(steer),1)];

rdes=((Vx.*steer)/(lf+lr+((lr*Cr-lf*Cf)*m*Vx^2)/2*Cf*Cr*(lf+lr)));
r_des=[t' rdes];

%s_des=[t' zeros(length(steer),1)];

%% CALLING SIMULINK MODEL
sim('LQR_Reference_Tracking_Constant_Vx_Cf_Cr_Sim')

figure(2)
subplot(2,2,1)
plot(t,controlEffort(2:end))
subplot(2,2,2)
plot(t,r_track(2:end),'--',t,rdes)
subplot(2,2,3)
plot(t,Vy_track(2:end))
subplot(2,2,4)
plot(t,s_track(2:end))

