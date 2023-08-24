% Initialization
%clc;clear;
%load('Params_2Cf_2Cr_15mps.mat');
T_final=time(end);
T_sample=T;

%% Code for System Modelling
A=[-(Cf+Cr)/(m*Vx), -1-(lf*Cf-lr*Cr)/(m*Vx^2);
    -(lf*Cf-lr*Cr)/Iz, -(lf^2*Cf+lr^2*Cr)/(Iz*Vx)];
B=[Cf/(m*Vx);(lf*Cf)/Iz];
C=eye(2);
D=[0];
sys=ss(A,B,C,D);
plot(step(sys,T_final))
legend('Side Slip','Yaw Rate')

%% Discretization of Continous State Space Model
sysd=c2d(ss(A,B,C,D),T_sample);
Ad=sysd.A;
Bd=sysd.B;
Cd=sysd.C;
Dd=sysd.D;
plot(step(sysd,T_final))
legend('Side Slip','Yaw Rate')

%% Testing controllability of the system
Continous_Controllability=ctrb(A,B);
Rc=rank(Continous_Controllability);
Cont_Unctrb_States=length(A)-Rc;

if Cont_Unctrb_States==0
    disp('The system is Controllable')
else
    disp('The system is Uncontrollable\n')
    disp('No. of Uncontrollable States = ',Cont_Unctrb_States)
end

%% Linear Quadratic Integral Controller Design
% In this section, the 'lqi' command is used for controller designing. The control task is to regulate  and track .
% Step 1.1: Modulating the C matrix for Tracking
C_lqi=[1 0; 0 1];

% This step is avoided due to the following error regarding matrix dimension
% Step 1.2: Choosing weights for Q and R matrices
% **Working Q=diag[10,10,100,1000] and R=100
w_slip=100;
w_yawRate=10;
w_steer=100;
w_errYawRate=1000;
w_errSlip=100;

Q_lqi=diag([w_slip,w_yawRate,w_errSlip,w_errYawRate]);
R_lqi=[w_steer];

% Step 1.3.a: LQI formulation for Continous Time State space Model.
Kc_lqi_t1=lqi(ss(A,B,C_lqi,D),Q_lqi,R_lqi);

% Step 1.3.b: LQI formulation for Discrete Time State Space Model
Kd_lqi_t1=lqi(ss(Ad,Bd,C_lqi,D),Q_lqi,R_lqi);
%%
% Type-2
% Here, the  augmented model has been designed manually and the lqi is realized using LQR command. The augmented model is given as;
%  where ' r ' is the reference.
% Step 2.1: Modulating the C matrix for tracking1
%  (Same as Step 1.1)
% Step 2.2: Designing Augmented System Model
Ac_aug = [A zeros(length(A),length(C)); -C zeros(length(C_lqi))];
Bc_aug =[B;zeros(length(C_lqi),1)];

% Step:2.3: Choosing weights for Q and R matrices
%  (Same as Step 1.2)
% Step 2.4: Formulation LQR Control using Augmented System (thereby formiing LQI)
Kc_aug_lqi=lqr(Ac_aug,Bc_aug,Q_lqi,R_lqi);
%%
% Type-3
% Here, the LQR formulation is done by solving the ARE for the Augmented System
% The function [X,L,G] = care(A,B,Q,R) is used.
% The said ARE is given as; ; with and , it can be reduced to;
% The formulation is given below
% Step 3.1, 3.2 and 3.3 are the same as Step 2.1, 2.2 and 2.3 respectively.
% Step 3.4:
P=care(Ac_aug,Bc_aug,Q_lqi,R_lqi);
Kc_areSolve=R_lqi\Bc_aug'*P;

% Simulation using Simulink model and Plot of system response with designed controller
sim('LQR_Lateral_Stability')
subplot(2,1,1)
plot(simTime,slipRef,'r.-',simTime,slipOut,'b-')
subplot(2,1,2)
plot(simTime,yawRef,'r.-',simTime,yawOut,'b-')

