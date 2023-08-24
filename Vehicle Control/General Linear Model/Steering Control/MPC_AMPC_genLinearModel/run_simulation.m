%% Readme
%   1. Run this file
%   2. Run AMPC_intCon.slx
%   3. Run errorCalc.m
%%
clc
clear all
%% Load Dataset
uiopen            % To run both AFS and IntCon load steer angle dataset...
                  % and add Mz from Integrated dataset

%%
% %% Down Sampling (** Run for split mu dataset **)
% 
% Alphaf = Alphaf(1:4:length(Time),1);
% Alphar = Alphar(1:4:length(Time),1);
% AY = AY(1:4:length(Time),1);
% Beta = Beta(1:4:length(Time),1);
% BetaR = BetaR(1:4:length(Time),1);
% Fxf = Fxf(1:4:length(Time),1);
% Fyf = Fyf(1:4:length(Time),1);
% Fyr = Fyr(1:4:length(Time),1);
% Myf = Myf(1:4:length(Time),1);
% Myr = Myr(1:4:length(Time),1);
% phi = phi(1:4:length(Time),1);
% t = t(1,1:4:length(Time));
% Vx = Vx(1:4:length(Time),1);
% Vy = Vy(1:4:length(Time),1);
% yaw = yaw(1:4:length(Time),1);
% YawAcc = YawAcc(1:4:length(Time),1);
% YawR = YawR(1:4:length(Time),1);
% Time = Time(1:4:length(Time),1); %% Should always be last to change

%% Generate Cornering stiffness value
run('CorneringStifnessGenerate.m');

mean_Cf_fit = mean(Cf_fit)*ones(length(Time),1);
mean_Cr_fit = mean(Cr_fit)*ones(length(Time),1);
Cf = mean(mean_Cf_fit);
Cr = mean(mean_Cr_fit);
%% Calculate desired values
% Kv = (m/2*l)*((lr./mean_Cf_fit)-(lf./mean_Cr_fit));
% V = sqrt(Vx.^2+Vy.^2);
% R = V.^2./AY;
% delta_ss = (1./R)+Kv.*AY;
% r_des = (Vx./(l + m*V.^2.*(lr*mean_Cr_fit - lf*mean_Cf_fit)./...
%     (2*mean_Cf_fit.*mean_Cr_fit*l))).*delta_ss;
% slip_des = ((lr-(m*V.^2.*(lf*m*V.^2))./(2*mean_Cf_fit.*mean_Cr_fit.*l))./...
%     (l + m*V.^2.*(lr*mean_Cr_fit - lf*mean_Cf_fit)./...
%     (2*mean_Cf_fit.*mean_Cr_fit*l))).*delta_ss;
V = sqrt(Vx.^2+Vy.^2);
desYawR = (V.*delta)./(lf + lr + ((m*V.^2)*((Cr*lr-Cf*lf)/(2*Cf*Cr*(lf+lr)))));
desBeta = 0.001*zeros(length(Time),1);
%% Load MPC Object data
%load('AFS_IntCon_AMPC_Object.mat');
load('AFS_IntCon_MPC_Object.mat');
%% Generate desired values
%load('DLC_u85_v60_Cf_Cr_Reference.mat', 'BetaRef_Filtered', 'YawRateRef_Filtered');
BetaRef_Filtered = desBeta;     %BetaRef_Filtered(1:length(Time));
YawRateRef_Filtered = desYawR;  %YawRateRef_Filtered(1:length(Time));
%% Run Simulink File
%sim('AMPC_NewNN_LinMod.slx', Time(end))

%% Gen MPC AFS and Integrated Model Setup
% A=[-(Cf+Cr)/(m*Vx), -1-(lf*Cf-lr*Cr)/(m*Vx^2);
%     -(lf*Cf-lr*Cr)/Iz, -(lf^2*Cf+lr^2*Cr)/(Iz*Vx)];
% Bc=[Cf/(m*Vx);(lf*Cf)/Iz];
% Bb=[Cf/(m*Vx) 0;(lf*Cf)/Iz 1/Iz];