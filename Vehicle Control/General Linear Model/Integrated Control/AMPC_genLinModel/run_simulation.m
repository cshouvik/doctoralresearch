%% Readme
%   1. Run this file
%   2. Run AMPC_intCon.slx
%   3. Run errorCalc.m
%%
clc
clear all
%% Load Dataset
uiopen

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
desBeta = zeros(321,1);

%% Load MPC Object data
load('AdaptivempcObject.mat');

%% Generate desired values
%load('DLC_u85_v60_Cf_Cr_Reference.mat', 'BetaRef_Filtered', 'YawRateRef_Filtered');
BetaRef_Filtered = BetaRef_Filtered(1:length(Time));
YawRateRef_Filtered = YawRateRef_Filtered(1:length(Time));
%% Run Simulink File
%sim('AMPC_NewNN_LinMod.slx', Time(end))
