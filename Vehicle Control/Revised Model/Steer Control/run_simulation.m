%% Readme
%   1. Run this file
%   2. Run AMPC_intCon.slx
%   3. Run errorCalc.m
%%
clc
clear all
%% Load Dataset
uiopen

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

Cf = mean_Cf_fit;
Cr = mean_Cr_fit;
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
%% Load MPC Object data
load('mpc_object_2.mat');

%% Generate desired values
load('reference.mat', 'Beta_ref', 'YawR_ref');

tl = length(Beta_ref);
%% Variable downsampling
Vx = mean(Vx);
Cf = mean(Cf);
Cr = mean(Cr);
YawR = YawR(1:tl);
Beta = Beta(1:tl);
Time = Time(1:tl);
%% Generate State Space Model
run(model_generate.m)
%% Run Simulink File
%sim('AMPC_NewNN_LinMod.slx', Time(end))
