% clear all;
% clc;
% echo off;
% 
% run CarSim_Initialize.m;
% 
% fprintf('-------------------------------------------------\n\n');
% fprintf('In this demo the UKF is used to estimate a simple\n');
% fprintf('nonlinear scalar time series.\n\n');
% 
% number_of_runs = input('Number of runs : ');
% 
% mean_RMSE_ukf = zeros(1,number_of_runs);

VY=Vy;

%for j=1:number_of_runs
slipangle.N = length(Time);                  % number of time steps
slipangle.Ts=Time(2);                        %Sampling Time

slipangle.Q    = 100;                       % process noise variance  
slipangle.R    = 0.001;                       % measurement noise variance
n      = slipangle.R*sqrt(slipangle.R)*randn(1,slipangle.N);    % measurement noise

slipangle.trueS=Beta;                        %True State

slipangle.x0  = slipangle.trueS(1);                     % initial state 
slipangle.P0  = 100;                          % initial state covariance 

slipangle.L = size(slipangle.x0,1);               % state dimension

slipangle.x  = zeros(slipangle.L,slipangle.N+1);       % state estimate buffer
P   = zeros(slipangle.L,slipangle.L,1,slipangle.N+1);  % state covariance buffer

slipangle.x(:,1)    = slipangle.x0;               % initialize buffers
slipangle.Px(:,:,1)  = slipangle.P0;

slipangle.ukf = slipangle.x;                      % create UKF buffers from template    
slipangle.P_ukf  = slipangle.Px;

slipangle.meas=VY+n';                           % Measurement from Sensor

slipangle.tune1=[0.5 100 20];               %Tune Parameters

slipangle.constS=[g m];             %Constants for State Equation
slipangle.constM=[];                      %Constants for Measurement Equation




%%-------------------------------------------------------------------
%%---------------------- GENERATE DATASET ---------------------------

fprintf('\nGenerating data...\n');

slipangle.x      = zeros(slipangle.L,slipangle.N+1);
slipangle.y      = zeros(1,slipangle.N+1);

slipangle.x(:,1) = slipangle.x0;                  % initial state condition

slipangle.y(:,1) = feval('BetaMeas',[],slipangle.x(:,1),VY,0); % initial onbservation of state

for k=2:(slipangle.N+1),
  slipangle.x(:,k) = feval('BetaState',slipangle.Ts,slipangle.x(:,k-1),[delta(k-1) phi(k-1) YawR(k-1) Vx(k-1) Fyf(k-1) Fyr(k-1)],slipangle.constS);
  slipangle.y(:,k) = feval('BetaMeas',[],slipangle.x(:,k),Vx(k-1),0);
end

%%-------------------------------------------------------------------
%%------------------- ESTIMATE STATE USING UKF ----------------------
fprintf('\nEstimating trajectory...\n');

for k=2:(slipangle.N),
 
    slipangle.inpS=[delta(k-1) phi(k-1) YawR(k-1) Vx(k-1) Fyf(k-1) Fyr(k-1)]; %input for State Equation
    slipangle.inpM=[Vx(k)];
  
   % Generate UKF estimate
  [slipangle.ukf(:,k),slipangle.P(:,k)] = ukf(slipangle.tune1,'BetaState',slipangle.constS,slipangle.inpS,slipangle.x0,slipangle.P0,'BetaMeas',slipangle.meas(k),slipangle.inpM,slipangle.constM,slipangle.R,slipangle.Q,slipangle.Ts);
                    %ukf(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 
end

% %%-------------------------------------------------------------------
% %%------------------- DISPLAY RESULTS -------------------------------
% 
% figure(1); clf;
% p2 = plot(Time,slipangle.trueS,'b-');hold on
% p3 = plot(Time,slipangle.ukf(2:end),'r.-'); hold off
% legend([p2 p3],'CarSim','Estimation');
% xlabel('Time (sec)');
% ylabel('Beta(rad/s)');
% axis([0 Time(length(Time)) -0.025 0.025]);
% %title('Estimation of Body Slip Angle');
% % drawnow
% Est_Beta=slipangle.ukf(2:end)';
% True_Beta=slipangle.trueS;
% NMSE_Beta2(j)=goodnessOfFit(Est_Beta,True_Beta,'NMSE');
% end
% Est_Beta=slipangle.ukf(2:end)';
% True_Beta=slipangle.trueS;
% NMSE_Beta=goodnessOfFit(Est_Beta,True_Beta,'NMSE');
% filename='results_Estimation_Beta_DLC.mat';
% save(filename,'Est_Beta','True_Beta','NMSE_Beta','delta')
% 
% fprintf('\n\n');
% fprintf('---------------------------------------------------------\n');
% fprintf('Mean & Variance of normalized RMSE over %d runs\n\n',number_of_runs);
% fprintf('UKF : %2.4f (%2.4f)\n\n',NMSE_Beta);
% fprintf('---------------------------------------------------------\n');