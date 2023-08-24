% clear all;
% clc;
% echo off;
% 
% run CarSim_Initialize.m;

% fprintf('-------------------------------------------------\n\n');
% fprintf('In this demo the UKF is used to estimate a simple\n');
% fprintf('nonlinear scalar time series.\n\n');
% 
% number_of_runs = input('Number of runs : ');
% 
% mean_RMSE_ukf1 = zeros(1,number_of_runs);



%for j=1:number_of_runs
rYaw.N = length(Time);                  % number of time steps
rYaw.Ts=Time(2);                        %Sampling Time

rYaw.Q    = 0.1;                       % process noise variance  
rYaw.R    = 0.1;                       % measurement noise variance
n      = sqrt(rYaw.R)*randn(1,rYaw.N);    % measurement noise

rYaw.trueS=YawR;                        %True State

rYaw.x0  = rYaw.trueS(1);                     % initial state 
rYaw.P0  = 1;                          % initial state covariance 

rYaw.L = size(rYaw.x0,1);               % state dimension

rYaw.x  = zeros(rYaw.L,rYaw.N+1);       % state estimate buffer
P   = zeros(rYaw.L,rYaw.L,1,rYaw.N+1);  % state covariance buffer

rYaw.x(:,1)    = rYaw.x0;               % initialize buffers
rYaw.Px(:,:,1)  = rYaw.P0;

rYaw.ukf1 = rYaw.x;                      % create UKF buffers from template    
rYaw.P_ukf1  = rYaw.Px;

rYaw.ukf2 = rYaw.x;                      % create UKF buffers from template    
rYaw.P_ukf2  = rYaw.Px;

rYaw.meas1=AY+n';                           % Measurement from Sensor
rYaw.meas2=YawR+n';

rYaw.tune1=[0.5 10 5];               %Tune Parameters
rYaw.tune2=[0.1 100 2];

rYaw.constS=[Iz lf lr];             %Constants for State Equation
rYaw.constM1=m;                      %Constants for Measurement Equation
rYaw.constM2=[];             %Constants for Measurement Equation

%%-------------------------------------------------------------------
%%---------------------- GENERATE DATASET ---------------------------

fprintf('\nGenerating data...\n');

rYaw.x      = zeros(rYaw.L,rYaw.N+1);
%rYaw.y      = zeros(1,rYaw.N+1);

rYaw.x(:,1) = rYaw.x0;                  % initial state condition

%rYaw.y(:,1) = rYawMeas1([],rYaw.x(:,1),[Fyf(1) Fyr(1) delta(1) Vx(1)],m); % initial onbservation of state

for k=2:(rYaw.N+1)
  rYaw.x(:,k) = rYawState(rYaw.Ts,rYaw.x(:,k-1),[Fyf(k-1) Fyr(k-1) delta(k-1)],rYaw.constS);
 % rYaw.y(:,k) = feval('rYawMeas1',[],rYaw.x(:,1),[Fyf(k-1) Fyr(k-1) delta(k-1) Vx(k-1)],m);
end

%%-------------------------------------------------------------------
%%------------------- ESTIMATE STATE USING UKF ----------------------
fprintf('\nEstimating trajectory...\n');

for k=2:(rYaw.N)
 
    rYaw.inpS=[Fyf(k-1) Fyr(k-1) delta(k-1)]; %input for State Equation
    rYaw.inpM1=[Fyf(k-1) Fyr(k-1) delta(k-1) Vx(k-1)];
    rYaw.inpM2=[];
  
   % Generate UKF estimate
  [rYaw.ukf1(:,k),rYaw.P(:,k)] = ukf(rYaw.tune1,'rYawState',rYaw.constS,rYaw.inpS,rYaw.x0,rYaw.P0,'rYawMeas2',rYaw.meas2(k),rYaw.inpM1,rYaw.constM1,rYaw.R,rYaw.Q,rYaw.Ts);
  %[rYaw.ukf2(:,k)] = ukf(rYaw.tune2,'rYawState',rYaw.constS,rYaw.inpS,rYaw.x0,rYaw.P0,'rYawMeas2',rYaw.meas2(k),rYaw.inpM2,rYaw.constM2,rYaw.R,rYaw.Q,rYaw.Ts);
                    %ukf(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 
end

%%-------------------------------------------------------------------
%%------------------- DISPLAY RESULTS -------------------------------
% 
% figure(1); clf;
% %p1 = plot(rYaw.meas2(2:end),'.','linewidth',1.5); hold on
% p2 = plot(Time,rYaw.trueS,'b-');hold on
% p3 = plot(Time,rYaw.ukf1(2:end),'r.-');
% %p4 = plot(rYaw.ukf2(2:end),'r.-'); hold off
% legend([p2 p3],'CarSim','Estimation');
% xlabel('Time (sec)');
% ylabel('Yaw Rate(rad/s)')
% axis([0 Time(length(Time)) -0.7 0.8]);
% %title('Estimation of Yaw Rate');
% drawnow
% Est_rYaw=rYaw.ukf1(2:end)';
% True_rYaw=rYaw.trueS;
%NMSE_rYaw(j)=goodnessOfFit(Est_rYaw,True_rYaw,'MSE');
%end
% Est_rYaw=rYaw.ukf1(2:end)';
% True_rYaw=rYaw.trueS;
% NMSE_rYaw=goodnessOfFit(Est_rYaw,True_rYaw,'NMSE');
% filename='results_Estimation_rYaw_DLC.mat';
% save(filename,'Est_rYaw','True_rYaw','NMSE_rYaw','delta')
% 
% fprintf('\n\n');
% fprintf('---------------------------------------------------------\n');
% fprintf('Mean & Variance of normalized RMSE over %d runs\n\n',number_of_runs);
% fprintf('UKF : %2.4f (%2.4f)\n\n',NMSE_rYaw);
% %fprintf('UKF : %2.4f (%2.4f)\n\n',mean(mean_RMSE_ukf2/(max(rYaw.trueS)-min(rYaw.trueS))),var(mean_RMSE_ukf2/var(rYaw.x)));
% fprintf('---------------------------------------------------------\n');