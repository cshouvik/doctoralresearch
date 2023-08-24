% clear all;
% clc;
% echo off;
% 
% run CarSim_Initialize.m;

% fprintf('-------------------------------------------------\n\n');
% fprintf('In this demo the UKF is used to estimate a simple\n');
% fprintf('nonlinear scalar time series.\n\n');
% 
%number_of_runs = input('Number of runs : ');
% 
% mean_RMSE_ukf = zeros(1,number_of_runs);



%for j=1:number_of_runs
VLateral.N = length(Time);                  % number of time steps
VLateral.Ts=Time(2);                        %Sampling Time

VLateral.Q    = 0.1;                       % process noise variance  
VLateral.R    = 0.1;                       % measurement noise variance
% n      = sqrt(VLateral.R)*randn(1,VLateral.N);    % measurement noise

VLateral.trueS=Vy;                        %True State

VLateral.x0  = VLateral.trueS(1);                     % initial state 
VLateral.P0  = 100;                          % initial state covariance 

VLateral.L = size(VLateral.x0,1);               % state dimension

VLateral.x  = zeros(VLateral.L,VLateral.N+1);       % state estimate buffer
P   = zeros(VLateral.L,VLateral.L,1,VLateral.N+1);  % state covariance buffer

VLateral.x(:,1)    = VLateral.x0;               % initialize buffers
VLateral.Px(:,:,1)  = VLateral.P0;

VLateral.ukf = VLateral.x;                      % create UKF buffers from template    
VLateral.P_ukf  = VLateral.Px;

VLateral.meas=Vx+n';                           % Measurement from Sensor

VLateral.tune1=[0.5 -1 0];               %Tune Parameters

VLateral.constS=g;             %Constants for State Equation
VLateral.constM=[];                      %Constants for Measurement Equation

%%-------------------------------------------------------------------
%%---------------------- GENERATE DATASET ---------------------------

fprintf('\nGenerating data...\n');

VLateral.x      = zeros(VLateral.L,VLateral.N+1);
VLateral.y      = zeros(1,VLateral.N+1);

VLateral.x(:,1) = VLateral.x0;                  % initial state condition

VLateral.y(:,1) = VyMeas2([],VLateral.x(:,1),[Beta],0); % initial onbservation of state

for k=2:(VLateral.N+1)
  VLateral.x(:,k) = VyState(VLateral.Ts,VLateral.x(:,k-1),[AY(k-1) phi(k-1) Vx(k-1) YawR(k-1)],VLateral.constS);
  VLateral.y(:,k) = VyMeas2([],VLateral.x(:,1),[Beta],0);
end

%%-------------------------------------------------------------------
%%------------------- ESTIMATE STATE USING UKF ----------------------
fprintf('\nEstimating trajectory...\n');

for k=2:(VLateral.N),
 
    VLateral.inpS=[AY(k-1) phi(k-1) Vx(k-1) YawR(k-1)]; %input for State Equation
    VLateral.inpM=[Beta(k)];
  
   % Generate UKF estimate
  [VLateral.ukf(:,k),VLateral.P(:,k)] = ukf(VLateral.tune1,'VyState',VLateral.constS,VLateral.inpS,VLateral.x0,VLateral.P0,'VyMeas2',VLateral.meas(k),VLateral.inpM,VLateral.constM,VLateral.R,VLateral.Q,VLateral.Ts);
                    %ukf(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 
end


%%------------------- DISPLAY RESULTS -------------------------------

% figure(1); clf;
% p2 = plot(Time,VLateral.trueS,'b-');hold on
% p3 = plot(Time,VLateral.ukf(2:end),'r.-'); hold off
% legend([p2 p3],'CarSim','Estimation');
% xlabel('Time (sec)');
% ylabel('Vy(m/s)')
% axis([0 Time(length(Time)) -0.4 0.35]);
% %title('Estimation of Lateral Velocity');
% drawnow
% Est_Vy=VLateral.ukf(2:end)';
% True_Vy=VLateral.trueS;
% NMSE_Vy(j)=goodnessOfFit(Est_Vy,True_Vy,'MSE');
% end
% Est_Vy=VLateral.ukf(2:end)';
% True_Vy=VLateral.trueS;
% NMSE_Vy=goodnessOfFit(Est_Vy,True_Vy,'NMSE');
% filename='results_Estimation_Vy_DLC.mat';
% save(filename,'Est_Vy','True_Vy','NMSE_Vy','delta')
% 
% fprintf('\n\n');
% fprintf('---------------------------------------------------------\n');
% fprintf('Mean & Variance of normalized RMSE over %d runs\n\n',number_of_runs);
% fprintf('UKF : %2.4f (%2.4f)\n\n',NMSE_Vy);
% fprintf('---------------------------------------------------------\n');