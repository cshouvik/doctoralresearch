MyFront.N = length(Time);                  % number of time steps
MyFront.Ts=Time(2);                        %Sampling Time

MyFront.Q    = 0.1;                       % process noise variance  
MyFront.R    = 0.1;                       % measurement noise variance
n      = sqrt(MyFront.R)*randn(1,MyFront.N);    % measurement noise

MyFront.trueS=Myf;                        %True State

MyFront.x0  = MyFront.trueS(1);                     % initial state 
MyFront.P0  = 1;                          % initial state covariance 

MyFront.L = size(MyFront.x0,1);               % state dimension

MyFront.x  = zeros(MyFront.L,MyFront.N+1);       % state estimate buffer
P   = zeros(MyFront.L,MyFront.L,1,MyFront.N+1);  % state covariance buffer

MyFront.x(:,1)    = MyFront.x0;               % initialize buffers
MyFront.Px(:,:,1)  = MyFront.P0;

MyFront.ukf1 = MyFront.x;                      % create UKF buffers from template    
MyFront.P_ukf1  = MyFront.Px;

MyFront.ukf2 = MyFront.x;                      % create UKF buffers from template    
MyFront.P_ukf2  = MyFront.Px;

MyFront.meas1=AY+n';                           % Measurement from Sensor
MyFront.meas2=YawAcc+n';

MyFront.tune1=[0.5 10 5];               %Tune Parameters
MyFront.tune2=[0.1 100 2];

MyFront.constS=[Iz m lr l];             %Constants for State Equation
MyFront.constM1=m;                      %Constants for Measurement Equation
MyFront.constM2=[Iz lf lr];             %Constants for Measurement Equation

%%-------------------------------------------------------------------
%%---------------------- GENERATE DATASET ---------------------------

% fprintf('\nGenerating data...\n');
% 
% MyFront.x      = zeros(MyFront.L,MyFront.N+1);
% MyFront.y      = zeros(1,MyFront.N+1);
% 
% MyFront.x(:,1) = MyFront.x0;                  % initial state condition
% 
% 
% for k=2:(MyFront.N+1)
%   MyFront.x(:,k) = MyfState(MyFront.Ts,MyFront.x(:,k-1),[delta(k-1) Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)],MyFront.constS);
% end
% 
% %%-------------------------------------------------------------------
% %%------------------- ESTIMATE STATE USING UKF ----------------------
% fprintf('\nEstimating trajectory...\n');

for k=2:(MyFront.N)
 
    MyFront.inpS=[delta(k-1) Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; %input for State Equation
    MyFront.inpM1=[Myf(k-1) Fyr(k-1) delta(k-1) Vx(k-1) YawR(k-1)];
    MyFront.inpM2=[Myf(k-1) Fyr(k-1) delta(k-1)];
  
   % Generate UKF estimate
  [MyFront.ukf1(:,k),MyFront.P(:,k)] = ukf(MyFront.tune1,'MyfState',MyFront.constS,MyFront.inpS,MyFront.x0,MyFront.P0,'MyfMeas',MyFront.meas1(k),MyFront.inpM1,MyFront.constM1,MyFront.R,MyFront.Q,MyFront.Ts);
  
end

