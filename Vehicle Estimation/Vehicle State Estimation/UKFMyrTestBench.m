MyRear.N = length(Time);                  % number of time steps
MyRear.Ts=Time(2);                        %Sampling Time

MyRear.Q    = 0.1;                       % process noise variance  
MyRear.R    = 0.1;                       % measurement noise variance
% n      = sqrt(MyRear.R)*randn(1,MyRear.N);    % measurement noise

MyRear.trueS=Myr;                        %True State

MyRear.x0  = MyRear.trueS(1);                     % initial state 
MyRear.P0  = 0.1;                          % initial state covariance 

MyRear.L = size(MyRear.x0,1);               % state dimension

MyRear.x  = zeros(MyRear.L,MyRear.N+1);       % state estimate buffer
P   = zeros(MyRear.L,MyRear.L,1,MyRear.N+1);  % state covariance buffer

MyRear.x(:,1)    = MyRear.x0;               % initialize buffers
MyRear.Px(:,:,1)  = MyRear.P0;

MyRear.ukf1 = MyRear.x;                      % create UKF buffers from template    
MyRear.P_ukf1  = MyRear.Px;

MyRear.ukf2 = MyRear.x;                      % create UKF buffers from template    
MyRear.P_ukf2  = MyRear.Px;

MyRear.meas1=AY+n';                           % Measurement from Sensor
MyRear.meas2=YawAcc+n';

MyRear.tune1=[0.1 1 30];               %Tune Parameters
MyRear.tune2=[0.1 10 2];

MyRear.constS=[Iz m lr l];             %Constants for State Equation
MyRear.constM1=m;                      %Constants for Measurement Equation
MyRear.constM2=[Iz lf lr];             %Constants for Measurement Equation

%%-------------------------------------------------------------------
%%---------------------- GENERATE DATASET ---------------------------

fprintf('\nGenerating data...\n');

MyRear.x      = zeros(MyRear.L,MyRear.N+1);
MyRear.y      = zeros(1,MyRear.N+1);

MyRear.x(:,1) = MyRear.x0;                  % initial state condition


for k=2:(MyRear.N+1)
  MyRear.x(:,k) = MyrState(MyRear.Ts,MyRear.x(:,k-1),[Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)],MyRear.constS);
end

%%-------------------------------------------------------------------
%%------------------- ESTIMATE STATE USING UKF ----------------------
fprintf('\nEstimating trajectory...\n');

for k=2:(MyRear.N)
 
    MyRear.inpS=[Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; %input for State Equation
    MyRear.inpM1=[Myr(k-1) Fyf(k-1) delta(k-1) Vx(k-1) YawR(k-1)];
    MyRear.inpM2=[Myr(k-1) Fyf(k-1) delta(k-1)];
  
   % Generate UKF estimate
  [MyRear.ukf1(:,k),MyRear.P(:,k)] = ukf(MyRear.tune1,'MyrState',MyRear.constS,MyRear.inpS,MyRear.x0,MyRear.P0,'MyrMeas',MyRear.meas1(k),MyRear.inpM1,MyRear.constM1,MyRear.R,MyRear.Q,MyRear.Ts);
  %[MyRear.ukf2(:,k)] = ukf(MyRear.tune2,'MyrState',MyRear.constS,MyRear.inpS,MyRear.x0,MyRear.P0,'MyrMeas2',MyRear.meas2(k),MyRear.inpM2,MyRear.constM2,MyRear.R,MyRear.Q,MyRear.Ts);
                    %ukf(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 
end
