%% -----------------------------------------------------------------------
% ---------------------- Unscented Kalman Filter -------------------------
%-------------------------------------------------------------------------
clc
iteration = 10;
VY = Vy;    % Match the variable
%% ------------------------ Modules Only ---------------------------------
% ------------------------------------------------------------------------
% ---------------------------Estimation of Vy-----------------------------
tic
VLateral.N = length(Time);                 % number of time steps
VLateral.Ts=Time(2);                       %Sampling Time
VLateral.Q    = 0.001;                       % process noise variance  
VLateral.R    = scale*0.001;                       % measurement noise variance
n      = mul*Vx.*randn(1,VLateral.N);    % measurement noise
VLateral.trueS=Vy;                          %True State
VLateral.x0  = VLateral.trueS(1);            % initial state 
VLateral.P0  = 1;                          % initial state covariance 
VLateral.L = size(VLateral.x0,1);            % state dimension
VLateral.x  = zeros(VLateral.L,VLateral.N+1);     % state estimate buffer
P   = zeros(VLateral.L,VLateral.L,1,VLateral.N+1);% state covariance buffer
VLateral.x(:,1)    = VLateral.x0;   % initialize buffers
VLateral.Px(:,:,1)  = VLateral.P0;
VLateral.ukf = VLateral.x;          % create UKF buffers from template    
VLateral.P_ukf  = VLateral.Px;
VLateral.meas=Vx'+n;                 % Measurement from Sensor
VLateral.meas=AY'+n;  
VLateral.tune1=[0.001 1000 0];          %Tune Parameters
VLateral.constS=g;                  %Constants for State Equation
VLateral.constM=[];                 %Constants for Measurement Equation

for k=2:(VLateral.N)
 
    VLateral.inpS=[AY(k-1) phi(k-1) Vx(k-1) YawR(k-1)]; 
    VLateral.inpM=[Beta(k)];
    %VLateral.inpM=[VLateral.ukf(:,k-1), Vx(k), YawR(k)];
  
   % Generate UKF estimate
  [VLateral.ukf(:,k)] = ukf(VLateral.tune1,'VyState',VLateral.constS,...
      VLateral.inpS,VLateral.x0,VLateral.P0,'VyMeas2',VLateral.meas(k),...
      VLateral.inpM,VLateral.constM,VLateral.R,VLateral.Q,VLateral.Ts);
end
modules.ukf.elapsedTime.Vy=toc;
modules.ukf.Vy=-VLateral.ukf;
%% ---------------------------Estimation of Myf----------------------------
tic
MLatFront.N = length(Time);                  % number of time steps
MLatFront.Ts=Time(2);                        %Sampling Time
MLatFront.Q    = 0.1;                       % process noise variance  
MLatFront.R    = scale*0.1;                       % measurement noise variance
n      = mul*AY.*randn(1,MLatFront.N);    % measurement noise
MLatFront.trueS=Myf;                        %True State
MLatFront.x0  = MLatFront.trueS(1);                     % initial state 
MLatFront.P0  = 1;                          % initial state covariance 
MLatFront.L = size(MLatFront.x0,1);               % state dimension
MLatFront.x  = zeros(MLatFront.L,MLatFront.N+1);       % state estimate buffer
P   = zeros(MLatFront.L,MLatFront.L,1,MLatFront.N+1);  % state covariance buffer
MLatFront.x(:,1)    = MLatFront.x0;               % initialize buffers
MLatFront.Px(:,:,1)  = MLatFront.P0;
MLatFront.ukf1 = MLatFront.x;                      % create UKF buffers from template    
MLatFront.P_ukf1  = MLatFront.Px;
MLatFront.ukf2 = MLatFront.x;                      % create UKF buffers from template    
MLatFront.P_ukf2  = MLatFront.Px;
MLatFront.meas1=AY+n';                           % Measurement from Sensor
MLatFront.meas2=YawAcc+n';
MLatFront.tune1=[0.5 10 5];               %Tune Parameters
MLatFront.tune2=[0.1 100 2];
MLatFront.constS=[Iz m lr l];             %Constants for State Equation
MLatFront.constM1=m;                      %Constants for Measurement Equation
MLatFront.constM2=[Iz lf lr];             %Constants for Measurement Equation

for k=2:(MLatFront.N)
 
    MLatFront.inpS=[delta(k-1) Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; %input for State Equation
    MLatFront.inpM1=[Myf(k-1) Fyr(k-1) delta(k-1) Vx(k-1) YawR(k-1)];
    MLatFront.inpM2=[Myf(k-1) Fyr(k-1) delta(k-1)];
  
   % Generate UKF estimate
  [MLatFront.ukf(:,k)] = ukf(MLatFront.tune1,'MyfState',MLatFront.constS,MLatFront.inpS,MLatFront.x0,MLatFront.P0,'MyfMeas1',MLatFront.meas1(k),MLatFront.inpM1,MLatFront.constM1,MLatFront.R,MLatFront.Q,MLatFront.Ts);
  %[MLatFront.ukf2(:,k)] = ukfmod(MLatFront.tune2,'MyfState',MLatFront.constS,MLatFront.inpS,MLatFront.x0,MLatFront.P0,'MyfMeas2',MLatFront.meas2(k),MLatFront.inpM2,MLatFront.constM2,MLatFront.R,MLatFront.Q,MLatFront.Ts);
                    %ukfmod(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 
end
modules.ukf.elapsedTime.Myf=toc;
modules.ukf.Myf=MLatFront.ukf;

%----------------------- Estimation of Myr ------------------------------
tic
MyRear.N = length(Time);                  % number of time steps
MyRear.Ts=Time(2);                        %Sampling Time
MyRear.Q    = 0.1;                       % process noise variance  
MyRear.R    = scale*0.1;                       % measurement noise variance
n      = mul*AY.*randn(1,MyRear.N);    % measurement noise
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

for k=2:(MyRear.N)
 
    MyRear.inpS=[Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; %input for State Equation
    MyRear.inpM1=[Myr(k-1) Fyf(k-1) delta(k-1) Vx(k-1) YawR(k-1)];
    MyRear.inpM2=[Myr(k-1) Fyf(k-1) delta(k-1)];
  
   % Generate UKF estimate
  [MyRear.ukf(:,k),MyRear.P(:,k)] = ukf(MyRear.tune1,'MyrState',...
      MyRear.constS,MyRear.inpS,MyRear.x0,MyRear.P0,'MyrMeas',...
      MyRear.meas1(k),MyRear.inpM1,MyRear.constM1,MyRear.R,...
      MyRear.Q,MyRear.Ts);
  %[MyRear.ukf2(:,k)] = ukfmod(MyRear.tune2,'MyrState',MyRear.constS,MyRear.inpS,MyRear.x0,MyRear.P0,'MyrMeas2',MyRear.meas2(k),MyRear.inpM2,MyRear.constM2,MyRear.R,MyRear.Q,MyRear.Ts);
                    %ukfmod(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 
end
modules.ukf.elapsedTime.Myr=toc;
modules.ukf.Myr=MyRear.ukf;

% ------------------------- Tire Force ---------------------------------
ukfEstMLatFront = modules.ukf.Myf;
ukfEstMLatRear = modules.ukf.Myr;
modules.ukf.Fyf=[0;diff(ukfEstMLatFront')]/Ts;
modules.ukf.Fyr=[0;diff(ukfEstMLatRear')]/Ts;


%---------------------- Estimation of Yaw Rate---------------------------
tic
rYaw.N = length(Time);                  % number of time steps
rYaw.Ts=Time(2);                        %Sampling Time
rYaw.Q    = 0.1;                       % process noise variance  
rYaw.R    = scale*0.1;                       % measurement noise variance
n      = mul*AY.*randn(1,rYaw.N);    % measurement noise
rYaw.trueS=YawR;                        %True State
rYaw.x0  = rYaw.trueS(1);                     % initial state 
rYaw.P0  = 10;                          % initial state covariance 
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
for k=2:(rYaw.N)
 
    rYaw.inpS=[Fyf(k-1) Fyr(k-1) delta(k-1)]; %input for State Equation
    rYaw.inpM1=[Fyf(k-1) Fyr(k-1) delta(k-1) Vx(k-1)];
    rYaw.inpM2=[];
  
   % Generate UKF estimate
  [rYaw.ukf(:,k),rYaw.P(:,k)] = ukf(rYaw.tune1,'rYawState',rYaw.constS,...
      rYaw.inpS,rYaw.x0,rYaw.P0,'rYawMeas2',rYaw.meas2(k),rYaw.inpM1,...
      rYaw.constM1,rYaw.R,rYaw.Q,rYaw.Ts);
  %[rYaw.ukf2(:,k)] = ukfmod(rYaw.tune2,'rYawState',rYaw.constS,rYaw.inpS,rYaw.x0,rYaw.P0,'rYawMeas2',rYaw.meas2(k),rYaw.inpM2,rYaw.constM2,rYaw.R,rYaw.Q,rYaw.Ts);
                    %ukfmod(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 
end
modules.ukf.elapsedTime.YawR=toc;
modules.ukf.YawR=rYaw.ukf;

%---------------------- Estimation of Slip Angle--------------------------
tic
slipangle.N = length(Time);                  % number of time steps
slipangle.Ts=Time(2);                        %Sampling Time
slipangle.Q    = 100;                       % process noise variance  
slipangle.R    = scale*0.001;                       % measurement noise variance
n      = mul*VY.*randn(1,slipangle.N);    % measurement noise
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

for k=2:(slipangle.N)
 
    slipangle.inpS=[delta(k-1) phi(k-1) YawR(k-1) Vx(k-1) Fyf(k-1) Fyr(k-1)]; %input for State Equation
    slipangle.inpM=[Vx(k)];
  
   % Generate UKF estimate
  [slipangle.ukf(:,k),slipangle.P(:,k)] = ukf(slipangle.tune1,'BetaState',...
      slipangle.constS,slipangle.inpS,slipangle.x0,slipangle.P0,...
      'BetaMeas',slipangle.meas(k),slipangle.inpM,slipangle.constM,...
      slipangle.R,slipangle.Q,slipangle.Ts);
                    %ukfmod(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 
end
modules.ukf.elapsedTime.Beta=toc;
modules.ukf.Beta=slipangle.ukf;

xYawR = modules.ukf.YawR;
xBeta = modules.ukf.Beta;
xVy = modules.ukf.Vy;
modules.ukf.YawAcc = [0; diff(xYawR)'/Ts];
modules.ukf.Betar = [0; diff(xBeta)'/Ts];
modules.ukf.Ay = [diff(xVy)'/Ts]+ Vx.*modules.ukf.YawR';
%-------------------------------------------------------------------------
%% ------------------------ UES w/o KF -----------------------------------
% ------------------------------------------------------------------------
tic
run('UES_UKF.m')
ues.noKF.ukf.elapsedTime=toc/length(Time);
ues.noKF.ukf.Beta=slipangle.ukf;
ues.noKF.ukf.Vy=-VLateral.ukf;%%Variable Independent hence same as module
ues.noKF.ukf.MyF=MyRear.ukf;
ues.noKF.ukf.MyR=MyFront.ukf;
ues.noKF.ukf.YawR=yawRate.ukf;
xYawR = ues.noKF.ukf.YawR;
xBeta = ues.noKF.ukf.Beta;
xVy = ues.noKF.ukf.Vy;
xMyf = ues.noKF.ukf.MyF;
xMyr = ues.noKF.ukf.MyR;
ues.noKF.ukf.YawAcc = [0; diff(xYawR)'/Ts];
ues.noKF.ukf.Betar = [0; diff(xBeta)'/Ts];
ues.noKF.ukf.Ay = -[diff(xVy)'/Ts]+ Vx.*ues.noKF.ukf.YawR';
ues.noKF.ukf.FyF=[0; diff(xMyf)'/Ts];
ues.noKF.ukf.FyR=[0; diff(xMyr)'/Ts];

%-------------------------------------------------------------------------
%% ------------------------ UES w/ KF -----------------------------------
% ------------------------------------------------------------------------
dt = Ts;
c1 = m*lr/l;
c2 = Iz/l;
c3 = m*lf/l;
state = [];

% ----------------Out Data Processing UES-UKF -------------------------
ukfmod_Fyf = modules.ukf.Fyf;
ukfmod_Fyr = modules.ukf.Fyr;
ukfmod_rYaw = modules.ukf.YawR;
ukfmod_aYaw = modules.ukf.YawAcc;
ukfmod_Vy = modules.ukf.Vy;
ukfmod_Ay = modules.ukf.Ay;
ukfmod_Beta = modules.ukf.Beta;
ukfmod_BetaR = modules.ukf.Betar;
%% ---------------- Initialize Kalman Filter ----------------------------
initialState = [Fyf(1); Fyr(1); Vy(1); AY(1); YawR(1); YawAcc(1); Beta(1); BetaR(1)];
initialPx = eye(8)*0.01;
Est = [];
Px = initialPx;
xhat = initialState;
Q = 0.1*eye(8);
R = scale*0.01*eye(8);
B = 0;
H = eye(8);
inp = 0;
tic

for i = 1:length(Time)-1

% --------------- System Matrix Formation -----------------------
fyfS = [0 0 0 0 c1*Vx(i) c2 0 c1*Vx(i)];        % Fyf
fyrS = [0 0 0 0 c3*Vx(i) -c2 0 c3*Vx(i)];       % Fyr
vyS = [0 0 1 dt -dt*Vx(i) 0 0 0];                       % Vy
ayS = [(1/m) (1/m) 0 0 0 0 0 0];                % Ay
rS = [0 0 0 0 1 dt 0 0];                        % YawRate
raS = [(lf/Iz) (-lr/Iz) 0 0 0 0 0 0];           % YawAcc
bS = [0 0 0 0 0 0 1 dt];                        % SlipAngle
brS = [1/(m*Vx(i)) 1/(m*Vx(i)) 0 0 -1 0 0 0];    % SlipRate
 
 
A = [fyfS; fyrS; vyS; ayS; rS; raS; bS; brS];       % Approach 1

% ------------------ Call Meas Data From Module Outputs ------------------
meas = [ukfmod_Fyf(i); ukfmod_Fyr(i); ukfmod_Vy(i); ukfmod_Ay(i); ...
    ukfmod_rYaw(i); ukfmod_aYaw(i); ukfmod_Beta(i); ukfmod_BetaR(i)];

% ------------------------Kalman Filter-----------------------------------
    X = A*xhat + B*inp;
    P = A*Px*A' + Q;
    KGain = P*H'*(H*P*H' + R)^(-1);
    xEst = X + KGain*(meas - H*X);
    KH = KGain*H;
    Pk = (eye(size(KH))-KH)*P;
    
    xhat = xEst;
    Px = Pk;
    
    estimate(:,i) = xEst;
end
ues.KF.ukf.Fyf = estimate(1,:);
ues.KF.ukf.Fyr = estimate(2,:);
ues.KF.ukf.Vy = estimate(3,:);
ues.KF.ukf.Ay = estimate(4,:);
ues.KF.ukf.YawR = estimate(5,:);
ues.KF.ukf.YawAcc = estimate(6,:);
ues.KF.ukf.Beta = estimate(7,:);
ues.KF.ukf.BetaR = estimate(8,:);
ues.KF.ukf.simTime = toc/321;

%% ---------------------- Save Results ----------------------------------
%save('EKF_DataSetName','true', 'modules','ues');
