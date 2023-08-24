% clc; clear all
% uiopen
% Iz=input('Enter moment of inertia Iz(Nm): ');
% lf=input('Enter Value of lf(mm): ')/1000;
% lr=input('Enter Value of lr(mm): ')/1000;
% ms=input('Enter sprung mass of vehicle ms(kg): ');
% mu=input('Enter unsprung mass of vehicle mu(kg): ');
% 
% % %RUN001
% % load('D:\Research\Publication New\Estimation Paper\Program\Final Revision\Estimation\all_progs\RUN001.mat')
% % Iz=750;
% % lf=1100/1000;
% % lr=1250/1000;
% % ms=750;
% % mu=41.5;
% 
% %% Initialization
% rad=pi/180; deg=180/pi; g=9.8;
% Vx=Vx.*5/18; VY=Vy.*5/18; AY=Ay*9.8;
% yaw=Yaw; phi=Roll*rad;
% delta=(1/2.*(Steer_L1+Steer_R1))*rad;
% Fyf=(1.*(Fy_L1+Fy_R1)); Fyr=(1.*(Fy_L2+Fy_R2));
% Fxf=(Fx_L1+Fx_R1); Fxr=(Fx_L2+Fx_R2);
% Alphaf=(1/2.*(Alpha_L1+Alpha_R1)); Alphar=(1/2.*(Alpha_L2+Alpha_R2));
% YawR=AVz*rad; YawAcc=AAz;
% BetaR=BetaR*rad; Beta=Beta*rad;
% m=ms+mu;
% l=lf+lr;
% t=1:length(Time);
% Tsample=Time(2);
% %% Calcultaing Myf and Myr
% TStop=Time(length(Time));
% FYF=[Time Fyf];
% FYR=[Time Fyr];
% options=simset('Solver','FixedStepDiscrete','FixedStep','Tsample');
% [x,y]=sim('myf_myr_extract',TStop,simset('Solver','ode1','FixedStep','Tsample'));
% 
% number_of_runs = 200;     %input('Number of runs : ');
% 
 N = length(Time);                  % number of time steps
 Ts=Time(2);                        %Sampling Time
 L = 1;               % state dimension
Ay = AY;
 %for p=1:number_of_runs
VLat.Q    = 0.1;                       % process noise variance  
VLat.R    = scale*0.1;                       % measurement noise variance
yawRate.Q    = 20;                       % process noise variance  
yawRate.R    = scale*0.01;                       % measurement noise variance
MyFront.Q    = 20;                       % process noise variance  
MyFront.R    = scale*0.001;                       % measurement noise variance
MyRear.Q    = 20;                       % process noise variance  
MyRear.R    = scale*0.001;                       % measurement noise variance
slipangle.Q    = 0.001;                       % process noise variance  
slipangle.R    = scale*0.0001;                       % measurement noise variance                    % measurement noise variance

% slipangle.trueS=Beta;                        %True State
% VLat.trueS=VY;                        %True State
% yawRate.trueS=YawR;                        %True State
% MyFront.trueS=Myf;                        %True State
% MyRear.trueS=Myr;                        %True State

VLat.ukf  = zeros(L,N);                    % initialize buffer 
yawRate.ukf  = zeros(L,N);                     % initialize buffer 
MyFront.ukf  = zeros(L,N);                     % initialize buffer 
MyRear.ukf  = zeros(L,N);                     % initialize buffer 
MyFront.ukf2  = zeros(L,N);                     % initialize buffer 
MyRear.ukf2  = zeros(L,N);                     % initialize buffer 
slipangle.ukf  = zeros(L,N);                     % initialize buffer 

VLat.ukf(1)  = VY(1);                     % initial state 
VLat.Pxx  = 1;                          % initial state covariance 
yawRate.ukf(1)  = YawR(1);                     % initial state 
yawRate.Pxx  = 1;                          % initial state covariance 
MyFront.ukf(1)  = Myf(1);                     % initial state 
MyFront.Pxx  = 1;                          % initial state covariance
MyRear.ukf(1)  = Myr(1);                     % initial state 
MyRear.Pxx  = 1;                          % initial state covariance
slipangle.ukf(1)  = Beta(1);                     % initial state 
slipangle.Pxx  = 1;                          % initial state covariance 


VLat.x  = zeros(L,N);       % state estimate buffer
yawRate.x  = zeros(L,N);       % state estimate buffer
MyFront.x  = zeros(L,N);       % state estimate buffer
MyRear.x  = zeros(L,N);       % state estimate buffer
slipangle.x  = zeros(L,N);       % state estimate buffer

slipangle.x(1)    = slipangle.ukf(1);               % initialize buffers
slipangle.Px(1)  = slipangle.Pxx;
VLat.x(1)    = VLat.ukf(1);               % initialize buffers
VLat.Px(1)  = VLat.Pxx;
yawRate.x(1)    = yawRate.ukf(1);               % initialize buffers
yawRate.Px(1)  = yawRate.Pxx;
MyFront.x(1)    = MyFront.ukf(1);               % initialize buffers
MyFront.Px(1)  = MyFront.Pxx;
MyRear.x(1)    = MyRear.ukf(1);               % initialize buffers
MyRear.Px(1)  = MyRear.Pxx;



n1      = sqrt(slipangle.R)*randn(1,N);    % measurement noise
n2      = sqrt(VLat.R)*randn(1,N);    % measurement noise
n3      = sqrt(yawRate.R)*randn(1,N);    % measurement noise
n4      = sqrt(MyFront.R)*randn(1,N);    % measurement noise
n5      = sqrt(MyRear.R)*randn(1,N);    % measurement noise

slipangle.meas=VY+n1';                           % Measurement from Sensor
VLat.meas=Vx+n2';                           % Measurement from Sensor
yawRate.meas1=AY+n3';                           % Measurement from Sensor
MyFront.meas1=AY+n4';                           % Measurement from Sensor
MyRear.meas1=AY+n5';                           % Measurement from Sensor

VLat.tune1=[10 0 0];               %Tune Parameters
yawRate.tune1=[0.5 -1 5];               %Tune Parameters
MyFront.tune1=[0.5 -1 30];               %Tune Parameters
MyRear.tune1=[0.5 -1 30];               %Tune Parameters
slipangle.tune1=[0.5 10 20];               %Tune Parameters[0.5 -1 20];

slipangle.constS=[g m];             %Constants for State Equation
slipangle.constM=[];                      %Constants for Measurement Equation
VLat.constS=g;             %Constants for State Equation
VLat.constM=[];                      %Constants for Measurement Equation
yawRate.constS=[Iz lf lr];             %Constants for State Equation
yawRate.constM1=m;                      %Constants for Measurement Equation
MyFront.constS=[Iz m lr l];             %Constants for State Equation
MyFront.constM1=m;                      %Constants for Measurement Equation
MyRear.constS=[Iz m lr l];             %Constants for State Equation
MyRear.constM1=m;                      %Constants for Measurement Equation


%%----------------------Initializing System Inputs-----------------------%%
inp1=Ay;
inp2=phi;
inp3=Vx;
inp8=delta;
inp9=YawAcc;
inp4=YawR;

inp5=zeros(1,N);
inp6=zeros(1,N);
inp7=zeros(1,N);
inp10=zeros(1,N);
inp11=zeros(1,N);
inp12=zeros(1,N);


inp5(1)=Beta(1);
inp6(1)=Fyf(1);
inp7(1)=Fyr(1);
inp10(1)=Myf(1);
inp11(1)=Myr(1);
inp12(1)=BetaR(1);


%%-------------------------------------------------------------------
%%------------------- ESTIMATE STATE USING UKF ----------------------
fprintf('\nEstimating trajectory...\n');

for k=2:(N)
 
    VLat.inpS=[inp1(k-1) inp2(k-1) inp3(k-1) inp4(k-1)]; %input for State Equation
    VLat.inpM=[inp5(k-1)];
    yawRate.inpS=[inp6(k-1) inp7(k-1) inp8(k-1)]; %input for State Equation
    yawRate.inpM1=[inp6(k-1) inp7(k-1) inp8(k-1) inp3(k-1)];
    slipangle.inpS=[inp8(k-1) inp2(k-1) inp4(k-1) inp3(k-1) inp6(k-1) inp7(k-1)]; %input for State Equation
    slipangle.inpM=[inp3(k-1)];
    MyFront.inpS=[inp8(k-1) inp3(k-1) inp12(k-1) inp4(k-1) inp9(k-1)]; %input for State Equation
    MyFront.inpM1=[inp10(k-1) inp7(k-1) inp8(k-1) inp3(k-1) inp4(k-1)];
    MyRear.inpS=[inp3(k-1) inp12(k-1) inp4(k-1) inp9(k-1)]; %input for State Equation
    MyRear.inpM1=[inp11(k-1) inp6(k-1) inp8(k-1) inp3(k-1) inp4(k-1)];
  
   % Generate UKF estimate
  [MyRear.ukf(k),MyRear.Pxx] = ukfmod(MyRear.tune1,'MyrState',MyRear.constS,MyRear.inpS,MyRear.ukf(k-1),MyRear.Pxx,'MyrMeas1',MyRear.meas1(k),MyRear.inpM1,MyRear.constM1,MyRear.R,MyRear.Q,Ts);

  [MyFront.ukf(k),MyFront.Pxx] = ukfmod(MyFront.tune1,'MyfState',MyFront.constS,MyFront.inpS,MyFront.ukf(k-1),MyFront.Pxx,'MyfMeas1',MyFront.meas1(k),MyFront.inpM1,MyFront.constM1,MyFront.R,MyFront.Q,Ts);

  [slipangle.ukf(k),slipangle.Pxx] = ukfmod(slipangle.tune1,'BetaState',slipangle.constS,slipangle.inpS,slipangle.ukf(k-1),slipangle.Pxx,'BetaMeas',slipangle.meas(k),slipangle.inpM,slipangle.constM,slipangle.R,slipangle.Q,Ts);

  [yawRate.ukf(k),yawRate.Pxx] = ukfmod(yawRate.tune1,'rYawState',yawRate.constS,yawRate.inpS,yawRate.ukf(k-1),yawRate.Pxx,'rYawMeas1',yawRate.meas1(k),yawRate.inpM1,yawRate.constM1,yawRate.R,yawRate.Q,Ts);

 %[VLat.ukf(k),VLat.Pxx] = ukfmod(VLat.tune1,'VyState',VLat.constS,VLat.inpS,VLat.ukf(k-1),VLat.Pxx,'VyMeas',VLat.meas(k),VLat.inpM,VLat.constM,VLat.R,VLat.Q,Ts);
                    %ukfmod(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts) 

    
inp10(k)=MyFront.ukf(k);   
inp11(k)=MyRear.ukf(k);     
inp5(k)=slipangle.ukf(k);
inp12(k)=(inp5(k)-inp5(k-1))/Ts;
%inp4(k)=yawRate.ukf(k);
inp6(k)=(MyFront.ukf(k)-MyFront.ukf(k-1))/Ts;
inp7(k)=(MyRear.ukf(k)-MyRear.ukf(k-1))/Ts;
iteration=k;

end



