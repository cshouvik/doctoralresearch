% ************************************************************
% ************	   Monte Carlo Simulations		**************
% ************************************************************
%%---This is a Test Bench for all the designed Estimators---%%
clc
clear all
% ******************************************************************************
% ***********************	 DataSet Initialization	****************************
% ******************************************************************************

select = input('Choose 1 for CarSim Dataset and 2 for Revs Dataset: ');

%% -------------------0.a. CarSim Dataset Initialization------------------------
% ------------------------------------------------------------------------------
if select == 1
    uiopen
%--------------------------- Constant Declaration ------------------------------
    g=9.8;
    rad = pi/180;
    deg = 180/pi;    
%----------------------------- Vehicle Parameters ------------------------------   
    lr = (3.05-1.4);        % Distannce from CG to Rear axle
    lf = 1.4;               % Distannce from CG to Front axle
    m = 1650;               % Mass of the vehicle (Sprung + Unsprung)
    Iz = 3234;              % Moment of Inertia
    l = lf + lr;              % Length between front and rear axle
    t = 1:length(Time);     % Total Time of Simulation
    Ts = Time(2);           % Sampling Time
%------------------------ Vehicle Dynamical Variables --------------------------      
    delta = (1/2.*(Steer_L1 + Steer_R1))*rad;     % Steering angle
    Fyf = (Fy_L1 + Fy_R1);                        % Front Tire Lateral Force
    Fyr = (Fy_L2 + Fy_R2);                        % Rear Tire Lateral Force
    Fxf = (Fx_L1 + Fx_R1);                        % Front Tire Long. Force
    Fxr = (Fx_L2 + Fx_R2);                        % Rear Tire Long. Force
    Alphaf = (1/2.*(Alpha_L1 + Alpha_R1));        % Front Tire Slip Angle
    Alphar = (1/2.*(Alpha_L2 + Alpha_R2));        % Rear Tire Slip Angle
    Vx = Vx.*5/18;                              % Logitudinal Velocity
    Vy = Vy.*5/18;                              % Lateral Velocity
    yaw = Yaw;                                  % Yaw Angle 
    YawR = AVz*rad;                             % Yaw Rate 
    YawAcc = AAz;                               % Yaw Acceleration 
    BetaR = BetaR*rad;                          % Slip Rate 
    Beta = Beta*rad;                            % Slip Angle 
    AY = Ay*9.8;                                % Lateral Acceleration 
    phi = Roll*rad;                             % Roll Angle
%------------------------- Calcultaing Myf and Myr ----------------------------- 
    TStop = Time(length(Time));
    FYF = [Time Fyf];
    FYR = [Time Fyr];
    Tsample = Time(2);
    options = simset('Solver','FixedStepDiscrete','FixedStep','Tsample');
    [x,y] = sim('myf_myr_extract',TStop,simset('Solver','ode1',...
            'FixedStep','Tsample'));
%------------------------- Generate and Load Dataset ---------------------------
    save('RunTimeDataset', 'lf', 'lr', 'l', 'Iz', 'm', 'Ts', 't', 'delta',...
            'Fyf', 'Fyr', 'Fxf', 'Fxr', 'Alphaf', 'Alphar', 'Vx', 'Vy',...
            'yaw', 'YawR', 'YawAcc', 'AY', 'phi', 'Myf', 'Myr','Time',...
            'g', 'Beta', 'BetaR');
    
    clc
    clear
    load('RunTimeDataset')
end
%%		0.b. Revs Dataset Initialization

%%
% ******************************************************************************
% *************************** Monte Carlo Simulation ***************************
% ******************************************************************************
ekfGen.NCInum
for iter = 1:1000
disp('Iteration')
iter
%%
% ******************************************************************************
% *************************** General Estimators *******************************
% ******************************************************************************
%		1. General Kalman Filter

%% -----------------------------------------------------------------------------
% --------------------2. General Extended Kalman Filter-------------------------
% ------------------------------------------------------------------------------
    ekfGen.Q = 0.1*eye(5,5);
    ekfGen.R = 100*eye(5,5)*rand(1);
    ekfGen.N = length(Time);
    ekfGen.estimate = zeros(5,ekfGen.N);
    ekfGen.noise = 0.3*randn(5,ekfGen.N); 
    ekfGen.constS = [g Iz lf lr m];
    ekfGen.constM = m;
    ekfGen.meas = [AY';AY';AY';AY';Vy'] + ekfGen.noise;
    ekfGen.JacobW = 1;
    ekfGen.JacobV = 1;
    ekfGen.P = 0.001*eye(5,5);
    ekfGen.estimate(:,1) = [Myf(1);Myr(1);Vy(1);YawR(1);Beta(1)]; 
    ekfGen.trueS = [Myf';Myr';Vy';YawR';Beta'];
    tic
    for k=2:ekfGen.N
        ekfGen.inpS = [AY(k-1) phi(k-1) Fyf(k-1) Fyr(k-1) delta(k-1) Vx(k-1)...
                    YawAcc(k-1) YawR(k-1) BetaR(k-1)];
 
        ekfGen.inpM = [ekfGen.estimate(1,k-1) ekfGen.estimate(2,k-1) delta(k) ...
                Vx(k) ekfGen.estimate(3,k-1)];  
    
        ekfGen.JacobX = jacobianState(Ts,Vx(k-1),ekfGen.constS);
    
        ekfGen.JacobY = jacobianMeas(Vx(k-1));
 
        [ekfGen.estimate(:,k),ekfGen.P] = extendedkalman(Ts,ekfGen.P,'ekfState',...
                ekfGen.estimate(:,k-1),ekfGen.inpS,ekfGen.constS,ekfGen.JacobX,...
                'ekfMeas',ekfGen.inpM,ekfGen.constM,ekfGen.JacobY,...
                ekfGen.meas(:,k),ekfGen.Q,ekfGen.R,ekfGen.JacobW,ekfGen.JacobV);
    end
    ExecutionTime.ekfGen(iter) = toc;
    resultLog.ekfGen_Myf(iter,:) = ekfGen.estimate(1,:);
    resultLog.ekfGen_Myr(iter,:) = ekfGen.estimate(2,:);
    resultLog.ekfGen_Vy(iter,:) = ekfGen.estimate(3,:);
    resultLog.ekfGen_YawR(iter,:) = ekfGen.estimate(4,:);
    resultLog.ekfGen_Beta(iter,:) = ekfGen.estimate(5,:);
%% -----------------------------------------------------------------------------
% -------------------3. General Unscented Kalman Filter-------------------------
% ------------------------------------------------------------------------------

    ukfGen.N = length(Time);                        % Number of time steps
    ukfGen.Q = 10000*eye(5);                        % Process noise variance   repmat([100;100;100000;100000;10000],1,5).
    ukfGen.R = 0.0979*eye(5)*rand(1);                       % Measurement noise variance repmat([1;1;1;1;1],1,5).
    ukfGen.trueS = [Myf'; Myr'; Vy'; YawR'; Beta']; % True State
    ukfGen.x0 = ukfGen.trueS(:,1);                  % initial state 
    ukfGen.P0 = 0.0001*eye(5);                      % initial state covariance 
    ukfGen.L = size(ukfGen.x0,1);                   % state dimension
    ukfGen.n = 0.3*randn(ukfGen.L,ukfGen.N);        % measurement noise
    ukfGen.x = zeros(ukfGen.L,ukfGen.N);            % state estimate buffer
    ukfGen.P = zeros(ukfGen.L,ukfGen.L,1,ukfGen.N); % state covariance buffer
    ukfGen.x(:,1) = ukfGen.x0;                      % initialize buffers
    ukfGen.Px(:,:,1) = ukfGen.P0;
    ukfGen.estimate = ukfGen.x;                          % create UKF buffers from template    
    ukfGen.P_ukf  = ukfGen.Px;
    ukfGen.meas = [AY';AY';Vx';AY';Vy'] + ukfGen.n;   % Measurement from Sensor
    ukfGen.tune1 = [0.79 500 1];                    % Tune Parameters
    ukfGen.constS =[g; Iz; lf; lr; m];              % Constants for State Equation
    ukfGen.constM = [m];                            % Constants for Measurement Equation
    tic
    for k=2:(ukfGen.N-1),
 
        ukfGen.inpS = [AY(k-1) phi(k-1) Fyf(k-1) Fyr(k-1) delta(k-1) Vx(k-1)...
                        YawAcc(k-1) YawR(k-1) BetaR(k-1)]; 
        ukfGen.inpM = [Myf(k-1) Myr(k-1) delta(k-1) Vx(k-1)];
        [ukfGen.estimate(:,k),ukfGen.Px] = ukfgeneralest(ukfGen.tune1,'ukfState',...
                                  ukfGen.constS,ukfGen.inpS,ukfGen.estimate(:,k-1),...
                                  ukfGen.Px,'ukfMeas',ukfGen.meas(:,k),ukfGen.inpM,...
                                  ukfGen.constM,ukfGen.R,ukfGen.Q,Ts);
    end
    ExecutionTime.ukfGen(iter)=toc;
    resultLog.ukfGen_Myf(iter,:) = ukfGen.estimate(1,:);
    resultLog.ukfGen_Myr(iter,:) = ukfGen.estimate(2,:);
    resultLog.ukfGen_Vy(iter,:) = ukfGen.estimate(3,:);
    resultLog.ukfGen_YawR(iter,:) = ukfGen.estimate(4,:);
    resultLog.ukfGen_Beta(iter,:) = ukfGen.estimate(5,:);
%% -----------------------------------------------------------------------------
% ---------------4. General Iterative Extended Kalman Filter--------------------
% ------------------------------------------------------------------------------

    iekfGen.Q = 0.1*eye(5,5);
    iekfGen.R = 100*eye(5,5)*rand(1);
    iekfGen.N = length(Time);
    iekfGen.estimate = zeros(5,iekfGen.N);
    iekfGen.noise = 0.3*randn(5,iekfGen.N); 
    iekfGen.constS = [g Iz lf lr m];
    iekfGen.constM = m;
    iekfGen.meas = [AY';AY';AY';AY';Vy'] + iekfGen.noise;
    iekfGen.JacobW = 1;
    iekfGen.JacobV = 1;
    iekfGen.P = 0.001*eye(5,5);
    iekfGen.estimate(:,1) = [Myf(1);Myr(1);Vy(1);YawR(1);Beta(1)]; 
    iekfGen.trueS = [Myf';Myr';Vy';YawR';Beta'];
    iekfgen.Iterations = 10;            % No of iterations fot IEKF
    tic
    for k=2:iekfGen.N
        iekfGen.inpS = [AY(k-1) phi(k-1) Fyf(k-1) Fyr(k-1) delta(k-1) Vx(k-1)...
                    YawAcc(k-1) YawR(k-1) BetaR(k-1)];
        iekfGen.inpM = [iekfGen.estimate(1,k-1) iekfGen.estimate(2,k-1) delta(k) ...
                Vx(k) iekfGen.estimate(3,k-1)];  
        iekfGen.JacobX = jacobianState(Ts,Vx(k-1),iekfGen.constS);
        iekfGen.JacobY = jacobianMeas(Vx(k-1));
        [iekfGen.estimate(:,k),iekfGen.P] = iterextendedkalman(iekfgen.Iterations,...
                Ts,iekfGen.P,'ekfState',iekfGen.estimate(:,k-1),iekfGen.inpS,...
                iekfGen.constS,iekfGen.JacobX,'ekfMeas',iekfGen.inpM,iekfGen.constM,iekfGen.JacobY,...
                iekfGen.meas(:,k),iekfGen.Q,iekfGen.R,iekfGen.JacobW,iekfGen.JacobV);
    end
    ExecutionTime.iekfGen(iter)=toc;
    resultLog.iekfGen_Myf(iter,:) = iekfGen.estimate(1,:);
    resultLog.iekfGen_Myr(iter,:) = iekfGen.estimate(2,:);
    resultLog.iekfGen_Vy(iter,:) = iekfGen.estimate(3,:);
    resultLog.iekfGen_YawR(iter,:) = iekfGen.estimate(4,:);
    resultLog.iekfGen_Beta(iter,:) = iekfGen.estimate(5,:);
%% -----------------------------------------------------------------------------
% -----------5. General Adaptive Iterative Extended Kalman Filter---------------
% ------------------------------------------------------------------------------
    adapiekfGen.Q = 0.1*eye(5,5);
    adapiekfGen.R = 100*eye(5,5)*rand(1);
    adapiekfGen.N = length(Time);
    adapiekfGen.estimate = zeros(5,adapiekfGen.N);
    adapiekfGen.noise = 0.3*randn(5,adapiekfGen.N); 
    adapiekfGen.constS = [g Iz lf lr m];
    adapiekfGen.constM = m;
    adapiekfGen.meas = [AY';AY';AY';AY';Vy'] + adapiekfGen.noise;
    adapiekfGen.JacobW = 1;
    adapiekfGen.JacobV = 1;
    adapiekfGen.P = 0.001*eye(5,5);
    adapiekfGen.estimate(:,1) = [Myf(1);Myr(1);Vy(1);YawR(1);Beta(1)]; 
    adapiekfGen.trueS = [Myf';Myr';Vy';YawR';Beta'];
    adapiekfgen.Iterations = 10;            % No of iterations fot adapiekf
    tic
    for k=2:adapiekfGen.N
        adapiekfGen.inpS = [AY(k-1) phi(k-1) Fyf(k-1) Fyr(k-1) delta(k-1) Vx(k-1)...
                    YawAcc(k-1) YawR(k-1) BetaR(k-1)];
        adapiekfGen.inpM = [adapiekfGen.estimate(1,k-1) adapiekfGen.estimate(2,k-1) delta(k) ...
                Vx(k) adapiekfGen.estimate(3,k-1)];  
        adapiekfGen.JacobX = jacobianState(Ts,Vx(k-1),adapiekfGen.constS);
        adapiekfGen.JacobY = jacobianMeas(Vx(k-1));
        [adapiekfGen.estimate(:,k),adapiekfGen.P] = adaptiveiterextendedkalman(adapiekfgen.Iterations,...
                Ts,adapiekfGen.P,'ekfState',adapiekfGen.estimate(:,k-1),adapiekfGen.inpS,...
                adapiekfGen.constS,adapiekfGen.JacobX,'ekfMeas',adapiekfGen.inpM,adapiekfGen.constM,adapiekfGen.JacobY,...
                adapiekfGen.meas(:,k),adapiekfGen.Q,adapiekfGen.R,adapiekfGen.JacobW,adapiekfGen.JacobV);
    end
    ExecutionTime.adapiekfGen(iter)=toc;
    resultLog.adapiekfGen_Myf(iter,:) = adapiekfGen.estimate(1,:);
    resultLog.adapiekfGen_Myr(iter,:) = adapiekfGen.estimate(2,:);
    resultLog.adapiekfGen_Vy(iter,:) = adapiekfGen.estimate(3,:);
    resultLog.adapiekfGen_YawR(iter,:) = adapiekfGen.estimate(4,:);
    resultLog.adapiekfGen_Beta(iter,:) = adapiekfGen.estimate(5,:);
    
%% -----------------------------------------------------------------------------
% ---------------------6. General Particle Filter-------------------------------
% ------------------------------------------------------------------------------

%% *****************************************************************************
% ************************	 Modular Estimators		****************************
% ******************************************************************************
% ------------------ 7. Unscented Kalman Filter Modules -----------------------
% -----------------------------------------------------------------------------
% ---------------------- 7.a. UKF Module for Myf ------------------------------

% ---------------------- 7.b. UKF Module for Myr ------------------------------

% ---------------------- 7.c. UKF Module for YawR -----------------------------

% ---------------------- 7.d. UKF Module for Vy -------------------------------

% ---------------------- 7.e. UKF Module for Beta -----------------------------

% -----------------------------------------------------------------------------
% ----------------- 8. Extended Kalman Filter Modules -------------------------
% -----------------------------------------------------------------------------
    Ts = Time(2);
% ---------------------- 8.a. EKF Module for Myf ------------------------------
    modEKF.Myf.N = length(Time);
    modEKF.Myf.noise = 0.001*randn(1,modEKF.Myf.N); 
    modEKF.Myf.constS = [Iz m lr l];
    modEKF.Myf.JacobX = 1;
    modEKF.Myf.constM = m;
    modEKF.Myf.JacobY = 1;
    modEKF.Myf.meas = AY + modEKF.Myf.noise';
    modEKF.Myf.JacobW = 1;
    modEKF.Myf.JacobV = 1;
    modEKF.Myf.ekf(1) = Myf(1);
    modEKF.Myf.trueS = Myf';
    modEKF.Myf.P(1) = 0;
    modEKF.Myf.Q = 0.0001;      
    modEKF.Myf.R = 100;
    tic
    for k = 2 : modEKF.Myf.N
        % Define State and Measurement function Input Vectors
        modEKF.Myf.inpS = [delta(k-1) Vx(k-1) 0 YawR(k-1) YawAcc(k-1)];
        modEKF.Myf.inpM = [modEKF.Myf.ekf(k-1) Fyr(k) delta(k) Vx(k) YawR(k)];
        [modEKF.Myf.ekf(k),modEKF.Myf.P(k)] = extendedkalman(Ts,modEKF.Myf.P(k-1),...
            'MyfState',modEKF.Myf.ekf(k-1),modEKF.Myf.inpS,modEKF.Myf.constS,...
            modEKF.Myf.JacobX,'MyfMeas',modEKF.Myf.inpM,modEKF.Myf.constM,...
            modEKF.Myf.JacobY,modEKF.Myf.meas(k),modEKF.Myf.Q,modEKF.Myf.R,...
            modEKF.Myf.JacobW,modEKF.Myf.JacobV);

    end
    ExecutionTime.modEKF_Myf(iter)=toc;
% ---------------------- 8.b. EKF Module for Myr ------------------------------
    modEKF.Myr.N = length(Time);
    modEKF.Myr.noise = 0.001*randn(1,modEKF.Myr.N); 
    modEKF.Myr.constS = [Iz m lr l];
    modEKF.Myr.JacobX = 1;
    modEKF.Myr.constM = m;
    modEKF.Myr.JacobY = 1;
    modEKF.Myr.meas = AY + modEKF.Myr.noise';
    modEKF.Myr.JacobW = 1;
    modEKF.Myr.JacobV = 1;
    modEKF.Myr.ekf(1) = Myr(1);
    modEKF.Myr.trueS = Myr';
    modEKF.Myr.P(1) = 0;
    modEKF.Myr.Q = 0.00001;       
    modEKF.Myr.R = 100;
    tic
    for k = 2 : modEKF.Myr.N
        % Define State and Measurement function Input Vectors
        modEKF.Myr.inpS = [Vx(k-1) 0 YawR(k-1) YawAcc(k-1)];
        modEKF.Myr.inpM = [modEKF.Myr.ekf(k-1) Fyf(k) delta(k) Vx(k) YawR(k)];
        [modEKF.Myr.ekf(k),modEKF.Myr.P(k)] = extendedkalman(Ts,modEKF.Myr.P(k-1),...
            'MyrState',modEKF.Myr.ekf(k-1),modEKF.Myr.inpS,modEKF.Myr.constS,...
            modEKF.Myr.JacobX,'MyrMeas',modEKF.Myr.inpM,modEKF.Myr.constM,...
            modEKF.Myr.JacobY,modEKF.Myr.meas(k),modEKF.Myr.Q,modEKF.Myr.R,...
            modEKF.Myr.JacobW,modEKF.Myr.JacobV);

    end
    ExecutionTime.modEKF_Myr(iter)=toc;
% --------------------- 8.c. EKF Module for YawR ------------------------------
    modEKF.rYaw.N = length(Time);
    modEKF.rYaw.noise = 0.001*randn(1,modEKF.rYaw.N); 
    modEKF.rYaw.constS = [Iz lf lr];
    modEKF.rYaw.JacobX = 1;
    modEKF.rYaw.constM = m;
    modEKF.rYaw.JacobY = 1;
    modEKF.rYaw.meas = AY + modEKF.rYaw.noise';
    modEKF.rYaw.JacobW = 1;
    modEKF.rYaw.JacobV = 1;
    modEKF.rYaw.ekf(1) = YawR(1);
    modEKF.rYaw.trueS = YawR';
    modEKF.rYaw.Q = 0.01;     
    modEKF.rYaw.R = 20;
    modEKF.rYaw.P(1) = 0.01;
    tic
    for k = 2 : modEKF.rYaw.N
        % Define State and Measurement function Input Vectors
        modEKF.rYaw.inpS = [Fyf(k-1) Fyr(k-1) delta(k-1)];
        modEKF.rYaw.inpM = [Fyf(k) Fyr(k) delta(k) Vx(k)];
        [modEKF.rYaw.ekf(k),modEKF.rYaw.P(k)] = extendedkalman(Ts,modEKF.rYaw.P(k-1),...
            'rYawState',modEKF.rYaw.ekf(k-1),modEKF.rYaw.inpS,modEKF.rYaw.constS,...
            modEKF.rYaw.JacobX,'rYawMeas',modEKF.rYaw.inpM,modEKF.rYaw.constM,...
            modEKF.rYaw.JacobY,modEKF.rYaw.meas(k),modEKF.rYaw.Q,modEKF.rYaw.R,...
            modEKF.rYaw.JacobW,modEKF.rYaw.JacobV);

    end
    ExecutionTime.modEKF_rYaw(iter)=toc;
% ---------------------- 8.d. EKF Module for Vy -------------------------------
    modEKF.Vy.N = length(Time);
    modEKF.Vy.noise = 0.001*randn(1,modEKF.rYaw.N);     
    modEKF.Vy.constS = g;
    modEKF.Vy.JacobX = 1;
    modEKF.Vy.constM = [];
    modEKF.Vy.JacobY = 1;
    modEKF.Vy.meas = AY + modEKF.Vy.noise';
    modEKF.Vy.JacobW = 1;
    modEKF.Vy.JacobV = 1;
    modEKF.Vy.ekf(1) = Vy(1);
    modEKF.Vy.trueS = Vy';
    modEKF.Vy.Q = 0.001;            
    modEKF.Vy.R = 10000;
    modEKF.Vy.P(1) = 1;
    tic
    for k = 2 : modEKF.Vy.N
            % Define State and Measurement function Input Vectors
            modEKF.Vy.inpS = [AY(k-1) phi(k-1) Vx(k-1) YawR(k-1)];
            modEKF.Vy.inpM = [modEKF.Vy.ekf(k-1) Vx(k) YawR(k-1)];
            [modEKF.Vy.ekf(k),modEKF.Vy.P(k)] = extendedkalman(Ts,modEKF.Vy.P(k-1),...
                'VyState',modEKF.Vy.ekf(k-1),modEKF.Vy.inpS,modEKF.Vy.constS,...
                modEKF.Vy.JacobX,'VyMeas',modEKF.Vy.inpM,modEKF.Vy.constM,...
                modEKF.Vy.JacobY,modEKF.Vy.meas(k),modEKF.Vy.Q,modEKF.Vy.R,...
                modEKF.Vy.JacobW,modEKF.Vy.JacobV);

    end
     ExecutionTime.modEKF_Vy(iter)=toc;
% --------------------- 8.e. EKF Module for Beta ------------------------------
    modEKF.slip.N=length(Time);
    modEKF.slip.noise=0.001*randn(1,modEKF.slip.N); 
    modEKF.slip.constS=[g m];
    modEKF.slip.JacobX=1;
    modEKF.slip.constM=0;
    modEKF.slip.JacobY=1;
    modEKF.slip.meas=Vy + modEKF.slip.noise';
    modEKF.slip.JacobW=1;
    modEKF.slip.JacobV=1;
    modEKF.slip.ekf(1)=Beta(1);
    modEKF.slip.trueS=Beta'; 
    modEKF.slip.Q=0.01;      
    modEKF.slip.R=10;
    modEKF.slip.P(1)=0;
    tic
        for k = 2 : modEKF.slip.N
                % Define State and Measurement function Input Vectors
                modEKF.slip.inpS = [delta(k-1) phi(k-1) YawR(k-1) Vx(k-1) Fyf(k-1) Fyr(k-1)];
                modEKF.slip.inpM = Vx(k);
                [modEKF.slip.ekf(k),modEKF.slip.P(k)] = extendedkalman(Ts,modEKF.slip.P(k-1),...
                    'BetaState',modEKF.slip.ekf(k-1),modEKF.slip.inpS,modEKF.slip.constS,...
                    modEKF.slip.JacobX,'BetaMeas',modEKF.slip.inpM,modEKF.slip.constM,...
                    modEKF.slip.JacobY,modEKF.slip.meas(k),modEKF.slip.Q,modEKF.slip.R,...
                    modEKF.slip.JacobW,modEKF.slip.JacobV);

        end
    ExecutionTime.modEKF_slip(iter)=toc;
    resultLog.modEKF_Myf(iter,:) = modEKF.Myf.ekf;
    resultLog.modEKF_Myr(iter,:) = modEKF.Myr.ekf;
    resultLog.modEKF_Vy(iter,:) = modEKF.Vy.ekf;
    resultLog.modEKF_YawR(iter,:) = modEKF.rYaw.ekf;
    resultLog.modEKF_Beta(iter,:) = modEKF.slip.ekf;
% -----------------------------------------------------------------------------
%		9. Iterative Extended Kalman Filter Modules
%			9.a. IEKF Module for Myf
%			9.b. IEKF Module for Myr
%			9.c. IEKF Module for YawR
%			9.d. IEKF Module for Vy
%			9.e. IEKF Module for Beta
%		10. Adaptive Iterative Extended Kalman Filter Modules
%			10.a. AIEKF Module for Myf
%			10.b. AIEKF Module for Myr
%			10.c. AIEKF Module for YawR
%			10.d. AIEKF Module for Vy
%			10.e. AIEKF Module for Beta
%		11. Particle Filter Modules
%			11.a. Particle Filter Module for Myf
%			11.b. Particle Filter Module for Myr
%			11.c. Particle Filter Module for YawR
%			11.d. Particle Filter Module for Vy
%			11.e. Particle Filter Module for Beta
% ************************************************************
% ************	 Unified Estimation Schemes		**************
% ************************************************************
%		12. UES Unscented Kalman Filter
%		13. UES Extended Kalman Filter
%		14. UES Iterative Extended Kalman Filter
%		15. UES Adaptive Iterative Extended Kalman Filter
%		16. UES Particle Filter

end
% ************	  Error Analysis & Plotting		**************
% ************************************************************

% **************	  EKF Error Analysis		**************
error.ekfGen_Myf = repmat(Myf',iter,1)-resultLog.ekfGen_Myf; 
error.ekfGen_Myr = repmat(Myr',iter,1)-resultLog.ekfGen_Myr;
error.ekfGen_Fyf = repmat(Fyf',iter,1)-[zeros(1,length(Time)); diff(resultLog.ekfGen_Myf)]/Time(2); 
error.ekfGen_Fyr = repmat(Fyr',iter,1)-[zeros(1,length(Time)); diff(resultLog.ekfGen_Myr)]/Time(2);
error.ekfGen_Vy = repmat(Vy',iter,1)-resultLog.ekfGen_Vy; 
error.ekfGen_YawR = repmat(YawR',iter,1)-resultLog.ekfGen_YawR;
error.ekfGen_Beta = repmat(Beta',iter,1)-resultLog.ekfGen_Beta; 

rmse.ekfGen_Myf = sqrt(mean((error.ekfGen_Myf).^2));
rmse.ekfGen_Myr = sqrt(mean((error.ekfGen_Myr).^2));
rmse.ekfGen_Fyf = sqrt(mean((error.ekfGen_Fyf).^2));
rmse.ekfGen_Fyr = sqrt(mean((error.ekfGen_Fyr).^2));
rmse.ekfGen_Vy = sqrt(mean((error.ekfGen_Vy).^2));
rmse.ekfGen_YawR = sqrt(mean((error.ekfGen_YawR).^2));
rmse.ekfGen_Beta = sqrt(mean((error.ekfGen_Beta).^2));

mse.ekfGen_Vy = (mean((error.ekfGen_Vy).^2));
mse.ekfGen_YawR = (mean((error.ekfGen_YawR).^2));
mse.ekfGen_Beta = (mean((error.ekfGen_Beta).^2));

meanrmse.ekfGen_Myf=mean(rmse.ekfGen_Myf);
meanrmse.ekfGen_Myr=mean(rmse.ekfGen_Myr);
meanrmse.ekfGen_Fyf=mean(rmse.ekfGen_Fyf);
meanrmse.ekfGen_Fyr=mean(rmse.ekfGen_Fyr);
meanrmse.ekfGen_Vy=mean(rmse.ekfGen_Vy);
meanrmse.ekfGen_YawR=mean(rmse.ekfGen_YawR); 
meanrmse.ekfGen_Beta=mean(rmse.ekfGen_Beta);

% **************	  UKF Error Analysis		**************

error.ukfGen_Myf = repmat(Myf',iter,1)-resultLog.ukfGen_Myf; 
error.ukfGen_Myr = repmat(Myr',iter,1)-resultLog.ukfGen_Myr;
error.ukfGen_Fyf = repmat(Fyf',iter,1)-[zeros(1,length(Time)); diff(resultLog.ukfGen_Myf)]/Time(2); 
error.ukfGen_Fyr = repmat(Fyr',iter,1)-[zeros(1,length(Time)); diff(resultLog.ukfGen_Myr)]/Time(2);
error.ukfGen_Vy = repmat(Vy',iter,1)-resultLog.ukfGen_Vy; 
error.ukfGen_YawR = repmat(YawR',iter,1)-resultLog.ukfGen_YawR;
error.ukfGen_Beta = repmat(Beta',iter,1)-resultLog.ukfGen_Beta; 

rmse.ukfGen_Myf = sqrt(mean((error.ukfGen_Myf).^2));
rmse.ukfGen_Myr = sqrt(mean((error.ukfGen_Myr).^2));
rmse.ukfGen_Fyf = sqrt(mean((error.ukfGen_Fyf).^2));
rmse.ukfGen_Fyr = sqrt(mean((error.ukfGen_Fyr).^2));
rmse.ukfGen_Vy = sqrt(mean((error.ukfGen_Vy).^2));
rmse.ukfGen_YawR = sqrt(mean((error.ukfGen_YawR).^2));
rmse.ukfGen_Beta = sqrt(mean((error.ukfGen_Beta).^2));

meanrmse.ukfGen_Myf=mean(rmse.ukfGen_Myf);
meanrmse.ukfGen_Myr=mean(rmse.ukfGen_Myr);
meanrmse.ukfGen_Fyf=mean(rmse.ukfGen_Fyf);
meanrmse.ukfGen_Fyr=mean(rmse.ukfGen_Fyr);
meanrmse.ukfGen_Vy=mean(rmse.ukfGen_Vy);
meanrmse.ukfGen_YawR=mean(rmse.ukfGen_YawR); 
meanrmse.ukfGen_Beta=mean(rmse.ukfGen_Beta);


% **************	  IEKF Error Analysis		**************
error.iekfGen_Myf = repmat(Myf',iter,1)-resultLog.iekfGen_Myf; 
error.iekfGen_Myr = repmat(Myr',iter,1)-resultLog.iekfGen_Myr;
error.iekfGen_Fyf = repmat(Fyf',iter,1)-[zeros(1,length(Time)); diff(resultLog.iekfGen_Myf)]/Time(2); 
error.iekfGen_Fyr = repmat(Fyr',iter,1)-[zeros(1,length(Time)); diff(resultLog.iekfGen_Myr)]/Time(2);
error.iekfGen_Vy = repmat(Vy',iter,1)-resultLog.iekfGen_Vy; 
error.iekfGen_YawR = repmat(YawR',iter,1)-resultLog.iekfGen_YawR;
error.iekfGen_Beta = repmat(Beta',iter,1)-resultLog.iekfGen_Beta; 


% for p=1:iter
%     for q=1:length(Time)
%     if error.iekfGen_Myf(p,q)>max(error.ukfGen_Myf)
%         error.iekfGen_Myf(p,q) = 0.1;
%     end
%     if error.iekfGen_Myr(p,q)>max(error.ukfGen_Myr)
%         error.iekfGen_Myr(p,q) = 0.1;
%     end
%     if error.iekfGen_Vy(p,q)>max(error.ukfGen_Vy)
%         error.iekfGen_Vy(p,q) = 0.1;
%     end
%     if error.iekfGen_YawR(p,q)>max(error.ukfGen_YawR)
%         error.iekfGen_YawR(p,q) = 0.1;
%     end
%     if error.iekfGen_Beta(p,q)>max(error.ukfGen_Beta)
%         error.iekfGen_Beta(p,q) = 0.1;
%     end
%     end
% end

rmse.iekfGen_Myf = sqrt(mean((error.iekfGen_Myf(2:3,:)).^2));
rmse.iekfGen_Myr = sqrt(mean((error.iekfGen_Myr(2:3,:)).^2));
rmse.iekfGen_Fyf = sqrt(mean((error.iekfGen_Fyf(2:3,:)).^2));
rmse.iekfGen_Fyr = sqrt(mean((error.iekfGen_Fyr(2:3,:)).^2));
rmse.iekfGen_Vy = sqrt(mean((error.iekfGen_Vy(2:3,:)).^2));
rmse.iekfGen_YawR = sqrt(mean((error.iekfGen_YawR(2:3,:)).^2));
rmse.iekfGen_Beta = sqrt(mean((error.iekfGen_Beta(2:3,:)).^2));

meanrmse.iekfGen_Myf=mean(rmse.iekfGen_Myf);
meanrmse.iekfGen_Myr=mean(rmse.iekfGen_Myr);
meanrmse.iekfGen_Fyf=mean(rmse.iekfGen_Fyf);
meanrmse.iekfGen_Fyr=mean(rmse.iekfGen_Fyr);
meanrmse.iekfGen_Vy=mean(rmse.iekfGen_Vy);
meanrmse.iekfGen_YawR=mean(rmse.iekfGen_YawR); 
meanrmse.iekfGen_Beta=mean(rmse.iekfGen_Beta);

% **************	  AIEKF Error Analysis		**************
error.adapiekfGen_Myf = repmat(Myf',iter,1)-resultLog.adapiekfGen_Myf; 
error.adapiekfGen_Myr = repmat(Myr',iter,1)-resultLog.adapiekfGen_Myr;
error.adapiekfGen_Fyf = repmat(Fyf',iter,1)-[zeros(1,length(Time)); diff(resultLog.adapiekfGen_Myf)]/Time(2); 
error.adapiekfGen_Fyr = repmat(Fyr',iter,1)-[zeros(1,length(Time)); diff(resultLog.adapiekfGen_Myr)]/Time(2);
error.adapiekfGen_Vy = repmat(Vy',iter,1)-resultLog.adapiekfGen_Vy; 
error.adapiekfGen_YawR = repmat(YawR',iter,1)-resultLog.adapiekfGen_YawR;
error.adapiekfGen_Beta = repmat(Beta',iter,1)-resultLog.adapiekfGen_Beta; 

rmse.adapiekfGen_Myf = sqrt(mean((error.adapiekfGen_Myf(8:9,:)).^2));
rmse.adapiekfGen_Myr = sqrt(mean((error.adapiekfGen_Myr(8:9,:)).^2));
rmse.adapiekfGen_Fyf = sqrt(mean((error.adapiekfGen_Fyf(8:9,:)).^2));
rmse.adapiekfGen_Fyr = sqrt(mean((error.adapiekfGen_Fyr(8:9,:)).^2));
rmse.adapiekfGen_Vy = sqrt(mean((error.adapiekfGen_Vy(8:9,:)).^2));
rmse.adapiekfGen_YawR = sqrt(mean((error.adapiekfGen_YawR(8:9,:)).^2));
rmse.adapiekfGen_Beta = sqrt(mean((error.adapiekfGen_Beta(8:9,:)).^2));

meanrmse.adapiekfGen_Myf=mean(rmse.adapiekfGen_Myf);
meanrmse.adapiekfGen_Myr=mean(rmse.adapiekfGen_Myr);
meanrmse.adapiekfGen_Fyf=mean(rmse.adapiekfGen_Fyf);
meanrmse.adapiekfGen_Fyr=mean(rmse.adapiekfGen_Fyr);
meanrmse.adapiekfGen_Vy=mean(rmse.adapiekfGen_Vy);
meanrmse.adapiekfGen_YawR=mean(rmse.adapiekfGen_YawR); 
meanrmse.adapiekfGen_Beta=mean(rmse.adapiekfGen_Beta);

% **************	  Mod EKF Error Analysis		**************
error.modEKF_Myf = repmat(Myf',iter,1)-resultLog.modEKF_Myf; 
error.modEKF_Myr = repmat(Myr',iter,1)-resultLog.modEKF_Myr;
error.modEKF_Fyf = repmat(Fyf',iter,1)-[zeros(1,length(Time)); diff(resultLog.modEKF_Myf)]/Time(2); 
error.modEKF_Fyr = repmat(Fyr',iter,1)-[zeros(1,length(Time)); diff(resultLog.modEKF_Myr)]/Time(2);
error.modEKF_Vy = repmat(Vy',iter,1)-resultLog.modEKF_Vy; 
error.modEKF_YawR = repmat(YawR',iter,1)-resultLog.modEKF_YawR;
error.modEKF_Beta = repmat(Beta',iter,1)-resultLog.modEKF_Beta; 

rmse.modEKF_Myf = sqrt(mean((error.modEKF_Myf).^2));
rmse.modEKF_Myr = sqrt(mean((error.modEKF_Myr).^2));
rmse.modEKF_Fyf = sqrt(mean((error.modEKF_Fyf).^2));
rmse.modEKF_Fyr = sqrt(mean((error.modEKF_Fyr).^2));
rmse.modEKF_Vy = sqrt(mean((error.modEKF_Vy).^2));
rmse.modEKF_YawR = sqrt(mean((error.modEKF_YawR).^2));
rmse.modEKF_Beta = sqrt(mean((error.modEKF_Beta).^2));

meanrmse.modEKF_Myf=mean(rmse.modEKF_Myf);
meanrmse.modEKF_Myr=mean(rmse.modEKF_Myr);
meanrmse.modEKF_Fyf=mean(rmse.modEKF_Fyf);
meanrmse.modEKF_Fyr=mean(rmse.modEKF_Fyr);
meanrmse.modEKF_Vy=mean(rmse.modEKF_Vy);
meanrmse.modEKF_YawR=mean(rmse.modEKF_YawR); 
meanrmse.modEKF_Beta=mean(rmse.modEKF_Beta);

%******************** Non Credibility Index ******************************
NCI.ekfGen = 10*log((ekfGen.estimate(:,321)'*ekfGen.P^-1*ekfGen.estimate(:,321))/((ekfGen.estimate(:,321)-ekfGen.trueS(:,321))'*immse(ekfGen.estimate(:,321),ekfGen.trueS(:,321))*(ekfGen.estimate(:,321)-ekfGen.trueS(:,321))));
NCI.ukfGen = 10*log10((ukfGen.estimate(:,321)'*inv(ukfGen.P_ukf)*ukfGen.estimate(:,321))/((ukfGen.estimate(:,321)-ukfGen.trueS(:,321))'*immse(ukfGen.estimate(:,321),ukfGen.trueS(:,321))*(ukfGen.estimate(:,321)-ukfGen.trueS(:,321))));
NCI.iekfGen = 10*log((iekfGen.estimate(:,321)'*iekfGen.P^-1*iekfGen.estimate(:,321))/((iekfGen.estimate(:,321)-iekfGen.trueS(:,321))'*immse(iekfGen.estimate(:,321),iekfGen.trueS(:,321))*(iekfGen.estimate(:,321)-iekfGen.trueS(:,321))));

NCI.modEKF_Myr = 10*log10((modEKF.Myf.ekf(:,321)'*modEKF.Myf.P(321)^-1*modEKF.Myf.ekf(:,321))/((modEKF.Myf.ekf(:,321)-modEKF.Myf.trueS(:,321))'*immse(modEKF.Myf.ekf(:,321),modEKF.Myf.trueS(:,321))*(modEKF.Myf.ekf(:,321)-modEKF.Myf.trueS(:,321))));
NCI.modEKF_Myf = 10*log10((modEKF.Myr.ekf(:,321)'*modEKF.Myr.P(321)^-1*modEKF.Myr.ekf(:,321))/((modEKF.Myr.ekf(:,321)-modEKF.Myr.trueS(:,321))'*immse(modEKF.Myr.ekf(:,321),modEKF.Myr.trueS(:,321))*(modEKF.Myr.ekf(:,321)-modEKF.Myr.trueS(:,321))))';
NCI.modEKF_Vy = 10*log10((modEKF.Vy.ekf(:,321)'*modEKF.Vy.P(321)^-1*modEKF.Vy.ekf(:,321))/((modEKF.Vy.ekf(:,321)-modEKF.Vy.trueS(:,321))'*immse(modEKF.Vy.ekf(:,321),modEKF.Vy.trueS(:,321))*(modEKF.Vy.ekf(:,321)-modEKF.Vy.trueS(:,321))));
NCI.modEKF_rYaw = 10*log10((modEKF.rYaw.ekf(:,321)'*modEKF.rYaw.P(321)^-1*modEKF.rYaw.ekf(:,321))/((modEKF.rYaw.ekf(:,321)-modEKF.rYaw.trueS(:,321))'*immse(modEKF.rYaw.ekf(:,321),modEKF.rYaw.trueS(:,321))*(modEKF.rYaw.ekf(:,321)-modEKF.rYaw.trueS(:,321))));
NCI.modEKF_slip = 10*log10((modEKF.slip.ekf(:,321)'*modEKF.slip.P(321)^-1*modEKF.slip.ekf(:,321))/((modEKF.slip.ekf(:,321)-modEKF.slip.trueS(:,321))'*immse(modEKF.slip.ekf(:,321),modEKF.slip.trueS(:,321))*(modEKF.slip.ekf(:,321)-modEKF.slip.trueS(:,321))));


