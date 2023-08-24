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
% *************************** General Estimators *******************************
% ******************************************************************************
%		1. General Kalman Filter

%% -----------------------------------------------------------------------------
% --------------------2. General Extended Kalman Filter-------------------------
% ------------------------------------------------------------------------------
select = input('Run General Extended Kalman Filter(1 for Yes and 0 for NO): ');
if select == 1
    disp('Executing General Extended Kalman Filter...');
    ekfGen.Q = 0.1*eye(5,5);
    ekfGen.R = 100*eye(5,5);
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
    ekfGen.ExecutionTime = toc;
    
    disp('General Extended Kalman Filter Execution Complete...');
    disp('Plotting Outputs...');
end
%% -----------------------------------------------------------------------------
% -------------------3. General Unscented Kalman Filter-------------------------
% ------------------------------------------------------------------------------
select = input('Run General Unscented Kalman Filter(1 for Yes and 0 for NO): ');
if select == 1   
    disp('Executing General Unscented Kalman Filter...');
    ukfGen.N = length(Time);                        % Number of time steps
    ukfGen.Q = 100*eye(5);                        % Process noise variance   repmat([100;100;100000;100000;10000],1,5).
    ukfGen.R = 0.0979*eye(5);                       % Measurement noise variance repmat([1;1;1;1;1],1,5).
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
    ukfGen.ExecutionTime=toc;
    disp('General Unscented Kalman Filter Execution Complete...');
    disp('Plotting Outputs...');
%     figure(3)
%     subplot(2,3,1)
%     plot(Time, delta)
%     ylabel('Steering Angle');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
% 
%     subplot(2,3,2)
%     plot(Time, Myf, Time, ukfGen.estimate(1,:))
%     ylabel('Myf');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General UKF')
% 
%     subplot(2,3,3)
%     plot(Time, Myr, Time, ukfGen.estimate(2,:))
%     ylabel('Myr');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General UKF')
% 
%     subplot(2,3,4)
%     plot(Time, Vy, Time, ukfGen.estimate(3,:))
%     ylabel('Vy');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General UKF')
% 
%     subplot(2,3,5)
%     plot(Time, YawR, Time, ukfGen.estimate(4,:))
%     ylabel('YawR');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General UKF')
% 
%     subplot(2,3,6)
%     plot(Time, Beta, Time, ukfGen.estimate(5,:))
%     ylabel('Beta');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General UKF')
end
%% -----------------------------------------------------------------------------
% ---------------4. General Iterative Extended Kalman Filter--------------------
% ------------------------------------------------------------------------------
select = input('Run General Iterative Extended Kalman Filter(1 for Yes and 0 for NO): ');
if select == 1
    disp('Executing General Iterative Extended Kalman Filter...');
    iekfGen.Q = 0.1*eye(5,5);
    iekfGen.R = 100*eye(5,5);
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
    iekfGen.ExecutionTime=toc;

    disp('General Iterative Extended Kalman Filter Execution Complete...');
    disp('Plotting Outputs...');

%     figure(4)
%     %suptitle('General iekf Plot')
% 
%     subplot(2,3,1)
%     plot(Time, delta)
%     ylabel('Steering Angle');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
% 
%     subplot(2,3,2)
%     plot(Time, Myf, Time, iekfGen.estimate(1,:))
%     ylabel('Myf');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General iekf')
% 
%     subplot(2,3,3)
%     plot(Time, Myr, Time, iekfGen.estimate(2,:))
%     ylabel('Myr');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General iekf')
% 
%     subplot(2,3,4)
%     plot(Time, Vy, Time, iekfGen.estimate(3,:))
%     ylabel('Vy');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General iekf')
% 
%     subplot(2,3,5)
%     plot(Time, YawR, Time, iekfGen.estimate(4,:))
%     ylabel('YawR');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General iekf')
% 
%     subplot(2,3,6)
%     plot(Time, Beta, Time, iekfGen.estimate(5,:))
%     ylabel('Beta');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General iekf')
end
%% -----------------------------------------------------------------------------
% -----------5. General Adaptive Iterative Extended Kalman Filter---------------
% ------------------------------------------------------------------------------
select = input('Run General Adaptive Iterative Extended Kalman Filter(1 for Yes and 0 for NO): ');
if select == 1
    disp('Executing General Adaptive Iterative Extended Kalman Filter...');
    adapiekfGen.Q = 0.1*eye(5,5);
    adapiekfGen.R = 100*eye(5,5);
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
    adapiekfGen.ExecutionTime=toc;

    disp('General Iterative Extended Kalman Filter Execution Complete...');
    disp('Plotting Outputs...');

    figure(5)
    %suptitle('General adapiekf Plot')

    subplot(2,3,1)
    plot(Time, delta)
    ylabel('Steering Angle');
    xlabel('Time');
    xlim([0,Time(length(Time))])

    subplot(2,3,2)
    plot(Time, Myf, Time, adapiekfGen.estimate(1,:))
    ylabel('Myf');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'General adapiekf')

    subplot(2,3,3)
    plot(Time, Myr, Time, adapiekfGen.estimate(2,:))
    ylabel('Myr');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'General adapiekf')

    subplot(2,3,4)
    plot(Time, Vy, Time, adapiekfGen.estimate(3,:))
    ylabel('Vy');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'General adapiekf')

    subplot(2,3,5)
    plot(Time, YawR, Time, adapiekfGen.estimate(4,:))
    ylabel('YawR');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'General adapiekf')

    subplot(2,3,6)
    plot(Time, Beta, Time, adapiekfGen.estimate(5,:))
    ylabel('Beta');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'General adapiekf')
end
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
select = input('Run EKF Estimation Modules (1 for YES and 0 for NO) : ');
if select == 1
    disp('Executing Extended Kalman Filter Modules...');
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
        
    figure(7)
    suptitle('EKF Module Plot')

    subplot(2,3,1)
    plot(Time, delta)
    ylabel('Steering Angle');
    xlabel('Time');
    xlim([0,Time(length(Time))])

    subplot(2,3,2)
    plot(Time, Myf, Time, modEKF.Myf.ekf)
    ylabel('Myf');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'EKF Module')

    subplot(2,3,3)
    plot(Time, Myr, Time, modEKF.Myr.ekf)
    ylabel('Myr');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'EKF Module')

    subplot(2,3,4)
    plot(Time, Vy, Time, modEKF.Vy.ekf)
    ylabel('Vy');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'EKF Module')

    subplot(2,3,5)
    plot(Time, YawR, Time, modEKF.rYaw.ekf)
    ylabel('YawR');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'EKF Module')

    subplot(2,3,6)
    plot(Time, Beta, Time, modEKF.slip.ekf)
    ylabel('Beta');
    xlabel('Time');
    xlim([0,Time(length(Time))])
    legend('True', 'EKF Module')

end
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

