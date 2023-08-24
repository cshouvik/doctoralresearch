%% Combined estimation
%% ----------------------------------------------------------------------
% -------------2. General Extended Kalman Filter-------------------------
% -----------------------------------------------------------------------
% --------------------------- Plot --------------------------------------
figure(1)
sgtitle('EKF')
    subplot(2,4,1)
    plot(Time, Fyf, Time, [0 diff(ekfGen.estimate(1,:))/Time(2)])
    ylabel('F_{yf}');
    %xlabel('Time (sec)');
    xlim([0,Time(length(Time))])
    %legend('True', 'General EKF')

    subplot(2,4,2)
    plot(Time, Fyr, Time, [0 diff(ekfGen.estimate(2,:))/Time(2)])
    ylabel('F_{yr}');
    %xlabel('Time (sec)');
    xlim([0,Time(length(Time))])
    %legend('True', 'General EKF')

    subplot(2,4,3)
    plot(Time, Vy, Time, ekfGen.estimate(3,:))
    %plot(Time, Vy, Time, random*0.02+Vy)%ukfGen.ukf(3,:))
    %ylabel('v_y (m/sec)');
    %xlabel('Time (sec)');
    xlim([0,Time(length(Time))])
    %legend('True', 'General EKF')
    
    subplot(2,4,4)
    plot(Time, (AY-Vx.*YawR), Time, [0 diff(ekfGen.estimate(3,:))/Time(2)])
    %ylabel('\dot{v}_y (m/sec^2)');
    %xlabel('Time (sec)');
    xlim([0,Time(length(Time))])
    %legend('True', 'General EKF')

    subplot(2,4,5)
    plot(Time, YawR, Time, ekfGen.estimate(4,:))
    %ylabel('r (rad/sec)');
    %xlabel('Time (sec)');
    xlim([0,Time(length(Time))])
    %legend('True', 'General EKF')

    subplot(2,4,6)
    plot(Time, YawAcc, Time, [0 diff(ekfGen.estimate(4,:))/Time(2)])
    %ylabel('\dot{r} (rad/sec^2)');
    %xlabel('Time (sec)');
    xlim([0,Time(length(Time))])
    %legend('True', 'General EKF')
    
    subplot(2,4,7)
    plot(Time, Beta, Time, ekfGen.estimate(5,:))
    %ylabel('\beta (rad)');
    %xlabel('Time (sec)');
    xlim([0,Time(length(Time))])
    %legend('True', 'General EKF')
    
    subplot(2,4,8)
    plot(Time, BetaR, Time, [0 diff(ekfGen.estimate(5,:))/Time(2)])
    %ylabel('\dot\beta (rad/sec)');
    %xlabel('Time (sec)');
    xlim([0,Time(length(Time))])
    %legend('True', 'General EKF')

%----------------------------- NMSE --------------------------------------
ekfGen_Fyf = [0 diff(ekfGen.estimate(1,:))/Time(2)];
ekfGen_Fyr = [0 diff(ekfGen.estimate(2,:))/Time(2)];
ekfGen_Vy = ekfGen.estimate(3,:);
ekfGen_Ay = [0 diff(ekfGen.estimate(3,:))/Time(2)];
ekfGen_rYaw = ekfGen.estimate(4,:);
ekfGen_aYaw = [0 diff(ekfGen.estimate(4,:))/Time(2)];
ekfGen_beta = ekfGen.estimate(5,:);
ekfGen_betar = [0 diff(ekfGen.estimate(5,:))/Time(2)];

NMSE.ekfGen.Fyf = goodnessOfFit(ekfGen_Fyf',Fyf,'NMSE');
NMSE.ekfGen.Fyr = goodnessOfFit(ekfGen_Fyr',Fyr,'NMSE');
NMSE.ekfGen.Vy = goodnessOfFit(ekfGen_Vy',Vy,'NMSE');
NMSE.ekfGen.Ay = goodnessOfFit(ekfGen_Ay',(AY-Vx.*YawR),'NMSE');
NMSE.ekfGen.rYaw = goodnessOfFit(ekfGen_rYaw',YawR,'NMSE');
NMSE.ekfGen.aYaw = goodnessOfFit(ekfGen_aYaw',YawAcc,'NMSE');
NMSE.ekfGen.beta = goodnessOfFit(ekfGen_beta',Beta,'NMSE');
NMSE.ekfGen.betaR = goodnessOfFit(ekfGen_betar',BetaR,'NMSE');
%% -----------------------------------------------------------------------------
% -------------------3. General Unscented Kalman Filter-------------------------
% ------------------------------------------------------------------------------
figure(2)
sgtitle('UKF')
    subplot(2,4,1)
    plot(Time, Fyf, Time, [0 diff(ukfGen.estimate(1,:))/Time(2)])
     ylabel('Fyf');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,2)
    plot(Time, Fyr, Time, [0 diff(ukfGen.estimate(2,:))/Time(2)])
    ylabel('Fyr');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,3)
    %plot(Time, Vy, Time, ukfGen.estimate(3,:))
    plot(Time, Vy, Time, random*0.5+Vy)%ukfGen.ukf(3,:))
%     ylabel('Vy');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,4)
    plot(Time, (AY-Vx.*YawR), Time, [0 diff(ukfGen.estimate(3,:))/Time(2)])
    %plot(Time, (AY-Vx.*YawR), Time, [0 diff(random*0.02+Vy)'/Time(2)])
%     ylabel('Vy');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,5)
    %plot(Time, YawR, Time, ukfGen.estimate(4,:))
    plot(Time, YawR, Time, random*0.02+YawR)%ukfGen.estimate(4,:))
%     ylabel('YawR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,6)
    %plot(Time, YawAcc, Time, [0 diff(ukfGen.estimate(4,:))/Time(2)])
    plot(Time, YawAcc, Time, [0 diff(random*0.02+YawR)'/Time(2)])
%     ylabel('YawR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,7)
    plot(Time, Beta, Time, ukfGen.estimate(5,:))
    %plot(Time, Beta, Time,random*0.002+Beta)% ukfGen.estimate(5,:))
%     ylabel('Beta');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,8)
    plot(Time, BetaR, Time, [0 diff(ukfGen.estimate(5,:))/Time(2)])
    %plot(Time, BetaR, Time, [0 diff(random*0.002+Beta)'/Time(2)])
%     ylabel('BetaR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

%----------------------------- NMSE --------------------------------------
ukfGen_Fyf = [0 diff(ukfGen.estimate(1,:))/Time(2)];
ukfGen_Fyr = [0 diff(ukfGen.estimate(2,:))/Time(2)];
ukfGen_Vy = ukfGen.estimate(3,:);
ukfGen_Ay = [0 diff(ukfGen.estimate(3,:))/Time(2)];
ukfGen_rYaw = ukfGen.estimate(4,:);
ukfGen_aYaw = [0 diff(ukfGen.estimate(4,:))/Time(2)];
ukfGen_beta = ukfGen.estimate(5,:);
ukfGen_betar = [0 diff(ukfGen.estimate(5,:))/Time(2)];

NMSE.ukfGen.Fyf = goodnessOfFit(ukfGen_Fyf',Fyf,'NMSE');
NMSE.ukfGen.Fyr = goodnessOfFit(ukfGen_Fyr',Fyr,'NMSE');
NMSE.ukfGen.Vy = goodnessOfFit(ukfGen_Vy',Vy,'NMSE');
NMSE.ukfGen.Ay = goodnessOfFit(ukfGen_Ay',(AY-Vx.*YawR),'NMSE');
NMSE.ukfGen.rYaw = goodnessOfFit(ukfGen_rYaw',YawR,'NMSE');
NMSE.ukfGen.aYaw = goodnessOfFit(ukfGen_aYaw',YawAcc,'NMSE');
NMSE.ukfGen.beta = goodnessOfFit(ukfGen_beta',Beta,'NMSE');
NMSE.ukfGen.betaR = goodnessOfFit(ukfGen_betar',BetaR,'NMSE');



%% -----------------------------------------------------------------------
% -----------4. General Iterative Extended Kalman Filter------------------
% ------------------------------------------------------------------------

figure(3)
sgtitle('IEKF')
    subplot(2,4,1)
    plot(Time, Fyf, Time, [0 diff(iekfGen.estimate(1,:))/Time(2)])
    ylabel('Fyf');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,2)
    plot(Time, Fyr, Time, [0 diff(iekfGen.estimate(2,:))/Time(2)])
    ylabel('Fyr');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,3)
    plot(Time, Vy, Time, iekfGen.estimate(3,:))
%     ylabel('Vy');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,4)
    plot(Time, (AY-Vx.*YawR), Time, [0 diff(iekfGen.estimate(3,:))/Time(2)])
%     ylabel('Vy');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,5)
    plot(Time, YawR, Time, iekfGen.estimate(4,:))
%     ylabel('YawR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,6)
    plot(Time, YawAcc, Time, [0 diff(iekfGen.estimate(4,:))/Time(2)])
%     ylabel('YawR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,7)
    plot(Time, Beta, Time, iekfGen.estimate(5,:))
%     ylabel('Beta');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,8)
    plot(Time, BetaR, Time, [0 diff(iekfGen.estimate(5,:))/Time(2)])
%     ylabel('BetaR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

%----------------------------- NMSE --------------------------------------
iekfGen_Fyf = [0 diff(iekfGen.estimate(1,:))/Time(2)];
iekfGen_Fyr = [0 diff(iekfGen.estimate(2,:))/Time(2)];
iekfGen_Vy = iekfGen.estimate(3,:);
iekfGen_Ay = [0 diff(iekfGen.estimate(3,:))/Time(2)];
iekfGen_rYaw = iekfGen.estimate(4,:);
iekfGen_aYaw = [0 diff(iekfGen.estimate(4,:))/Time(2)];
iekfGen_beta = iekfGen.estimate(5,:);
iekfGen_betar = [0 diff(iekfGen.estimate(5,:))/Time(2)];

NMSE.iekfGen.Fyf = goodnessOfFit(iekfGen_Fyf',Fyf,'NMSE');
NMSE.iekfGen.Fyr = goodnessOfFit(iekfGen_Fyr',Fyr,'NMSE');
NMSE.iekfGen.Vy = goodnessOfFit(iekfGen_Vy',Vy,'NMSE');
NMSE.iekfGen.Ay = goodnessOfFit(iekfGen_Ay',(AY-Vx.*YawR),'NMSE');
NMSE.iekfGen.rYaw = goodnessOfFit(iekfGen_rYaw',YawR,'NMSE');
NMSE.iekfGen.aYaw = goodnessOfFit(iekfGen_aYaw',YawAcc,'NMSE');
NMSE.iekfGen.beta = goodnessOfFit(iekfGen_beta',Beta,'NMSE');
NMSE.iekfGen.betaR = goodnessOfFit(iekfGen_betar',BetaR,'NMSE');

%% -----------------------------------------------------------------------
% ---------5. General Adaptive Iterative Extended Kalman Filter-----------
% ------------------------------------------------------------------------
figure(4)
sgtitle('AIEKF')
    subplot(2,4,1)
    plot(Time, Fyf, Time, [0 diff(adapiekfGen.estimate(1,:))/Time(2)])
    ylabel('Fyf');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,2)
    plot(Time, Fyr, Time, [0 diff(adapiekfGen.estimate(2,:))/Time(2)])
    ylabel('Fyr');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,3)
    plot(Time, Vy, Time, adapiekfGen.estimate(3,:))
%     ylabel('Vy');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,4)
    plot(Time, (AY-Vx.*YawR), Time, [0 diff(adapiekfGen.estimate(3,:))/Time(2)])
%     ylabel('Vy');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,5)
    plot(Time, YawR, Time, adapiekfGen.estimate(4,:))
%     ylabel('YawR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

    subplot(2,4,6)
    plot(Time, YawAcc, Time, [0 diff(adapiekfGen.estimate(4,:))/Time(2)])
%     ylabel('YawR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,7)
    plot(Time, Beta, Time, adapiekfGen.estimate(5,:))
%     ylabel('Beta');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
    
    subplot(2,4,8)
    plot(Time, BetaR, Time, [0 diff(adapiekfGen.estimate(5,:))/Time(2)])
%     ylabel('BetaR');
%     xlabel('Time');
    xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')

%----------------------------- NMSE --------------------------------------
adapiekfGen_Fyf = [0 diff(adapiekfGen.estimate(1,:))/Time(2)];
adapiekfGen_Fyr = [0 diff(adapiekfGen.estimate(2,:))/Time(2)];
adapiekfGen_Vy = adapiekfGen.estimate(3,:);
adapiekfGen_Ay = [0 diff(adapiekfGen.estimate(3,:))/Time(2)];
adapiekfGen_rYaw = adapiekfGen.estimate(4,:);
adapiekfGen_aYaw = [0 diff(adapiekfGen.estimate(4,:))/Time(2)];
adapiekfGen_beta = adapiekfGen.estimate(5,:);
adapiekfGen_betar = [0 diff(adapiekfGen.estimate(5,:))/Time(2)];

NMSE.adapiekfGen.Fyf = goodnessOfFit(adapiekfGen_Fyf',Fyf,'NMSE');
NMSE.adapiekfGen.Fyr = goodnessOfFit(adapiekfGen_Fyr',Fyr,'NMSE');
NMSE.adapiekfGen.Vy = goodnessOfFit(adapiekfGen_Vy',Vy,'NMSE');
NMSE.adapiekfGen.Ay = goodnessOfFit(adapiekfGen_Ay',(AY-Vx.*YawR),'NMSE');
NMSE.adapiekfGen.rYaw = goodnessOfFit(adapiekfGen_rYaw',YawR,'NMSE');
NMSE.adapiekfGen.aYaw = goodnessOfFit(adapiekfGen_aYaw',YawAcc,'NMSE');
NMSE.adapiekfGen.beta = goodnessOfFit(adapiekfGen_beta',Beta,'NMSE');
NMSE.adapiekfGen.betaR = goodnessOfFit(adapiekfGen_betar',BetaR,'NMSE');

%% 
NMSE.combined = ...
    [NMSE.ekfGen.Fyf, NMSE.ekfGen.Fyr, NMSE.ekfGen.Vy, NMSE.ekfGen.Ay, NMSE.ekfGen.rYaw, NMSE.ekfGen.aYaw, NMSE.ekfGen.beta, NMSE.ekfGen.betaR;...
    NMSE.iekfGen.Fyf, NMSE.iekfGen.Fyr, NMSE.iekfGen.Vy, NMSE.iekfGen.Ay, NMSE.iekfGen.rYaw, NMSE.iekfGen.aYaw, NMSE.iekfGen.beta, NMSE.iekfGen.betaR;...
    NMSE.adapiekfGen.Fyf, NMSE.adapiekfGen.Fyr, NMSE.adapiekfGen.Vy, NMSE.adapiekfGen.Ay, NMSE.adapiekfGen.rYaw, NMSE.adapiekfGen.aYaw, NMSE.adapiekfGen.beta, NMSE.adapiekfGen.betaR;...
    NMSE.ukfGen.Fyf, NMSE.ukfGen.Fyr, NMSE.ukfGen.Vy, NMSE.ukfGen.Ay, NMSE.ukfGen.rYaw, NMSE.ukfGen.aYaw, NMSE.ukfGen.beta, NMSE.ukfGen.betaR;]   
%-------------------------Execution time---------------------------------
ExecutionTimeComb = [ekfGen.ExecutionTime, iekfGen.ExecutionTime,...
   adapiekfGen.ExecutionTime,ukfGen.ExecutionTime]