dt = Ts;
c1 = m*lr/l;
c2 = Iz/l;
c3 = m*lf/l;
state = [];

%% ----------------Out Data Processing UES-UKF -------------------------
ukfmod_Fyf = [diff(MyFront.ukf1)]/dt;
ukfmod_Fyr = [diff(MyRear.ukf1)]/dt;
ukfmod_rYaw = rYaw.ukf1(1:321);
ukfmod_aYaw = [diff(rYaw.ukf1(1:322))]/dt;
ukfmod_Vy = VLateral.ukf(1:321);
ukfmod_Ay = [diff(VLateral.ukf(1:322))]/dt + Vx'.*ukfMod_rYaw;
ukfmod_Beta = slipangle.ukf(1:321);
ukfmod_BetaR = [diff(slipangle.ukf(1:322))]/dt;

ukfmod_Fyf_R = [diff(MyFront.P)]/dt;
ukfmod_Fyr_R = [diff(MyRear.P)]/dt;
ukfmod_rYaw_R = rYaw.P(1:321);
ukfmod_Vy_R = VLateral.P(1:321);
ukfmod_Beta_R = slipangle.P(1:321);
%% ---------------- Initialize Kalman Filter ----------------------------
initialState = [Fyf(1); Fyr(1); Vy(1); Ay(1); YawR(1); YawAcc(1); Beta(1); BetaR(1)];
initialPx = eye(8)*0.1;
Est = [];
Px = initialPx;
xhat = initialState;
Q = 0.01*eye(8);
R = eye(8);
B = 0;
H = eye(8);
inp = 0;
tic

for i = 1:length(Time)-1
%% --------------- Trial -1 (Discarded) ------------------------
% myfS = [1 dt 0 0 0 0 0 0 0 0];                      % myf
% fyfS = [0 0 0 0 0 0 c1*Vx(i) c2 0 c1*Vx(i)];        % Fyf
% myrS = [0 0 1 dt 0 0 0 0 0 0];                      % myr
% fyrS = [0 0 0 0 0 0 c3*Vx(i) -c2 0 c3*Vx(i)];       % Fyr
% vyS = [0 0 0 0 1 dt 0 0 0 0];                       % Vy
% ayS = [0 (1/m) 0 (1/m) 0 0 0 0 0 0];                % Ay
% rS = [0 0 0 0 0 0 1 dt 0 0];                        % YawRate
% raS = [0 (lf/Iz) 0 (-lr/Iz) 0 0 0 0 0 0];           % YawAcc
% bS = [0 0 0 0 0 0 0 0 1 dt];                        % SlipAngle
% brS = [0 (1/m*Vx(i)) 0 (1/m*Vx(i)) 0 0 0 0 0 0];    % SlipRate
% A = [myfS; fyfS; myrS; fyrS; vyS; ayS; rS; raS; bS; brS];

%% --------------- System Matrix Formation -----------------------
fyfS = [0 0 0 0 c1*Vx(i) c2 0 c1*Vx(i)];        % Fyf
fyrS = [0 0 0 0 c3*Vx(i) -c2 0 c3*Vx(i)];       % Fyr
vyS = [0 0 1 dt 0 0 0 0];                       % Vy
ayS = [(1/m) (1/m) 0 0 0 0 0 0];                % Ay
rS = [0 0 0 0 1 dt 0 0];                        % YawRate
raS = [(lf/Iz) (-lr/Iz) 0 0 0 0 0 0];           % YawAcc
bS = [0 0 0 0 0 0 1 dt];                        % SlipAngle
brS = [1/(m*Vx(i)) 1/(m*Vx(i)) 0 0 -1 0 0 0];    % SlipRate
 
A = [fyfS; fyrS; vyS; ayS; rS; raS; bS; brS];       % Approach 1
%% -------------------------Test with CarSim Data-------------------------
%state = [Fyf(i); Fyr(i); Vy(i); Ay(i); YawR(i); YawAcc(i); Beta(i); BetaR(i)];
%out(:,i) = A * state;
%meas = [Fyf(i); Fyr(i); Vy(i); Ay(i); YawR(i); YawAcc(i); Beta(i); BetaR(i)];
%% ------------------ Call Meas Data From Module Outputs ----------------------
meas = [ukfmod_Fyf(i); ukfmod_Fyr(i); ukfmod_Vy(i); ukfmod_Ay(i); ukfmod_rYaw(i); ukfmod_aYaw(i); ukfmod_Beta(i); ukfmod_BetaR(i)];
%meas = [ukfmod_Fyf(i); ukfmod_Fyr(i); ukfmod_Vy(i); 0; ukfmod_rYaw(i); 0; ukfmod_Beta(i); 0];
%R = [ukfmod_Fyf_R(i); ukfmod_Fyr_R(i); ukfmod_Vy_R(i); 100; ukfmod_rYaw_R(i); 100; ukfmod_Beta_R(i); 100];
%% ------------------------Kalman Filter----------------------------------
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
estFyf = estimate(1,:);
estFyr = estimate(2,:);
estVy = estimate(3,:);
estAy = estimate(4,:);
estYawR = estimate(5,:);
estYawAcc = estimate(6,:);
estBeta = estimate(7,:);
estBetaR = estimate(8,:);
simTime = toc/321;
simTime

subplot(2,4,1)
plot(Time, estFyf, Time, ukfmod_Fyf, Time, Fyf)
xlabel("Time")
ylabel("Front Tire Force (N)")
legend("Processed", "Calculated", "True")
subplot(2,4,2)
plot(Time, estFyr, Time, ukfmod_Fyr, Time, Fyr)
xlabel("Time")
ylabel("Rear Tire Force (N)")
legend("Processed", "Calculated", "True")
subplot(2,4,3)
plot(Time, ukfmod_Vy, Time, ukfmod_Vy, Time, Vy)
xlabel("Time")
ylabel("Lat. Velocity (m/s)")
legend("Processed", "Calculated", "True")
subplot(2,4,4)
plot(Time, ukfmod_Ay, Time, ukfmod_Ay, Time, Ay)
xlabel("Time")
ylabel("Lat. Acc. (m/s^2)")
legend("Processed", "Calculated", "True")
subplot(2,4,5)
plot(Time, estYawR, Time, ukfmod_rYaw, Time, YawR)
xlabel("Time")
ylabel("Yaw Rate (rad/s)")
legend("Processed", "Calculated", "True")
subplot(2,4,6)
plot(Time, estYawAcc, Time, ukfmod_aYaw, Time, YawAcc)
xlabel("Time")
ylabel("Yaw Acceleration (rad/s^2)")
legend("Processed", "Calculated", "True")
subplot(2,4,7)
plot(Time, estBeta, Time, ukfmod_Beta, Time, Beta)
xlabel("Time")
ylabel("Slip Angle (rad)")
legend("Processed", "Calculated", "True")
subplot(2,4,8)
plot(Time, estBetaR, Time, ukfmod_BetaR, Time, BetaR)
xlabel("Time")
ylabel("Slip Rate (m/s)")
legend("Processed", "Calculated", "True")

%     dataProcEKF.Q = 0.01*eye(5,5);
%     dataProcEKF.R = 200*eye(5,5);
%     dataProcEKF.N = length(Time);
%     dataProcEKF.estimate = zeros(5,dataProcEKF.N);
%     dataProcEKF.noise = 0.3*randn(5,dataProcEKF.N); 
%     dataProcEKF.constS = [g Iz lf lr m];
%     dataProcEKF.constM = m;
%     dataProcEKF.meas = [MyFront.ukf1; FyRear.ukf1; VLateral.ukf; rYaw.ukf1;slipangle.ukf];
%     dataProcEKF.JacobW = 1;
%     dataProcEKF.JacobV = 1;
%     dataProcEKF.P = 0.1*eye(5,5);
%     dataProcEKF.estimate(:,1) = [Myf(1);Myr(1);Vy(1);YawR(1);Beta(1)]; 
%     dataProcEKF.trueS = [Myf';Myr';Vy';YawR';Beta'];
%     tic
%     for k=2:dataProcEKF.N
%         dataProcEKF.inpS = [AY(k-1) phi(k-1) Fyf(k-1) Fyr(k-1) delta(k-1) Vx(k-1)...
%                     YawAcc(k-1) YawR(k-1) BetaR(k-1)];
%  
%         dataProcEKF.inpM = [dataProcEKF.estimate(1,k-1) dataProcEKF.estimate(2,k-1) delta(k) ...
%                 Vx(k) dataProcEKF.estimate(3,k-1)];  
%     
%         dataProcEKF.JacobX = eye(5);%jacobianState(Ts,Vx(k-1),dataProcEKF.constS);
%     
%         dataProcEKF.JacobY = eye(5);%jacobianMeas(Vx(k-1));
%  
%         [dataProcEKF.estimate(:,k),dataProcEKF.P] = extendedkalman(Ts,dataProcEKF.P,'ekfState',...
%                 dataProcEKF.estimate(:,k-1),dataProcEKF.inpS,dataProcEKF.constS,dataProcEKF.JacobX,...
%                 'ekfMeas',dataProcEKF.inpM,dataProcEKF.constM,dataProcEKF.JacobY,...
%                 dataProcEKF.meas(:,k),dataProcEKF.Q,dataProcEKF.R,dataProcEKF.JacobW,dataProcEKF.JacobV);
%     end
%     dataProcEKF.ExecutionTime = toc;
% 
%     disp('General Extended Kalman Filter Execution Complete...');
%     disp('Plotting Outputs...');
% 
%     figure(2)
%     title('General EKF Plot')
% 
%     subplot(2,3,1)
%     plot(Time, delta)
%     ylabel('Steering Angle');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
% 
%     subplot(2,3,2)
%     plot(Time, Myf, Time, dataProcEKF.estimate(1,:))
%     ylabel('Myf');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
% 
%     subplot(2,3,3)
%     plot(Time, Myr, Time, dataProcEKF.estimate(2,:))
%     ylabel('Myr');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
% 
%     subplot(2,3,4)
%     plot(Time, Vy, Time, dataProcEKF.estimate(3,:))
%     ylabel('Vy');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
% 
%     subplot(2,3,5)
%     plot(Time, YawR, Time, dataProcEKF.estimate(4,:))
%     ylabel('YawR');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')
% 
%     subplot(2,3,6)
%     plot(Time, Beta, Time, dataProcEKF.estimate(5,:))
%     ylabel('Beta');
%     xlabel('Time');
%     xlim([0,Time(length(Time))])
%     legend('True', 'General EKF')


% for i = 1:length(Time)
% meas = [MyFront.ukf1(i); FyRear.ukf1(i); VLateral.ukf(i); rYaw.ukf1(i);slipangle.ukf(1)];
% end