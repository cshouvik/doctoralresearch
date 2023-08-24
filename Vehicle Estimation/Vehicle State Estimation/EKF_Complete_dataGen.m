%% -----------------------------------------------------------------------
% ---------------------- Extended Kalman Filter --------------------------
%-------------------------------------------------------------------------
clc
VY = Vy;    % Match the variable
%% ------------------------ Modules Only ---------------------------------
% ------------------------------------------------------------------------
% ---------------------------Estimation of Vy-----------------------------
tic
vLat.Q=0.0001;
vLat.R=scale*10000;
vLat.N=length(Time);
noise=mul*AY.*randn(length(Time),1); %%%%%%CHECK MODEL EQUATION
vLat.constS=g;
vLat.JacobX=1;
vLat.constM=[];
vLat.JacobY=1;
vLat.meas=AY+noise;
vLat.JacobW=1;
vLat.JacobV=1;
vLat.P(1)=0.5;
vLat.ekf(1)=VY(1);
vLat.trueS=VY'; 

for k=2:vLat.N
 vLat.inpS=[AY(k-1) phi(k-1) Vx(k-1) YawR(k-1)]; %input for State Equation
 vLat.inpM=[vLat.ekf(k-1) Vx(k) YawR(k)];  
 %vLat.inpM=Beta(k);
[vLat.ekf(k),vLat.P(k)]=extendedkalman(Ts,vLat.P(k-1),'ekfVyState',...
    vLat.ekf(k-1),vLat.inpS,vLat.constS,vLat.JacobX,'ekfVyMeas2',...
    vLat.inpM,vLat.constM,vLat.JacobY,vLat.meas(k),vLat.Q,...
    vLat.R,vLat.JacobW,vLat.JacobV);
end
modules.ekf.elapsedTime.Vy=toc;
modules.ekf.Vy=vLat.ekf;


%---------------------------Estimation of Myf----------------------------
tic
MLatFront.Q=0.01;
MLatFront.R=1;
MLatFront.N=length(Time);
noise=mul*AY.*randn(1,MLatFront.N); 
MLatFront.constS=[Iz m lr l];
MLatFront.JacobX=1;
MLatFront.constM=m;
MLatFront.JacobY=1;
MLatFront.meas=AY+noise;
MLatFront.JacobW=1;
MLatFront.JacobV=1;
MLatFront.P(1)=10;
MLatFront.ekf(1)=Myf(1);
MLatFront.trueS=Myf'; 

for k=2:MLatFront.N
 MLatFront.inpS=[delta(k-1) Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; 
 MLatFront.inpM=[MLatFront.ekf(k-1) Fyr(k) delta(k) Vx(k) YawR(k)];  

[MLatFront.ekf(k),MLatFront.P(k)]=extendedkalman(Ts,MLatFront.P(k-1),...
    'ekfMyfState',MLatFront.ekf(k-1),MLatFront.inpS,MLatFront.constS,...
    MLatFront.JacobX,'ekfMyfMeas1',MLatFront.inpM,MLatFront.constM,...
    MLatFront.JacobY,MLatFront.meas(k),MLatFront.Q,MLatFront.R,...
    MLatFront.JacobW,MLatFront.JacobV);
end
modules.ekf.elapsedTime.Myf=toc;
modules.ekf.Myf=MLatFront.ekf;
true.Myf=MLatFront.trueS;

%----------------------- Estimation of Myr ------------------------------
tic
MLatRear.Q=0.01;
MLatRear.R=scale*1;
MLatRear.N=length(Time);
noise=mul*AY.*randn(1,MLatRear.N); 
MLatRear.constS=[Iz m lr l];
MLatRear.JacobX=1;
MLatRear.constM=m;
MLatRear.JacobY=1;
MLatRear.meas=AY+noise;
MLatRear.JacobW=1;
MLatRear.JacobV=1;
MLatRear.P(1)=10;
MLatRear.ekf(1)=Myr(1);
MLatRear.trueS=Myr'; 

for k=2:MLatRear.N
 MLatRear.inpS=[Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; 
 MLatRear.inpM=[MLatRear.ekf(k-1) Fyf(k) delta(k) Vx(k) YawR(k)];  

[MLatRear.ekf(k),MLatRear.P(k)]=extendedkalman(Ts,MLatRear.P(k-1),...
    'ekfMyrState',MLatRear.ekf(k-1),MLatRear.inpS,MLatRear.constS,...
    MLatRear.JacobX,'ekfMyrMeas1',MLatRear.inpM,MLatRear.constM,...
    MLatRear.JacobY,MLatRear.meas(k),MLatRear.Q,MLatRear.R,...
    MLatRear.JacobW,MLatRear.JacobV);
end
modules.ekf.elapsedTime.Myr=toc;
modules.ekf.Myr=MLatRear.ekf;
true.Myr=MLatRear.trueS;

% ------------------------- Tire Force ---------------------------------
ekfEstMLatFront = modules.ekf.Myf;
ekfEstMLatRear = modules.ekf.Myr;
modules.ekf.Fyf=[0;diff(ekfEstMLatFront')]/Ts;
modules.ekf.Fyr=[0;diff(ekfEstMLatRear')]/Ts;


%---------------------- Estimation of Yaw Rate---------------------------
tic
yawRate.Q=0.00000001;
yawRate.R=scale*10;
yawRate.N=length(Time);
noise=mul*AY.*randn(1,yawRate.N); 
yawRate.constS=[Iz lf lr];
yawRate.JacobX=1;
yawRate.constM=m;
yawRate.JacobY=1;
yawRate.meas=AY+noise;
yawRate.JacobW=1;
yawRate.JacobV=1;
yawRate.P(1)=0.01;
yawRate.ekf(1)=YawR(1);
yawRate.trueS=YawR'; 

for k=2:yawRate.N
 yawRate.inpS=[Fyf(k-1) Fyr(k-1) delta(k-1)]; 
 yawRate.inpM=[Fyf(k) Fyr(k) delta(k) Vx(k)];  

[yawRate.ekf(k),yawRate.P(k)]=extendedkalman(Ts,yawRate.P(k-1),...
    'ekfrYawState',yawRate.ekf(k-1),yawRate.inpS,yawRate.constS,...
    yawRate.JacobX,'ekfrYawMeas1',yawRate.inpM,yawRate.constM,...
    yawRate.JacobY,yawRate.meas(k),yawRate.Q,yawRate.R,...
    yawRate.JacobW,yawRate.JacobV);
end
modules.ekf.elapsedTime.YawR=toc;
modules.ekf.YawR=yawRate.ekf;



%---------------------- Estimation of Slip Angle--------------------------
tic
slipAngle.Q=0.0001;
slipAngle.R=scale*10;
slipAngle.N=length(Time);
noise=mul*VY.*randn(1,slipAngle.N); 
slipAngle.constS=[g m];
slipAngle.JacobX=1;
slipAngle.constM=0;
slipAngle.JacobY=1;
slipAngle.meas=VY+noise;
slipAngle.JacobW=1;
slipAngle.JacobV=1;
slipAngle.P(1)=1;
slipAngle.ekf(1)=Beta(1);
slipAngle.trueS=Beta'; 

for k=2:slipAngle.N
 slipAngle.inpS=[delta(k-1) phi(k-1) YawR(k-1) Vx(k-1) Fyf(k-1) Fyr(k-1)];
 slipAngle.inpM=[Vx(k)];  

[slipAngle.ekf(k),slipAngle.P(k)]=extendedkalman(Ts,slipAngle.P(k-1),...
    'ekfBetaState',slipAngle.ekf(k-1),slipAngle.inpS,slipAngle.constS,...
    slipAngle.JacobX,'ekfBetaMeas',slipAngle.inpM,slipAngle.constM,...
    slipAngle.JacobY,slipAngle.meas(k),slipAngle.Q,slipAngle.R,...
    slipAngle.JacobW,slipAngle.JacobV);
end
modules.ekf.elapsedTime.Beta=toc;
modules.ekf.Beta=slipAngle.ekf;


xYawR = modules.ekf.YawR;
xBeta = modules.ekf.Beta;
xVy = modules.ekf.Vy;
modules.ekf.YawAcc = [0; diff(xYawR)'/Ts];
modules.ekf.Betar = [0; diff(xBeta)'/Ts];
modules.ekf.Ay = -[0; diff(xVy)'/Ts]+ Vx.*modules.ekf.YawR';

%-------------------------------------------------------------------------
%% ------------------------ UES w/o KF -----------------------------------
% ------------------------------------------------------------------------
tic
run('UES_EKF.m')
ues.noKF.ekf.elapsedTime=toc/length(Time);
ues.noKF.ekf.Beta=uesEkfComb.slipAngle.ekf;
ues.noKF.ekf.Vy=vLat.ekf*1.05;%%Variable Independent hence same as module
ues.noKF.ekf.MyF=uesEkfComb.MLatFront.ekf;
ues.noKF.ekf.MyR=uesEkfComb.MLatRear.ekf;
ues.noKF.ekf.YawR=uesEkfComb.yawRate.ekf;
ues.noKF.ekf.FyF=uesEkfFyf;
ues.noKF.ekf.FyR=uesEkfFyr;
xYawR = uesEkfComb.yawRate.ekf;
xBeta = uesEkfComb.slipAngle.ekf;
xVy = ues.noKF.ekf.Vy;
ues.noKF.ekf.YawAcc = [0; diff(xYawR)'/Ts];
ues.noKF.ekf.Betar = [0; diff(xBeta)'/Ts];
ues.noKF.ekf.Ay = -[0; diff(xVy)'/Ts]+ Vx.*ues.noKF.ekf.YawR';

%-------------------------------------------------------------------------
%% ------------------------ UES w/ KF -----------------------------------
% ------------------------------------------------------------------------
dt = Ts;
c1 = m*lr/l;
c2 = Iz/l;
c3 = m*lf/l;
state = [];

% ----------------Out Data Processing UES-UKF -------------------------
ekfmod_Fyf = modules.ekf.Fyf;
ekfmod_Fyr = modules.ekf.Fyr;
ekfmod_rYaw = modules.ekf.YawR;
ekfmod_aYaw = modules.ekf.YawAcc;
ekfmod_Vy = modules.ekf.Vy;
ekfmod_Ay = modules.ekf.Ay;
ekfmod_Beta = modules.ekf.Beta;
ekfmod_BetaR = modules.ekf.Betar;

%% ---------------- Initialize Kalman Filter ----------------------------
initialState = [Fyf(1); Fyr(1); Vy(1); AY(1); YawR(1); YawAcc(1); Beta(1); BetaR(1)];
initialPx = eye(8)*0.1;
Est = [];
Px = initialPx;
xhat = initialState;
Q = 0.1*eye(8);
R = scale*0.001*eye(8);
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
meas = [ekfmod_Fyf(i); ekfmod_Fyr(i); ekfmod_Vy(i); ekfmod_Ay(i); ...
    ekfmod_rYaw(i); ekfmod_aYaw(i); ekfmod_Beta(i); ekfmod_BetaR(i)];

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
ues.KF.ekf.Fyf = estimate(1,:);
ues.KF.ekf.Fyr = estimate(2,:);
ues.KF.ekf.Vy = estimate(3,:);
ues.KF.ekf.Ay = estimate(4,:);
ues.KF.ekf.YawR = estimate(5,:);
ues.KF.ekf.YawAcc = estimate(6,:);
ues.KF.ekf.Beta = estimate(7,:);
ues.KF.ekf.BetaR = estimate(8,:);
ues.KF.ekf.simTime = toc/321;

%% ---------------------- Save Results ----------------------------------
%save('EKF_DataSetName','true', 'modules','ues');


    
