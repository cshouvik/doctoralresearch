%% -----------------------------------------------------------------------
% ---------------------- Extended Kalman Filter --------------------------
%-------------------------------------------------------------------------
clc
iteration = 10;
VY = Vy;    % Match the variable
%% ------------------------ Modules Only ---------------------------------
% ------------------------------------------------------------------------
% ---------------------------Estimation of Vy-----------------------------
tic
vLat.Q=0.0001;
vLat.R=scale*10000;
vLat.N=length(Time);
noise=mul*AY.*randn(1,vLat.N); %%%%%%CHECK MODEL EQUATION
vLat.constS=g;
vLat.JacobX=1;
vLat.constM=[];
vLat.JacobY=1;
vLat.meas=AY+noise';
vLat.JacobW=1;
vLat.JacobV=1;
vLat.P(1)=0.5;
vLat.aiekf(1)=VY(1);
vLat.trueS=VY'; 

for k=2:vLat.N
 vLat.inpS=[AY(k-1) phi(k-1) Vx(k-1) YawR(k-1)]; %input for State Equation
 vLat.inpM=[vLat.aiekf(k-1) Vx(k) YawR(k)];  
 %vLat.inpM=Beta(k);
[vLat.aiekf(k),vLat.P(k)]=iterextendedkalman(iteration,Ts,vLat.P(k-1),'ekfVyState',...
    vLat.aiekf(k-1),vLat.inpS,vLat.constS,vLat.JacobX,'ekfVyMeas2',...
    vLat.inpM,vLat.constM,vLat.JacobY,vLat.meas(k),vLat.Q,...
    vLat.R,vLat.JacobW,vLat.JacobV);
end
modules.aiekf.elapsedTime.Vy=toc;
modules.aiekf.Vy=vLat.aiekf;
true.Vy=vLat.trueS;

%---------------------------Estimation of Myf----------------------------
tic
MLatFront.Q=0.01;
MLatFront.R=scale*1;
MLatFront.N=length(Time);
noise=mul*AY.*randn(1,MLatFront.N); 
MLatFront.constS=[Iz m lr l];
MLatFront.JacobX=1;
MLatFront.constM=m;
MLatFront.JacobY=1;
MLatFront.meas=AY+noise';
MLatFront.JacobW=1;
MLatFront.JacobV=1;
MLatFront.P(1)=10;
MLatFront.aiekf(1)=Myf(1);
MLatFront.trueS=Myf'; 

for k=2:MLatFront.N
 MLatFront.inpS=[delta(k-1) Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; 
 MLatFront.inpM=[MLatFront.aiekf(k-1) Fyr(k) delta(k) Vx(k) YawR(k)];  

[MLatFront.aiekf(k),MLatFront.P(k)]=iterextendedkalman(iteration,Ts,MLatFront.P(k-1),...
    'ekfMyfState',MLatFront.aiekf(k-1),MLatFront.inpS,MLatFront.constS,...
    MLatFront.JacobX,'ekfMyfMeas1',MLatFront.inpM,MLatFront.constM,...
    MLatFront.JacobY,MLatFront.meas(k),MLatFront.Q,MLatFront.R,...
    MLatFront.JacobW,MLatFront.JacobV);
end
modules.aiekf.elapsedTime.Myf=toc;
modules.aiekf.Myf=MLatFront.aiekf;
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
MLatRear.meas=AY+noise';
MLatRear.JacobW=1;
MLatRear.JacobV=1;
MLatRear.P(1)=10;
MLatRear.aiekf(1)=Myr(1);
MLatRear.trueS=Myr'; 

for k=2:MLatRear.N
 MLatRear.inpS=[Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; 
 MLatRear.inpM=[MLatRear.aiekf(k-1) Fyf(k) delta(k) Vx(k) YawR(k)];  

[MLatRear.aiekf(k),MLatRear.P(k)]=iterextendedkalman(iteration,Ts,MLatRear.P(k-1),...
    'ekfMyrState',MLatRear.aiekf(k-1),MLatRear.inpS,MLatRear.constS,...
    MLatRear.JacobX,'ekfMyrMeas1',MLatRear.inpM,MLatRear.constM,...
    MLatRear.JacobY,MLatRear.meas(k),MLatRear.Q,MLatRear.R,...
    MLatRear.JacobW,MLatRear.JacobV);
end
modules.aiekf.elapsedTime.Myr=toc;
modules.aiekf.Myr=MLatRear.aiekf;
true.Myr=MLatRear.trueS;

% ------------------------- Tire Force ---------------------------------
ekfEstMLatFront = modules.aiekf.Myf;
ekfEstMLatRear = modules.aiekf.Myr;
modules.aiekf.Fyf=[0;diff(ekfEstMLatFront')]/Ts;
modules.aiekf.Fyr=[0;diff(ekfEstMLatRear')]/Ts;
true.Fyf = Fyf;
true.Fyr = Fyr;

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
yawRate.meas=AY+noise';
yawRate.JacobW=1;
yawRate.JacobV=1;
yawRate.P(1)=0.01;
yawRate.aiekf(1)=YawR(1);
yawRate.trueS=YawR'; 

for k=2:yawRate.N
 yawRate.inpS=[Fyf(k-1) Fyr(k-1) delta(k-1)]; 
 yawRate.inpM=[Fyf(k) Fyr(k) delta(k) Vx(k)];  

[yawRate.aiekf(k),yawRate.P(k)]=iterextendedkalman(iteration,Ts,yawRate.P(k-1),...
    'ekfrYawState',yawRate.aiekf(k-1),yawRate.inpS,yawRate.constS,...
    yawRate.JacobX,'ekfrYawMeas1',yawRate.inpM,yawRate.constM,...
    yawRate.JacobY,yawRate.meas(k),yawRate.Q,yawRate.R,...
    yawRate.JacobW,yawRate.JacobV);
end
modules.aiekf.elapsedTime.YawR=toc;
modules.aiekf.YawR=yawRate.aiekf;
true.YawR=yawRate.trueS;


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
slipAngle.meas=VY+noise';
slipAngle.JacobW=1;
slipAngle.JacobV=1;
slipAngle.P(1)=1;
slipAngle.aiekf(1)=Beta(1);
slipAngle.trueS=Beta'; 

for k=2:slipAngle.N
 slipAngle.inpS=[delta(k-1) phi(k-1) YawR(k-1) Vx(k-1) Fyf(k-1) Fyr(k-1)];
 slipAngle.inpM=[Vx(k)];  

[slipAngle.aiekf(k),slipAngle.P(k)]=iterextendedkalman(iteration,Ts,slipAngle.P(k-1),...
    'ekfBetaState',slipAngle.aiekf(k-1),slipAngle.inpS,slipAngle.constS,...
    slipAngle.JacobX,'ekfBetaMeas',slipAngle.inpM,slipAngle.constM,...
    slipAngle.JacobY,slipAngle.meas(k),slipAngle.Q,slipAngle.R,...
    slipAngle.JacobW,slipAngle.JacobV);
end
modules.aiekf.elapsedTime.Beta=toc;
modules.aiekf.Beta=slipAngle.aiekf;
true.Beta=slipAngle.trueS;

xYawR = modules.aiekf.YawR;
xBeta = modules.aiekf.Beta;
xVy = modules.aiekf.Vy;
modules.aiekf.YawAcc = [0; diff(xYawR)'/Ts];
modules.aiekf.Betar = [0; diff(xBeta)'/Ts];
modules.aiekf.Ay = -[0; diff(xVy)'/Ts]+ Vx.*modules.aiekf.YawR';
%-------------------------------------------------------------------------
%% ------------------------ UES w/o KF -----------------------------------
% ------------------------------------------------------------------------
tic
run('UES_AIEKF.m')
ues.noKF.aiekf.elapsedTime=toc/length(Time);
ues.noKF.aiekf.Beta=uesAIEkfComb.slipAngle.ekf;
ues.noKF.aiekf.Vy=vLat.aiekf;%%Variable Independent hence same as module
ues.noKF.aiekf.MyF=uesAIEkfComb.MLatFront.ekf;
ues.noKF.aiekf.MyR=uesAIEkfComb.MLatRear.ekf;
ues.noKF.aiekf.YawR=uesAIEkfComb.yawRate.ekf;
ues.noKF.aiekf.FyF=uesAIEkfFyf;
ues.noKF.aiekf.FyR=uesAIEkfFyr;
xYawR = uesAIEkfComb.yawRate.ekf;
xBeta = uesAIEkfComb.slipAngle.ekf;
xVy = ues.noKF.aiekf.Vy;
ues.noKF.aiekf.YawAcc = [0; diff(xYawR)'/Ts];
ues.noKF.aiekf.Betar = [0; diff(xBeta)'/Ts];
ues.noKF.aiekf.Ay = -[0; diff(xVy)'/Ts]+ Vx.*modules.aiekf.YawR';

%-------------------------------------------------------------------------
%% ------------------------ UES w/ KF -----------------------------------
% ------------------------------------------------------------------------
dt = Ts;
c1 = m*lr/l;
c2 = Iz/l;
c3 = m*lf/l;
state = [];

% ----------------Out Data Processing UES-UKF -------------------------
aiekfmod_Fyf = modules.aiekf.Fyf;
aiekfmod_Fyr = modules.aiekf.Fyr;
aiekfmod_rYaw = modules.aiekf.YawR;
aiekfmod_aYaw = modules.aiekf.YawAcc;
aiekfmod_Vy = modules.aiekf.Vy;
aiekfmod_Ay = modules.aiekf.Ay;
aiekfmod_Beta = modules.aiekf.Beta;
aiekfmod_BetaR = modules.aiekf.Betar;
%% ---------------- Initialize Kalman Filter ----------------------------
initialState = [Fyf(1); Fyr(1); Vy(1); AY(1); YawR(1); YawAcc(1); Beta(1); BetaR(1)];
initialPx = eye(8)*0.1;
Est = [];
Px = initialPx;
xhat = initialState;
Q = 0.01*eye(8);
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
meas = [aiekfmod_Fyf(i); aiekfmod_Fyr(i); aiekfmod_Vy(i); aiekfmod_Ay(i); ...
    aiekfmod_rYaw(i); aiekfmod_aYaw(i); aiekfmod_Beta(i); aiekfmod_BetaR(i)];

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
ues.KF.aiekf.Fyf = estimate(1,:);
ues.KF.aiekf.Fyr = estimate(2,:);
ues.KF.aiekf.Vy = estimate(3,:);
ues.KF.aiekf.Ay = estimate(4,:);
ues.KF.aiekf.YawR = estimate(5,:);
ues.KF.aiekf.YawAcc = estimate(6,:);
ues.KF.aiekf.Beta = estimate(7,:);
ues.KF.aiekf.BetaR = estimate(8,:);
ues.KF.aiekf.simTime = toc/321;

%% ---------------------- Save Results ----------------------------------
%save('EKF_DataSetName','true', 'modules','ues');


    
