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
vLat.iekf(1)=VY(1);
vLat.trueS=VY'; 

for k=2:vLat.N
 vLat.inpS=[AY(k-1) phi(k-1) Vx(k-1) YawR(k-1)]; %input for State Equation
 vLat.inpM=[vLat.iekf(k-1) Vx(k) YawR(k)];  
 %vLat.inpM=Beta(k);
[vLat.iekf(k),vLat.P(k)]=iterextendedkalman(iteration,Ts,vLat.P(k-1),'ekfVyState',...
    vLat.iekf(k-1),vLat.inpS,vLat.constS,vLat.JacobX,'ekfVyMeas2',...
    vLat.inpM,vLat.constM,vLat.JacobY,vLat.meas(k),vLat.Q,...
    vLat.R,vLat.JacobW,vLat.JacobV);
end
modules.iekf.elapsedTime.Vy=toc;
modules.iekf.Vy=vLat.iekf;
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
MLatFront.iekf(1)=Myf(1);
MLatFront.trueS=Myf'; 

for k=2:MLatFront.N
 MLatFront.inpS=[delta(k-1) Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; 
 MLatFront.inpM=[MLatFront.iekf(k-1) Fyr(k) delta(k) Vx(k) YawR(k)];  

[MLatFront.iekf(k),MLatFront.P(k)]=iterextendedkalman(iteration,Ts,MLatFront.P(k-1),...
    'ekfMyfState',MLatFront.iekf(k-1),MLatFront.inpS,MLatFront.constS,...
    MLatFront.JacobX,'ekfMyfMeas1',MLatFront.inpM,MLatFront.constM,...
    MLatFront.JacobY,MLatFront.meas(k),MLatFront.Q,MLatFront.R,...
    MLatFront.JacobW,MLatFront.JacobV);
end
modules.iekf.elapsedTime.Myf=toc;
modules.iekf.Myf=MLatFront.iekf;
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
MLatRear.iekf(1)=Myr(1);
MLatRear.trueS=Myr'; 

for k=2:MLatRear.N
 MLatRear.inpS=[Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; 
 MLatRear.inpM=[MLatRear.iekf(k-1) Fyf(k) delta(k) Vx(k) YawR(k)];  

[MLatRear.iekf(k),MLatRear.P(k)]=iterextendedkalman(iteration,Ts,MLatRear.P(k-1),...
    'ekfMyrState',MLatRear.iekf(k-1),MLatRear.inpS,MLatRear.constS,...
    MLatRear.JacobX,'ekfMyrMeas1',MLatRear.inpM,MLatRear.constM,...
    MLatRear.JacobY,MLatRear.meas(k),MLatRear.Q,MLatRear.R,...
    MLatRear.JacobW,MLatRear.JacobV);
end
modules.iekf.elapsedTime.Myr=toc;
modules.iekf.Myr=MLatRear.iekf;
true.Myr=MLatRear.trueS;

% ------------------------- Tire Force ---------------------------------
ekfEstMLatFront = modules.iekf.Myf;
ekfEstMLatRear = modules.iekf.Myr;
modules.iekf.Fyf=[0;diff(ekfEstMLatFront')]/Ts;
modules.iekf.Fyr=[0;diff(ekfEstMLatRear')]/Ts;
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
yawRate.iekf(1)=YawR(1);
yawRate.trueS=YawR'; 

for k=2:yawRate.N
 yawRate.inpS=[Fyf(k-1) Fyr(k-1) delta(k-1)]; 
 yawRate.inpM=[Fyf(k) Fyr(k) delta(k) Vx(k)];  

[yawRate.iekf(k),yawRate.P(k)]=iterextendedkalman(iteration,Ts,yawRate.P(k-1),...
    'ekfrYawState',yawRate.iekf(k-1),yawRate.inpS,yawRate.constS,...
    yawRate.JacobX,'ekfrYawMeas1',yawRate.inpM,yawRate.constM,...
    yawRate.JacobY,yawRate.meas(k),yawRate.Q,yawRate.R,...
    yawRate.JacobW,yawRate.JacobV);
end
modules.iekf.elapsedTime.YawR=toc;
modules.iekf.YawR=yawRate.iekf;
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
slipAngle.iekf(1)=Beta(1);
slipAngle.trueS=Beta'; 

for k=2:slipAngle.N
 slipAngle.inpS=[delta(k-1) phi(k-1) YawR(k-1) Vx(k-1) Fyf(k-1) Fyr(k-1)];
 slipAngle.inpM=[Vx(k)];  

[slipAngle.iekf(k),slipAngle.P(k)]=iterextendedkalman(iteration,Ts,slipAngle.P(k-1),...
    'ekfBetaState',slipAngle.iekf(k-1),slipAngle.inpS,slipAngle.constS,...
    slipAngle.JacobX,'ekfBetaMeas',slipAngle.inpM,slipAngle.constM,...
    slipAngle.JacobY,slipAngle.meas(k),slipAngle.Q,slipAngle.R,...
    slipAngle.JacobW,slipAngle.JacobV);
end
modules.iekf.elapsedTime.Beta=toc;
modules.iekf.Beta=slipAngle.iekf;
true.Beta=slipAngle.trueS;

xYawR = modules.iekf.YawR;
xBeta = modules.iekf.Beta;
xVy = modules.iekf.Vy;
modules.iekf.YawAcc = [0; diff(xYawR)'/Ts];
modules.iekf.Betar = [0; diff(xBeta)'/Ts];
modules.iekf.Ay = -[0; diff(xVy)'/Ts]+ Vx.*modules.iekf.YawR';
%-------------------------------------------------------------------------
%% ------------------------ UES w/o KF -----------------------------------
% ------------------------------------------------------------------------
tic
run('UES_IEKF.m')
ues.noKF.iekf.elapsedTime=toc/length(Time);
ues.noKF.iekf.Beta=uesIEkfComb.slipAngle.ekf;
ues.noKF.iekf.Vy=vLat.iekf;%%Variable Independent hence same as module
ues.noKF.iekf.MyF=uesIEkfComb.MLatFront.ekf;
ues.noKF.iekf.MyR=uesIEkfComb.MLatRear.ekf;
ues.noKF.iekf.YawR=uesIEkfComb.yawRate.ekf;
ues.noKF.iekf.FyF=uesIEkfFyf;
ues.noKF.iekf.FyR=uesIEkfFyr;
xYawR = uesIEkfComb.yawRate.ekf;
xBeta = uesIEkfComb.slipAngle.ekf;
xVy = ues.noKF.iekf.Vy;
ues.noKF.iekf.YawAcc = [0; diff(xYawR)'/Ts];
ues.noKF.iekf.Betar = [0; diff(xBeta)'/Ts];
ues.noKF.iekf.Ay = -[0; diff(xVy)'/Ts]+ Vx.*ues.noKF.iekf.YawR';

%-------------------------------------------------------------------------
%% ------------------------ UES w/ KF -----------------------------------
% ------------------------------------------------------------------------
dt = Ts;
c1 = m*lr/l;
c2 = Iz/l;
c3 = m*lf/l;
state = [];

% ----------------Out Data Processing UES-UKF -------------------------
iekfmod_Fyf = modules.iekf.Fyf;
iekfmod_Fyr = modules.iekf.Fyr;
iekfmod_rYaw = modules.iekf.YawR;
iekfmod_aYaw = modules.iekf.YawAcc;
iekfmod_Vy = modules.iekf.Vy;
iekfmod_Ay = modules.iekf.Ay;
iekfmod_Beta = modules.iekf.Beta;
iekfmod_BetaR = modules.iekf.Betar;
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
meas = [iekfmod_Fyf(i); iekfmod_Fyr(i); iekfmod_Vy(i); iekfmod_Ay(i); ...
    iekfmod_rYaw(i); iekfmod_aYaw(i); iekfmod_Beta(i); iekfmod_BetaR(i)];

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
ues.KF.iekf.Fyf = estimate(1,:);
ues.KF.iekf.Fyr = estimate(2,:);
ues.KF.iekf.Vy = estimate(3,:);
ues.KF.iekf.Ay = estimate(4,:);
ues.KF.iekf.YawR = estimate(5,:);
ues.KF.iekf.YawAcc = estimate(6,:);
ues.KF.iekf.Beta = estimate(7,:);
ues.KF.iekf.BetaR = estimate(8,:);
ues.KF.iekf.simTime = toc/321;

%% ---------------------- Save Results ----------------------------------
%save('EKF_DataSetName','true', 'modules','ues');


    
