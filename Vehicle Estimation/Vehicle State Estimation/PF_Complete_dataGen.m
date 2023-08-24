%pf complete datagen
%% -----------------------------------------------------------------------
% --------------------------- Particle Filter ----------------------------
%-------------------------------------------------------------------------
clc
VY = Vy;    % Match the variable
%% ------------------------ Modules Only ---------------------------------
% ------------------------------------------------------------------------
% ---------------------------Estimation of Vy-----------------------------
tic
vLat.Q=0.00000001;
vLat.R=scale*1000000;
vLat.N=length(Time);
noise=mul*AY.*randn(1,vLat.N); %%%%%%CHECK MODEL EQUATION
vLat.constS=g;
vLat.constM=[];
vLat.meas=AY+noise';
vLat.P=0.000000001;
vLat.pf_xEst(1)=VY(1);
vLat.trueS=VY';
vLat.NoOfParticles = 100;

for k= 2 : vLat.N
    x_Particle = [];
    for i = 1 : vLat.NoOfParticles
        x_Particle(1,i) = vLat.pf_xEst(1,k-1) + 0.01*sqrt(vLat.P) .* randn(1);		% Generate Particles Randomly
    end
    
    vLat.inpS=[AY(k-1) phi(k-1) Vx(k-1) YawR(k-1)]; %input for State Equation
    vLat.inpM=[vLat.pf_xEst(k-1) Vx(k) YawR(k)];
    x_particle_update = [];
    meas_update = [];
    P_w = [];
    % Propagate particles
    for i = 1:vLat.NoOfParticles
        x_particle_update(:,i) = ekfVyState(Ts,x_Particle(:,i),vLat.inpS,g) + sqrt(vLat.Q);
        meas_update(:,i) = ekfVyMeas2(Ts,x_particle_update(:,1),vLat.inpM,[]);
        P_w(:,i) = (1./sqrt(2*pi*vLat.R)) .* exp(-(vLat.meas(k) - meas_update(:,i)).^2./(2*vLat.R));
    end 

    % Normalize to form probability distribution
    P_w_norm = [];
    for i = 1:length(P_w(:,1))
        P_w_norm(i,:) = P_w(i,:)/sum(P_w(i,:));
    end
    
%     for i = 1 : vLat.NoOfParticles
%         x_particle(1,i) = x_particle_update(find(rand(vLat.NoOfParticles) <= cumsum(P_w(:,i)),1));
%     end
    
    %resample = zeros(1,10);
         population = [1:vLat.NoOfParticles];
         kz = vLat.NoOfParticles;
         resample = randsample(population,kz,'true',P_w_norm);
         %The final estimate is the mean value or variance
         xresampled = x_particle_update(1,resample);
   
    
    
       xEst = P_w_norm*xresampled';

    vLat.P = mean(P_w);
    vLat.pf_xEst(k) = xEst;
end
modules.pf.elapsedTime.Vy=toc;
modules.pf.Vy=vLat.pf_xEst;
% plot(modules.pf.Vy)
% hold on
% plot(Vy)
% hold off

%---------------------------Estimation of Myf----------------------------
tic
MLatFront.Q=0.01;
MLatFront.R=scale*1;
MLatFront.N=length(Time);
noise=mul*AY.*randn(1,MLatFront.N); 
MLatFront.constS=[Iz m lr l];
MLatFront.constM=m;
MLatFront.meas=AY+noise';
MLatFront.P=10;
MLatFront.pf_xEst(1)=Myf(1);
MLatFront.trueS=Myf'; 
MLatFront.NoOfParticles = 100;
for k=2:MLatFront.N
    x_Particle = [];
    for i = 1 : vLat.NoOfParticles
        x_Particle(1,i) = MLatFront.pf_xEst(1,k-1) + sqrt(MLatFront.P) .* randn(1);		% Generate Particles Randomly
    end
 MLatFront.inpS=[delta(k-1) Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; 
 MLatFront.inpM=[MLatFront.pf_xEst(1,k-1) Fyr(k) delta(k) Vx(k) YawR(k)];  


x_particle_update = [];
    meas_update = [];
    P_w = [];
    % Propagate particles
    for i = 1:vLat.NoOfParticles
        x_particle_update(:,i) = ekfMyfState(Ts,x_Particle(:,i),MLatFront.inpS,MLatFront.constS) + sqrt(MLatFront.Q);
        meas_update(:,i) = ekfMyfMeas1(Ts,x_particle_update(:,1),MLatFront.inpM,MLatFront.constM);
        P_w(:,i) = (1./sqrt(2*pi*MLatFront.R)) .* exp(-(MLatFront.meas(k) - meas_update(:,i)).^2./(2*MLatFront.R));
    end 

    % Normalize to form probability distribution
    P_w_norm = [];
    for i = 1:length(P_w(:,1))
        P_w_norm(i,:) = P_w(i,:)/sum(P_w(i,:));
    end
    
%     for i = 1 : vLat.NoOfParticles
%         x_particle(1,i) = x_particle_update(find(rand(vLat.NoOfParticles) <= cumsum(P_w(:,i)),1));
%     end
    
    %resample = zeros(1,10);
         population = [1:MLatFront.NoOfParticles];
         kz = MLatFront.NoOfParticles;
         resample = randsample(population,kz,'true',P_w_norm);
         %The final estimate is the mean value or variance
         xresampled = x_particle_update(1,resample);
   
    
    
       xEst = P_w_norm*xresampled';


    MLatFront.pf_xEst(k) = xEst;
end
modules.pf.elapsedTime.Myf=toc;
modules.pf.Myf=MLatFront.pf_xEst;
% plot(modules.pf.Myf)
% hold on
% plot(Myf)
% hold off

%---------------------------Estimation of Myr----------------------------
tic
MLatRear.Q=0.01;
MLatRear.R=scale*1;
MLatRear.N=length(Time);
noise=mul*AY.*randn(1,MLatRear.N); 
MLatRear.constS=[Iz m lr l];
MLatRear.constM=m;
MLatRear.meas=AY+noise';
MLatRear.P=10;
MLatRear.pf_xEst(1)=Myr(1);
MLatRear.trueS=Myr'; 
MLatRear.NoOfParticles = 100;
for k=2:MLatRear.N
    x_Particle = [];
    for i = 1 : vLat.NoOfParticles
        x_Particle(1,i) = MLatRear.pf_xEst(1,k-1) + sqrt(MLatRear.P) .* randn(1);		% Generate Particles Randomly
    end
 MLatRear.inpS=[Vx(k-1) BetaR(k-1) YawR(k-1) YawAcc(k-1)]; 
 MLatRear.inpM=[MLatRear.pf_xEst(1,k-1) Fyf(k) delta(k) Vx(k) YawR(k)];  


x_particle_update = [];
    meas_update = [];
    P_w = [];
    % Propagate particles
    for i = 1:vLat.NoOfParticles
        x_particle_update(:,i) = ekfMyrState(Ts,x_Particle(:,i),MLatRear.inpS,MLatRear.constS) + sqrt(MLatRear.Q);
        meas_update(:,i) = ekfMyrMeas1(Ts,x_particle_update(:,1),MLatRear.inpM,MLatRear.constM);
        P_w(:,i) = (1./sqrt(2*pi*MLatRear.R)) .* exp(-(MLatRear.meas(k) - meas_update(:,i)).^2./(2*MLatRear.R));
    end 

    % Normalize to form probability distribution
    P_w_norm = [];
    for i = 1:length(P_w(:,1))
        P_w_norm(i,:) = P_w(i,:)/sum(P_w(i,:));
    end
    
%     for i = 1 : vLat.NoOfParticles
%         x_particle(1,i) = x_particle_update(find(rand(vLat.NoOfParticles) <= cumsum(P_w(:,i)),1));
%     end
    
    %resample = zeros(1,10);
         population = [1:MLatRear.NoOfParticles];
         kz = MLatRear.NoOfParticles;
         resample = randsample(population,kz,'true',P_w_norm);
         %The final estimate is the mean value or variance
         xresampled = x_particle_update(1,resample);
   
    
    
       xEst = P_w_norm*xresampled';


    MLatRear.pf_xEst(k) = xEst;
end
modules.pf.elapsedTime.Myr=toc;
modules.pf.Myr=MLatRear.pf_xEst;
% plot(modules.pf.Myr)
% hold on
% plot(Myr)
% hold off
% ------------------------- Tire Force ---------------------------------
pfEstMLatFront = modules.pf.Myf;
pfEstMLatRear = modules.pf.Myr;
modules.pf.Fyf=[0;diff(pfEstMLatFront')]/Ts;
modules.pf.Fyr=[0;diff(pfEstMLatRear')]/Ts;

%---------------------------Estimation of YawR----------------------------
tic
yawRate.Q=0.00000001;
yawRate.R=scale*10000000;
yawRate.N=length(Time);
noise=mul*AY.*randn(1,yawRate.N); 
yawRate.constS=[Iz lf lr];
yawRate.constM=m;
yawRate.meas=AY+noise';
yawRate.P=0.000000001;
yawRate.pf_xEst(1)=YawR(1);
yawRate.trueS=YawR'; 
yawRate.NoOfParticles = 100;
for k=2:yawRate.N
    x_Particle = [];
    for i = 1 : vLat.NoOfParticles
        x_Particle(1,i) = yawRate.pf_xEst(1,k-1) + 0.01*sqrt(yawRate.P) .* randn(1);		% Generate Particles Randomly
    end
 yawRate.inpS=[Fyf(k-1) Fyr(k-1) delta(k-1)]; 
 yawRate.inpM=[Fyf(k) Fyr(k) delta(k) Vx(k)];  



x_particle_update = [];
    meas_update = [];
    P_w = [];
    % Propagate particles
    for i = 1:vLat.NoOfParticles
        x_particle_update(:,i) = ekfrYawState(Ts,x_Particle(:,i),yawRate.inpS,yawRate.constS) + sqrt(yawRate.Q);
        meas_update(:,i) = ekfrYawMeas1(Ts,x_particle_update(:,1),yawRate.inpM,yawRate.constM);
        P_w(:,i) = (1./sqrt(2*pi*yawRate.R)) .* exp(-(yawRate.meas(k) - meas_update(:,i)).^2./(2*yawRate.R));
    end 

    % Normalize to form probability distribution
    P_w_norm = [];
    for i = 1:length(P_w(:,1))
        P_w_norm(i,:) = P_w(i,:)/sum(P_w(i,:));
    end
    
%     for i = 1 : vLat.NoOfParticles
%         x_particle(1,i) = x_particle_update(find(rand(vLat.NoOfParticles) <= cumsum(P_w(:,i)),1));
%     end
    
    %resample = zeros(1,10);
         population = [1:yawRate.NoOfParticles];
         kz = yawRate.NoOfParticles;
         resample = randsample(population,kz,'true',P_w_norm);
         %The final estimate is the mean value or variance
         xresampled = x_particle_update(1,resample);
   
    
    
       xEst = P_w_norm*xresampled';


    yawRate.pf_xEst(k) = xEst;
end
modules.pf.elapsedTime.YawR=toc;
modules.pf.YawR=yawRate.pf_xEst;
% plot(modules.pf.YawR)
% hold on
% plot(YawR)
% hold off

%---------------------------Estimation of Beta----------------------------
tic
slipAngle.Q=0.00000000001;
slipAngle.R=scale*1000000;
slipAngle.N=length(Time);
noise=mul*VY.*randn(1,slipAngle.N); 
slipAngle.constS=[g m];
slipAngle.constM=0;
slipAngle.meas=VY+noise';
slipAngle.P=0.00000001;
slipAngle.pf_xEst(1)=Beta(1);
slipAngle.trueS=Beta'; 
slipAngle.NoOfParticles = 100;
for k=2:slipAngle.N
    x_Particle = [];
    for i = 1 : vLat.NoOfParticles
        x_Particle(1,i) = slipAngle.pf_xEst(1,k-1) + 0.01*sqrt(slipAngle.P) .* randn(1);		% Generate Particles Randomly
    end
 slipAngle.inpS=[delta(k-1) phi(k-1) YawR(k-1) Vx(k-1) Fyf(k-1) Fyr(k-1)];
 slipAngle.inpM=[Vx(k)]; 


x_particle_update = [];
    meas_update = [];
    P_w = [];
    % Propagate particles
    for i = 1:vLat.NoOfParticles
        x_particle_update(:,i) = ekfBetaState(Ts,x_Particle(:,i),slipAngle.inpS,slipAngle.constS) + sqrt(slipAngle.Q);
        meas_update(:,i) = ekfBetaMeas(Ts,x_particle_update(:,1),slipAngle.inpM,slipAngle.constM);
        P_w(:,i) = (1./sqrt(2*pi*slipAngle.R)) .* exp(-(slipAngle.meas(k) - meas_update(:,i)).^2./(2*slipAngle.R));
    end 

    % Normalize to form probability distribution
    P_w_norm = [];
    for i = 1:length(P_w(:,1))
        P_w_norm(i,:) = P_w(i,:)/sum(P_w(i,:));
    end
    
%     for i = 1 : vLat.NoOfParticles
%         x_particle(1,i) = x_particle_update(find(rand(vLat.NoOfParticles) <= cumsum(P_w(:,i)),1));
%     end
    
    %resample = zeros(1,10);
         population = [1:slipAngle.NoOfParticles];
         kz = slipAngle.NoOfParticles;
         resample = randsample(population,kz,'true',P_w_norm);
         %The final estimate is the mean value or variance
         xresampled = x_particle_update(1,resample);
   
    
    
       xEst = P_w_norm*xresampled';


    slipAngle.pf_xEst(k) = xEst;
end
modules.pf.elapsedTime.Beta=toc;
modules.pf.Beta=slipAngle.pf_xEst;
% plot(modules.pf.Beta)
% hold on
% plot(Beta)
% hold off
xYawR = modules.pf.YawR;
xBeta = modules.pf.Beta;
xVy = modules.pf.Vy;
modules.pf.YawAcc = [0; diff(xYawR)'/Ts];
modules.pf.Betar = [0; diff(xBeta)'/Ts];
modules.pf.Ay = -[0; diff(xVy)'/Ts]+ Vx.*modules.pf.YawR';
%-------------------------------------------------------------------------
%% ------------------------ UES w/ KF -----------------------------------
% ------------------------------------------------------------------------
dt = Ts;
c1 = m*lr/l;
c2 = Iz/l;
c3 = m*lf/l;
state = [];

% ----------------Out Data Processing UES-UKF -------------------------
pfmod_Fyf = modules.pf.Fyf;
pfmod_Fyr = modules.pf.Fyr;
pfmod_rYaw = modules.pf.YawR;
pfmod_aYaw = modules.pf.YawAcc;
pfmod_Vy = modules.pf.Vy;
pfmod_Ay = modules.pf.Ay;
pfmod_Beta = modules.pf.Beta;
pfmod_BetaR = modules.pf.Betar;

%% ---------------- Initialize Kalman Filter ----------------------------
initialState = [Fyf(1); Fyr(1); Vy(1); AY(1); YawR(1); YawAcc(1); Beta(1); BetaR(1)];
initialPx = eye(8)*0.1;
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
meas = [pfmod_Fyf(i); pfmod_Fyr(i); pfmod_Vy(i); pfmod_Ay(i); ...
    pfmod_rYaw(i); pfmod_aYaw(i); pfmod_Beta(i); pfmod_BetaR(i)];

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
ues.KF.pf.Fyf = estimate(1,:);
ues.KF.pf.Fyr = estimate(2,:);
ues.KF.pf.Vy = estimate(3,:);
ues.KF.pf.Ay = estimate(4,:);
ues.KF.pf.YawR = estimate(5,:);
ues.KF.pf.YawAcc = estimate(6,:);
ues.KF.pf.Beta = estimate(7,:);
ues.KF.pf.BetaR = estimate(8,:);
ues.KF.pf.simTime = toc/321;