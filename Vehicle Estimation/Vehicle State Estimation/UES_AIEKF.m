% clc; clear all
% run CarSim_Initialize.m
% g=9.8;
 Ts=Time(2);
 N=length(Time);
 noise=mul*AY.*randn(1,N);
 noise2=mul*VY.*randn(1,N);
%-------------------------------------Estimation of Vy---------------------------------------
%noise=0.0001*randn(1,uesAIEkfComb.vLat.N); %%%%%%CHECK MODEL EQUATION
uesAIEkfComb.vLat.constS=g;
uesAIEkfComb.vLat.JacobX=1;
uesAIEkfComb.vLat.constM=[];
uesAIEkfComb.vLat.JacobY=1;
uesAIEkfComb.vLat.meas=AY+noise';
uesAIEkfComb.vLat.JacobW=1;
uesAIEkfComb.vLat.JacobV=1;
uesAIEkfComb.vLat.ekf(1)=VY(1);
uesAIEkfComb.vLat.trueS=VY'; 
uesAIEkfComb.vLat.Q=[];
uesAIEkfComb.vLat.R=[];
%-------------------------------------Estimation of Myf---------------------------------------
uesAIEkfComb.MLatFront.N=length(Time);
%uesEkfYawR=0.001*randn(1,uesAIEkfComb.MLatFront.N); 
uesAIEkfComb.MLatFront.constS=[Iz m lr l];
uesAIEkfComb.MLatFront.JacobX=1;
uesAIEkfComb.MLatFront.constM=m;
uesAIEkfComb.MLatFront.JacobY=1;
uesAIEkfComb.MLatFront.meas=AY+noise';
uesAIEkfComb.MLatFront.JacobW=1;
uesAIEkfComb.MLatFront.JacobV=1;
uesAIEkfComb.MLatFront.ekf(1)=Myf(1);
uesAIEkfComb.MLatFront.trueS=Myf'; 
uesAIEkfComb.MLatFront.Q=[];
uesAIEkfComb.MLatFront.R=[];
%-------------------------------------Estimation of Myr---------------------------------------
uesAIEkfComb.MLatRear.N=length(Time);
%noise=0.001*randn(1,uesAIEkfComb.MLatRear.N); 
uesAIEkfComb.MLatRear.constS=[Iz m lr l];
uesAIEkfComb.MLatRear.JacobX=1;
uesAIEkfComb.MLatRear.constM=m;
uesAIEkfComb.MLatRear.JacobY=1;
uesAIEkfComb.MLatRear.meas=AY+noise';
uesAIEkfComb.MLatRear.JacobW=1;
uesAIEkfComb.MLatRear.JacobV=1;
uesAIEkfComb.MLatRear.ekf(1)=Myr(1);
uesAIEkfComb.MLatRear.trueS=Myr'; 
uesAIEkfComb.MLatRear.Q=[];
uesAIEkfComb.MLatRear.R=[];
%-------------------------------------Estimation of Yaw Rate---------------------------------------
uesAIEkfComb.yawRate.N=length(Time);
%noise=0.001*randn(1,uesAIEkfComb.yawRate.N); 
uesAIEkfComb.yawRate.constS=[Iz lf lr];
uesAIEkfComb.yawRate.JacobX=1;
uesAIEkfComb.yawRate.constM=m;
uesAIEkfComb.yawRate.JacobY=1;
uesAIEkfComb.yawRate.meas=AY+noise';
uesAIEkfComb.yawRate.JacobW=1;
uesAIEkfComb.yawRate.JacobV=1;
uesAIEkfComb.yawRate.ekf(1)=YawR(1);
uesAIEkfComb.yawRate.trueS=YawR';
uesAIEkfComb.yawAcc.trueS=YawAcc';
uesAIEkfComb.yawRate.Q=[];
uesAIEkfComb.yawRate.R=[];
%-------------------------------------Estimation of Slip Angle---------------------------------------
uesAIEkfComb.slipAngle.N=length(Time);
%noise=0.001*randn(1,uesAIEkfComb.slipAngle.N); 
uesAIEkfComb.slipAngle.constS=[g m];
uesAIEkfComb.slipAngle.JacobX=1;
uesAIEkfComb.slipAngle.constM=0;
uesAIEkfComb.slipAngle.JacobY=1;
uesAIEkfComb.slipAngle.meas=VY+noise2';
uesAIEkfComb.slipAngle.JacobW=1;
uesAIEkfComb.slipAngle.JacobV=1;
uesAIEkfComb.slipAngle.ekf(1)=Beta(1);
uesAIEkfComb.slipAngle.trueS=Beta'; 
uesAIEkfComb.slipAngle.Q=[];
uesAIEkfComb.slipAngle.R=[];

%----------------------------------Tuning Parameter------------------------
uesAIEkfComb.vLat.Q(1)=0.001;            
uesAIEkfComb.vLat.R(1)=scale*10000;
uesAIEkfComb.MLatFront.Q(1)=-0.000001;      
uesAIEkfComb.MLatFront.R(1)=scale*-100;
uesAIEkfComb.MLatRear.Q(1)=0.00000001;       
uesAIEkfComb.MLatRear.R(1)=scale*1;
uesAIEkfComb.slipAngle.Q(1)=0.01;      
uesAIEkfComb.slipAngle.R(1)=scale*10;
uesAIEkfComb.yawRate.Q(1)=0.00001;     
uesAIEkfComb.yawRate.R(1)=scale*0.1;

uesAIEkfComb.vLat.P(1)=-45;
uesAIEkfComb.MLatFront.P(1)=0;
uesAIEkfComb.MLatRear.P(1)=0;
uesAIEkfComb.slipAngle.P(1)=0;
uesAIEkfComb.yawRate.P(1)=0;

%--------Estimation Parameter Initialization-------------%
uesAIEkfFyf(1)=Fyf(1);
uesAIEkfFyr(1)=Fyr(1);
uesAIEkfBetaR(1)=BetaR(1);
uesAIEkfYawR(1)=YawR(1);
%estYawAcc(1)=YawAcc(1);

%%

for k=2:length(Time)
 uesAIEkfComb.slipAngle.inpS=[delta(k-1) phi(k-1) uesAIEkfYawR(k-1) Vx(k-1) uesAIEkfFyf(k-1) uesAIEkfFyr(k-1)]; %input for State Equation
 uesAIEkfComb.slipAngle.inpM=[Vx(k)];  
 
 uesAIEkfComb.vLat.inpS=[AY(k-1) phi(k-1) Vx(k-1) uesAIEkfYawR(k-1)]; %input for State Equation
 uesAIEkfComb.vLat.inpM=[uesAIEkfComb.vLat.ekf(k-1) Vx(k) uesAIEkfYawR(k-1)];  
 %uesAIEkfComb.vLat.inpM=Beta(k);
 
 uesAIEkfComb.MLatFront.inpS=[delta(k-1) Vx(k-1) 0 uesAIEkfYawR(k-1) YawAcc(k-1)]; %input for State Equation
 uesAIEkfComb.MLatFront.inpM=[uesAIEkfComb.MLatFront.ekf(k-1) uesAIEkfFyr(k-1) delta(k) Vx(k) uesAIEkfYawR(k-1)];  
 
 uesAIEkfComb.MLatRear.inpS=[Vx(k-1) 0 uesAIEkfYawR(k-1) YawAcc(k-1)]; %input for State Equation
 uesAIEkfComb.MLatRear.inpM=[uesAIEkfComb.MLatRear.ekf(k-1) uesAIEkfFyf(k-1) delta(k) Vx(k) uesAIEkfYawR(k-1)]; 
 
 uesAIEkfComb.yawRate.inpS=[uesAIEkfFyf(k-1) uesAIEkfFyr(k-1) delta(k-1)]; %input for State Equation
 uesAIEkfComb.yawRate.inpM=[uesAIEkfFyf(k-1) uesAIEkfFyr(k-1) delta(k) Vx(k)]; 
 
[uesAIEkfComb.slipAngle.ekf(k),uesAIEkfComb.slipAngle.P(k),uesAIEkfComb.slipAngle.Q(k),uesAIEkfComb.slipAngle.R(k)]=adaptiveiterextendedkalman(50,Ts,uesAIEkfComb.slipAngle.P(k-1),'ekfBetaState',uesAIEkfComb.slipAngle.ekf(k-1),uesAIEkfComb.slipAngle.inpS,uesAIEkfComb.slipAngle.constS,uesAIEkfComb.slipAngle.JacobX,'ekfBetaMeas',uesAIEkfComb.slipAngle.inpM,uesAIEkfComb.slipAngle.constM,uesAIEkfComb.slipAngle.JacobY,uesAIEkfComb.slipAngle.meas(k),uesAIEkfComb.slipAngle.Q(k-1),uesAIEkfComb.slipAngle.R(k-1),uesAIEkfComb.slipAngle.JacobW,uesAIEkfComb.slipAngle.JacobV);

 
[uesAIEkfComb.vLat.ekf(k),uesAIEkfComb.vLat.P(k),uesAIEkfComb.vLat.Q(k),uesAIEkfComb.vLat.R(k)]=adaptiveiterextendedkalman(50,Ts,uesAIEkfComb.vLat.P(k-1),'ekfVyState',uesAIEkfComb.vLat.ekf(k-1),uesAIEkfComb.vLat.inpS,uesAIEkfComb.vLat.constS,uesAIEkfComb.vLat.JacobX,'ekfVyMeas2',uesAIEkfComb.vLat.inpM,uesAIEkfComb.vLat.constM,uesAIEkfComb.vLat.JacobY,uesAIEkfComb.vLat.meas(k),uesAIEkfComb.vLat.Q(k-1),uesAIEkfComb.vLat.R(k-1),uesAIEkfComb.vLat.JacobW,uesAIEkfComb.vLat.JacobV);



[uesAIEkfComb.MLatFront.ekf(k),uesAIEkfComb.MLatFront.P(k),uesAIEkfComb.MLatFront.Q(k),uesAIEkfComb.MLatFront.R(k)]=adaptiveiterextendedkalman(50,Ts,uesAIEkfComb.MLatFront.P(k-1),'ekfMyfState',uesAIEkfComb.MLatFront.ekf(k-1),uesAIEkfComb.MLatFront.inpS,uesAIEkfComb.MLatFront.constS,uesAIEkfComb.MLatFront.JacobX,'ekfMyfMeas1',uesAIEkfComb.MLatFront.inpM,uesAIEkfComb.MLatFront.constM,uesAIEkfComb.MLatFront.JacobY,uesAIEkfComb.MLatFront.meas(k),uesAIEkfComb.MLatFront.Q(k-1),uesAIEkfComb.MLatFront.R(k-1),uesAIEkfComb.MLatFront.JacobW,uesAIEkfComb.MLatFront.JacobV);

  

[uesAIEkfComb.MLatRear.ekf(k),uesAIEkfComb.MLatRear.P(k),uesAIEkfComb.MLatRear.Q(k),uesAIEkfComb.MLatRear.R(k)]=adaptiveiterextendedkalman(50,Ts,uesAIEkfComb.MLatRear.P(k-1),'ekfMyrState',uesAIEkfComb.MLatRear.ekf(k-1),uesAIEkfComb.MLatRear.inpS,uesAIEkfComb.MLatRear.constS,uesAIEkfComb.MLatRear.JacobX,'ekfMyrMeas1',uesAIEkfComb.MLatRear.inpM,uesAIEkfComb.MLatRear.constM,uesAIEkfComb.MLatRear.JacobY,uesAIEkfComb.MLatRear.meas(k),uesAIEkfComb.MLatRear.Q(k-1),uesAIEkfComb.MLatRear.R(k-1),uesAIEkfComb.MLatRear.JacobW,uesAIEkfComb.MLatRear.JacobV);

 

[uesAIEkfComb.yawRate.ekf(k),uesAIEkfComb.yawRate.P(k),uesAIEkfComb.yawRate.Q(k),uesAIEkfComb.yawRate.R(k)]=adaptiveiterextendedkalman(50,Ts,uesAIEkfComb.yawRate.P(k-1),'ekfrYawState',uesAIEkfComb.yawRate.ekf(k-1),uesAIEkfComb.yawRate.inpS,uesAIEkfComb.yawRate.constS,uesAIEkfComb.yawRate.JacobX,'ekfrYawMeas1',uesAIEkfComb.yawRate.inpM,uesAIEkfComb.yawRate.constM,uesAIEkfComb.yawRate.JacobY,uesAIEkfComb.yawRate.meas(k),uesAIEkfComb.yawRate.Q(k-1),uesAIEkfComb.yawRate.R(k-1),uesAIEkfComb.yawRate.JacobW,uesAIEkfComb.yawRate.JacobV);

%-------------------Interdependency Declaration--------------------------%
uesAIEkfYawR(k)=uesAIEkfComb.yawRate.ekf(k);
uesAIEkfFyf(k)=(uesAIEkfComb.MLatFront.ekf(k)-uesAIEkfComb.MLatFront.ekf(k-1))/Ts;
uesAIEkfFyr(k)=(uesAIEkfComb.MLatRear.ekf(k)-uesAIEkfComb.MLatRear.ekf(k-1))/Ts;
uesAIEkfBetaR(k)=(uesAIEkfComb.slipAngle.ekf(k)-uesAIEkfComb.slipAngle.ekf(k-1))/Ts;%BetaR(k);%
%estYawAcc(k)=(uesAIEkfComb.yawRate.ekf(k)-uesAIEkfComb.yawRate.ekf(k-1))/Ts;
%EstYawAcc(k)=(uesAIEkfComb.yawRate.ekf(k)-uesAIEkfComb.yawRate.ekf(k-1))/Ts;
%plot(Time(k),EstYawAcc(k),Time(k),uesAIEkfComb.yawAcc.trueS(k))
%hold on
end

% figure(1)
% subplot(2,2,1)
% uesAIEkfComb.uesEkfYawRate=uesAIEkfComb.yawRate.ekf;
% uesAIEkfComb.trueyawRate=uesAIEkfComb.yawRate.trueS;
% plot(Time,uesAIEkfComb.uesEkfYawRate,Time,uesAIEkfComb.trueyawRate)
% title('Yaw Rate')
% subplot(2,2,2)
% uesAIEkfComb.estMLatRear=uesAIEkfComb.MLatRear.ekf;
% uesAIEkfComb.trueMLatRear=uesAIEkfComb.MLatRear.trueS;
% plot(Time,uesAIEkfComb.estMLatRear,Time,uesAIEkfComb.trueMLatRear)
% title('MLat Rear')
% subplot(2,2,3)
% uesAIEkfComb.estMLatFront=uesAIEkfComb.MLatFront.ekf;
% uesAIEkfComb.trueMLatFront=uesAIEkfComb.MLatFront.trueS;
% plot(Time,uesAIEkfComb.estMLatFront,Time,uesAIEkfComb.trueMLatFront)
% title('MLat Front')
% subplot(3,2,4) NOT WORKING
% uesAIEkfComb.estvLat=uesAIEkfComb.vLat.ekf;
% uesAIEkfComb.truevLat=uesAIEkfComb.vLat.trueS;
% plot(Time,-uesAIEkfComb.estvLat,Time,uesAIEkfComb.truevLat)
% title('Vy')
% subplot(2,2,4)
% uesAIEkfComb.estslipAngle=uesAIEkfComb.slipAngle.ekf;
% uesAIEkfComb.trueslipAngle=uesAIEkfComb.slipAngle.trueS;
% plot(Time,uesAIEkfComb.estslipAngle,Time,uesAIEkfComb.trueslipAngle)
% title('Slip Angle')
% 
% 
% figure(2)
% subplot(2,2,1)
% plot(Time,uesAIEkfComb.yawRate.R)
% title('Yaw Rate Estimator Noise Covariences')
% 
% subplot(2,2,2)
% plot(Time,uesAIEkfComb.MLatRear.R)
% title('MLat Rear Estimator Noise Covariences')
% 
% subplot(2,2,3)
% plot(Time,uesAIEkfComb.MLatFront.R)
% title('MLat Front Estimator Noise Covariences')
% 
% % subplot(3,2,4) NOT WORKING
% % plot(Time,uesAIEkfComb.vLat.R)
% % title('Vy Estimator Noise Covariences')
% 
% subplot(2,2,4)
% plot(Time,uesAIEkfComb.slipAngle.R)
% title('Slip Angle Estimator Noise Covariences')
% 
% % figure(2)
% % subplot(3,2,1)
% % plot(Time,uesAIEkfComb.yawRate.Q,Time,uesAIEkfComb.yawRate.R)
% % title('Yaw Rate Estimator Noise Covariences')
% % 
% % subplot(3,2,2)
% % plot(Time,uesAIEkfComb.MLatRear.Q,Time,uesAIEkfComb.MLatRear.R)
% % title('MLat Rear Estimator Noise Covariences')
% % 
% % subplot(3,2,3)
% % plot(Time,uesAIEkfComb.MLatFront.Q,Time,uesAIEkfComb.MLatFront.R)
% % title('MLat Front Estimator Noise Covariences')
% % 
% % subplot(3,2,4)
% % plot(Time,uesAIEkfComb.vLat.Q,Time,uesAIEkfComb.vLat.R)
% % title('Vy Estimator Noise Covariences')
% % 
% % subplot(3,2,5)
% % plot(Time,uesAIEkfComb.slipAngle.Q,Time,uesAIEkfComb.slipAngle.R)
% % title('Slip Angle Estimator Noise Covariences')