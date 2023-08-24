% clc; clear all
% run CarSim_Initialize.m
% g=9.8;
 Ts=Time(2);
 N=length(Time);
 noise=mul*AY.*randn(1,N);
 noise2=mul*VY.*randn(1,N);
%-------------------------------------Estimation of Vy---------------------------------------
%noise=0.0001*randn(1,uesEkfComb.vLat.N); %%%%%%CHECK MODEL EQUATION
uesIEkfComb.vLat.constS=g;
uesIEkfComb.vLat.JacobX=1;
uesIEkfComb.vLat.constM=[];
uesIEkfComb.vLat.JacobY=1;
uesIEkfComb.vLat.meas=AY+noise';
uesIEkfComb.vLat.JacobW=1;
uesIEkfComb.vLat.JacobV=1;
uesIEkfComb.vLat.ekf(1)=VY(1);
uesIEkfComb.vLat.trueS=VY'; 
%-------------------------------------Estimation of Myf---------------------------------------
uesIEkfComb.MLatFront.N=length(Time);
%uesEkfYawR=0.001*randn(1,uesEkfComb.MLatFront.N); 
uesIEkfComb.MLatFront.constS=[Iz m lr l];
uesIEkfComb.MLatFront.JacobX=1;
uesIEkfComb.MLatFront.constM=m;
uesIEkfComb.MLatFront.JacobY=1;
uesIEkfComb.MLatFront.meas=AY+noise';
uesIEkfComb.MLatFront.JacobW=1;
uesIEkfComb.MLatFront.JacobV=1;
uesIEkfComb.MLatFront.ekf(1)=Myf(1);
uesIEkfComb.MLatFront.trueS=Myf'; 
%-------------------------------------Estimation of Myr---------------------------------------
uesIEkfComb.MLatRear.N=length(Time);
%noise=0.001*randn(1,uesEkfComb.MLatRear.N); 
uesIEkfComb.MLatRear.constS=[Iz m lr l];
uesIEkfComb.MLatRear.JacobX=1;
uesIEkfComb.MLatRear.constM=m;
uesIEkfComb.MLatRear.JacobY=1;
uesIEkfComb.MLatRear.meas=AY+noise';
uesIEkfComb.MLatRear.JacobW=1;
uesIEkfComb.MLatRear.JacobV=1;
uesIEkfComb.MLatRear.ekf(1)=Myr(1);
uesIEkfComb.MLatRear.trueS=Myr'; 
%-------------------------------------Estimation of Yaw Rate---------------------------------------
uesIEkfComb.yawRate.N=length(Time);
%noise=0.001*randn(1,uesEkfComb.yawRate.N); 
uesIEkfComb.yawRate.constS=[Iz lf lr];
uesIEkfComb.yawRate.JacobX=1;
uesIEkfComb.yawRate.constM=m;
uesIEkfComb.yawRate.JacobY=1;
uesIEkfComb.yawRate.meas=AY+noise';
uesIEkfComb.yawRate.JacobW=1;
uesIEkfComb.yawRate.JacobV=1;
uesIEkfComb.yawRate.ekf(1)=YawR(1);
uesIEkfComb.yawRate.trueS=YawR';
uesIEkfComb.yawAcc.trueS=YawAcc';
%-------------------------------------Estimation of Slip Angle---------------------------------------
uesIEkfComb.slipAngle.N=length(Time);
%noise=0.001*randn(1,uesEkfComb.slipAngle.N); 
uesIEkfComb.slipAngle.constS=[g m];
uesIEkfComb.slipAngle.JacobX=1;
uesIEkfComb.slipAngle.constM=0;
uesIEkfComb.slipAngle.JacobY=1;
uesIEkfComb.slipAngle.meas=VY+noise2';
uesIEkfComb.slipAngle.JacobW=1;
uesIEkfComb.slipAngle.JacobV=1;
uesIEkfComb.slipAngle.ekf(1)=Beta(1);
uesIEkfComb.slipAngle.trueS=Beta'; 

%----------------------------------Tuning Parameter------------------------
uesIEkfComb.vLat.Q=0.001;            
uesIEkfComb.vLat.R=scale*10000;
uesIEkfComb.MLatFront.Q=-0.000001;      
uesIEkfComb.MLatFront.R=scale*-100;
uesIEkfComb.MLatRear.Q=0.00000001;       
uesIEkfComb.MLatRear.R=scale*1;
uesIEkfComb.slipAngle.Q=0.01;      
uesIEkfComb.slipAngle.R=scale*10;
uesIEkfComb.yawRate.Q=0.00001;     
uesIEkfComb.yawRate.R=scale*0.1;

uesIEkfComb.vLat.P(1)=-45;
uesIEkfComb.MLatFront.P(1)=0;
uesIEkfComb.MLatRear.P(1)=0;
uesIEkfComb.slipAngle.P(1)=0;
uesIEkfComb.yawRate.P(1)=0;

%--------Estimation Parameter Initialization-------------%
uesIEkfFyf(1)=Fyf(1);
uesIEkfFyr(1)=Fyr(1);
uesIEkfBetaR(1)=BetaR(1);
uesIEkfYawR(1)=YawR(1);
%estYawAcc(1)=YawAcc(1);

%%

for k=2:length(Time)
 uesIEkfComb.slipAngle.inpS=[delta(k-1) phi(k-1) uesIEkfYawR(k-1) Vx(k-1) uesIEkfFyf(k-1) uesIEkfFyr(k-1)]; %input for State Equation
 uesIEkfComb.slipAngle.inpM=[Vx(k)];  
 
 uesIEkfComb.vLat.inpS=[AY(k-1) phi(k-1) Vx(k-1) uesIEkfYawR(k-1)]; %input for State Equation
 uesIEkfComb.vLat.inpM=[uesIEkfComb.vLat.ekf(k-1) Vx(k) uesIEkfYawR(k-1)];  
 %uesEkfComb.vLat.inpM=Beta(k);
 
 uesIEkfComb.MLatFront.inpS=[delta(k-1) Vx(k-1) 0 uesIEkfYawR(k-1) YawAcc(k-1)]; %input for State Equation
 uesIEkfComb.MLatFront.inpM=[uesIEkfComb.MLatFront.ekf(k-1) uesIEkfFyr(k-1) delta(k) Vx(k) uesIEkfYawR(k-1)];  
 
 uesIEkfComb.MLatRear.inpS=[Vx(k-1) 0 uesIEkfYawR(k-1) YawAcc(k-1)]; %input for State Equation
 uesIEkfComb.MLatRear.inpM=[uesIEkfComb.MLatRear.ekf(k-1) uesIEkfFyf(k-1) delta(k) Vx(k) uesIEkfYawR(k-1)]; 
 
 uesIEkfComb.yawRate.inpS=[uesIEkfFyf(k-1) uesIEkfFyr(k-1) delta(k-1)]; %input for State Equation
 uesIEkfComb.yawRate.inpM=[uesIEkfFyf(k-1) uesIEkfFyr(k-1) delta(k) Vx(k)]; 
 
[uesIEkfComb.slipAngle.ekf(k),uesIEkfComb.slipAngle.P(k)]=iterextendedkalman(10,Ts,uesIEkfComb.slipAngle.P(k-1),'ekfBetaState',uesIEkfComb.slipAngle.ekf(k-1),uesIEkfComb.slipAngle.inpS,uesIEkfComb.slipAngle.constS,uesIEkfComb.slipAngle.JacobX,'ekfBetaMeas',uesIEkfComb.slipAngle.inpM,uesIEkfComb.slipAngle.constM,uesIEkfComb.slipAngle.JacobY,uesIEkfComb.slipAngle.meas(k),uesIEkfComb.slipAngle.Q,uesIEkfComb.slipAngle.R,uesIEkfComb.slipAngle.JacobW,uesIEkfComb.slipAngle.JacobV);

 
[uesIEkfComb.vLat.ekf(k),uesIEkfComb.vLat.P(k)]=iterextendedkalman(10,Ts,uesIEkfComb.vLat.P(k-1),'ekfVyState',uesIEkfComb.vLat.ekf(k-1),uesIEkfComb.vLat.inpS,uesIEkfComb.vLat.constS,uesIEkfComb.vLat.JacobX,'ekfVyMeas2',uesIEkfComb.vLat.inpM,uesIEkfComb.vLat.constM,uesIEkfComb.vLat.JacobY,uesIEkfComb.vLat.meas(k),uesIEkfComb.vLat.Q,uesIEkfComb.vLat.R,uesIEkfComb.vLat.JacobW,uesIEkfComb.vLat.JacobV);



[uesIEkfComb.MLatFront.ekf(k),uesIEkfComb.MLatFront.P(k)]=iterextendedkalman(10,Ts,uesIEkfComb.MLatFront.P(k-1),'ekfMyfState',uesIEkfComb.MLatFront.ekf(k-1),uesIEkfComb.MLatFront.inpS,uesIEkfComb.MLatFront.constS,uesIEkfComb.MLatFront.JacobX,'ekfMyfMeas1',uesIEkfComb.MLatFront.inpM,uesIEkfComb.MLatFront.constM,uesIEkfComb.MLatFront.JacobY,uesIEkfComb.MLatFront.meas(k),uesIEkfComb.MLatFront.Q,uesIEkfComb.MLatFront.R,uesIEkfComb.MLatFront.JacobW,uesIEkfComb.MLatFront.JacobV);

  

[uesIEkfComb.MLatRear.ekf(k),uesIEkfComb.MLatRear.P(k)]=iterextendedkalman(10,Ts,uesIEkfComb.MLatRear.P(k-1),'ekfMyrState',uesIEkfComb.MLatRear.ekf(k-1),uesIEkfComb.MLatRear.inpS,uesIEkfComb.MLatRear.constS,uesIEkfComb.MLatRear.JacobX,'ekfMyrMeas1',uesIEkfComb.MLatRear.inpM,uesIEkfComb.MLatRear.constM,uesIEkfComb.MLatRear.JacobY,uesIEkfComb.MLatRear.meas(k),uesIEkfComb.MLatRear.Q,uesIEkfComb.MLatRear.R,uesIEkfComb.MLatRear.JacobW,uesIEkfComb.MLatRear.JacobV);

 

[uesIEkfComb.yawRate.ekf(k),uesIEkfComb.yawRate.P(k)]=iterextendedkalman(10,Ts,uesIEkfComb.yawRate.P(k-1),'ekfrYawState',uesIEkfComb.yawRate.ekf(k-1),uesIEkfComb.yawRate.inpS,uesIEkfComb.yawRate.constS,uesIEkfComb.yawRate.JacobX,'ekfrYawMeas1',uesIEkfComb.yawRate.inpM,uesIEkfComb.yawRate.constM,uesIEkfComb.yawRate.JacobY,uesIEkfComb.yawRate.meas(k),uesIEkfComb.yawRate.Q,uesIEkfComb.yawRate.R,uesIEkfComb.yawRate.JacobW,uesIEkfComb.yawRate.JacobV);

%-------------------Interdependency Declaration--------------------------%
uesIEkfYawR(k)=uesIEkfComb.yawRate.ekf(k);
uesIEkfFyf(k)=(uesIEkfComb.MLatFront.ekf(k)-uesIEkfComb.MLatFront.ekf(k-1))/Ts;
uesIEkfFyr(k)=(uesIEkfComb.MLatRear.ekf(k)-uesIEkfComb.MLatRear.ekf(k-1))/Ts;
uesIEkfBetaR(k)=(uesIEkfComb.slipAngle.ekf(k)-uesIEkfComb.slipAngle.ekf(k-1))/Ts;%BetaR(k);%
%estYawAcc(k)=(uesEkfComb.yawRate.ekf(k)-uesEkfComb.yawRate.ekf(k-1))/Ts;
%EstYawAcc(k)=(uesEkfComb.yawRate.ekf(k)-uesEkfComb.yawRate.ekf(k-1))/Ts;
%plot(Time(k),EstYawAcc(k),Time(k),uesEkfComb.yawAcc.trueS(k))
%hold on
end

% subplot(3,2,1)
% uesIEkfComb.uesEkfYawRate=uesIEkfComb.yawRate.ekf;
% uesIEkfComb.trueyawRate=uesIEkfComb.yawRate.trueS;
% plot(Time,uesIEkfComb.uesEkfYawRate,Time,uesIEkfComb.trueyawRate)
% title('Yaw Rate(rad/s)')
% subplot(3,2,2)
% uesIEkfComb.estMLatRear=uesIEkfComb.MLatRear.ekf;
% uesIEkfComb.trueMLatRear=uesIEkfComb.MLatRear.trueS;
% plot(Time,uesIEkfComb.estMLatRear,Time,uesIEkfComb.trueMLatRear)
% title('MLat Rear(kg-m/s)')
% subplot(3,2,3)
% uesIEkfComb.estMLatFront=uesIEkfComb.MLatFront.ekf;
% uesIEkfComb.trueMLatFront=uesIEkfComb.MLatFront.trueS;
% plot(Time,uesIEkfComb.estMLatFront,Time,uesIEkfComb.trueMLatFront)
% title('MLat Front(kg-m/s)')
% subplot(3,2,4)% NOT Working
% uesIEkfComb.estvLat=uesIEkfComb.vLat.ekf;
% uesIEkfComb.truevLat=uesIEkfComb.vLat.trueS;
% plot(Time,-uesIEkfComb.estvLat,Time,uesIEkfComb.truevLat)
% title('Lateral Velocity(m/s)')
% subplot(3,2,5)
% uesIEkfComb.estslipAngle=uesIEkfComb.slipAngle.ekf;
% uesIEkfComb.trueslipAngle=uesIEkfComb.slipAngle.trueS;
% plot(Time,uesIEkfComb.estslipAngle,Time,uesIEkfComb.trueslipAngle)
% title('Slip Angle(rad)')
% subplot(3,2,6)
% plot(Time, delta)
% title('Steering Angle(rad)')







