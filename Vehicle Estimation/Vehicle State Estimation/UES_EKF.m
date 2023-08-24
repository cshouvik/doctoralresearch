% clc; clear all
% run CarSim_Initialize.m
% g=9.8;
 Ts=Time(2);
 N=length(Time);
 noise=mul*AY.*randn(1,N);
 noise2=mul*VY.*randn(1,N);
% [state_ekf,P_ekf]=extendedkalman(Ts,pPrev,fstate,state,inpS,constS,JacobX,fmeas,inpM,constM,JacobY,meas,Q,R,JacobW,JacobV)
%-------------------------------------Estimation of Vy---------------------------------------
%noise=0.0001*randn(1,uesEkfComb.vLat.N); %%%%%%CHECK MODEL EQUATION
uesEkfComb.vLat.constS=g;
uesEkfComb.vLat.JacobX=1;
uesEkfComb.vLat.constM=[];
uesEkfComb.vLat.JacobY=1;
uesEkfComb.vLat.meas=AY+noise';
uesEkfComb.vLat.JacobW=1;
uesEkfComb.vLat.JacobV=1;
uesEkfComb.vLat.ekf(1)=VY(1);
uesEkfComb.vLat.trueS=VY'; 
%-------------------------------------Estimation of Myf---------------------------------------
uesEkfComb.MLatFront.N=length(Time);
%uesEkfYawR=0.001*randn(1,uesEkfComb.MLatFront.N); 
uesEkfComb.MLatFront.constS=[Iz m lr l];
uesEkfComb.MLatFront.JacobX=1;
uesEkfComb.MLatFront.constM=m;
uesEkfComb.MLatFront.JacobY=1;
uesEkfComb.MLatFront.meas=AY+noise';
uesEkfComb.MLatFront.JacobW=1;
uesEkfComb.MLatFront.JacobV=1;
uesEkfComb.MLatFront.ekf(1)=Myf(1);
uesEkfComb.MLatFront.trueS=Myf'; 
%-------------------------------------Estimation of Myr---------------------------------------
uesEkfComb.MLatRear.N=length(Time);
noise=0.001*randn(1,uesEkfComb.MLatRear.N); 
uesEkfComb.MLatRear.constS=[Iz m lr l];
uesEkfComb.MLatRear.JacobX=1;
uesEkfComb.MLatRear.constM=m;
uesEkfComb.MLatRear.JacobY=1;
uesEkfComb.MLatRear.meas=AY+noise';
uesEkfComb.MLatRear.JacobW=1;
uesEkfComb.MLatRear.JacobV=1;
uesEkfComb.MLatRear.ekf(1)=Myr(1);
uesEkfComb.MLatRear.trueS=Myr'; 
%-------------------------------------Estimation of Yaw Rate---------------------------------------
uesEkfComb.yawRate.N=length(Time);
%noise=0.001*randn(1,uesEkfComb.yawRate.N); 
uesEkfComb.yawRate.constS=[Iz lf lr];
uesEkfComb.yawRate.JacobX=1;
uesEkfComb.yawRate.constM=m;
uesEkfComb.yawRate.JacobY=1;
uesEkfComb.yawRate.meas=AY+noise';
uesEkfComb.yawRate.JacobW=1;
uesEkfComb.yawRate.JacobV=1;
uesEkfComb.yawRate.ekf(1)=YawR(1);
uesEkfComb.yawRate.trueS=YawR';
uesEkfComb.yawAcc.trueS=YawAcc';
%-------------------------------------Estimation of Slip Angle---------------------------------------
uesEkfComb.slipAngle.N=length(Time);
%noise=0.001*randn(1,uesEkfComb.slipAngle.N); 
uesEkfComb.slipAngle.constS=[g m];
uesEkfComb.slipAngle.JacobX=1;
uesEkfComb.slipAngle.constM=0;
uesEkfComb.slipAngle.JacobY=1;
uesEkfComb.slipAngle.meas=VY+noise2';
uesEkfComb.slipAngle.JacobW=1;
uesEkfComb.slipAngle.JacobV=1;
uesEkfComb.slipAngle.ekf(1)=Beta(1);
uesEkfComb.slipAngle.trueS=Beta'; 

%----------------------------------Tuning Parameter------------------------
uesEkfComb.vLat.Q=0.001;            
uesEkfComb.vLat.R=scale*10000;
uesEkfComb.MLatFront.Q=-0.000001;      
uesEkfComb.MLatFront.R=scale*-100;
uesEkfComb.MLatRear.Q=0.00000001;       
uesEkfComb.MLatRear.R=scale*1;
uesEkfComb.slipAngle.Q=0.01;      
uesEkfComb.slipAngle.R=scale*10;
uesEkfComb.yawRate.Q=0.00001;     
uesEkfComb.yawRate.R=scale*0.1;

uesEkfComb.vLat.P(1)=-45;
uesEkfComb.MLatFront.P(1)=0;
uesEkfComb.MLatRear.P(1)=0;
uesEkfComb.slipAngle.P(1)=0;
uesEkfComb.yawRate.P(1)=0;

%--------Estimation Parameter Initialization-------------%
uesEkfFyf(1)=Fyf(1);
uesEkfFyr(1)=Fyr(1);
uesEkfBetaR(1)=BetaR(1);
uesEkfYawR(1)=YawR(1);
%estYawAcc(1)=YawAcc(1);



for k=2:length(Time)
 uesEkfComb.slipAngle.inpS=[delta(k-1) phi(k-1) uesEkfYawR(k-1) Vx(k-1) uesEkfFyf(k-1) uesEkfFyr(k-1)]; %input for State Equation
 uesEkfComb.slipAngle.inpM=[Vx(k)];  
 
 uesEkfComb.vLat.inpS=[AY(k-1) phi(k-1) Vx(k-1) uesEkfYawR(k-1)]; %input for State Equation
 uesEkfComb.vLat.inpM=[uesEkfComb.vLat.ekf(k-1) Vx(k) uesEkfYawR(k-1)];  
 %uesEkfComb.vLat.inpM=Beta(k);
 
 uesEkfComb.MLatFront.inpS=[delta(k-1) Vx(k-1) 0 uesEkfYawR(k-1) YawAcc(k-1)]; %input for State Equation
 uesEkfComb.MLatFront.inpM=[uesEkfComb.MLatFront.ekf(k-1) uesEkfFyr(k-1) delta(k) Vx(k) uesEkfYawR(k-1)];  
 
 uesEkfComb.MLatRear.inpS=[Vx(k-1) 0 uesEkfYawR(k-1) YawAcc(k-1)]; %input for State Equation
 uesEkfComb.MLatRear.inpM=[uesEkfComb.MLatRear.ekf(k-1) uesEkfFyf(k-1) delta(k) Vx(k) uesEkfYawR(k-1)]; 
 
 uesEkfComb.yawRate.inpS=[uesEkfFyf(k-1) uesEkfFyr(k-1) delta(k-1)]; %input for State Equation
 uesEkfComb.yawRate.inpM=[uesEkfFyf(k-1) uesEkfFyr(k-1) delta(k) Vx(k)]; 
 
[uesEkfComb.slipAngle.ekf(k),uesEkfComb.slipAngle.P(k)]=extendedkalman(Ts,uesEkfComb.slipAngle.P(k-1),'ekfBetaState',uesEkfComb.slipAngle.ekf(k-1),uesEkfComb.slipAngle.inpS,uesEkfComb.slipAngle.constS,uesEkfComb.slipAngle.JacobX,'ekfBetaMeas',uesEkfComb.slipAngle.inpM,uesEkfComb.slipAngle.constM,uesEkfComb.slipAngle.JacobY,uesEkfComb.slipAngle.meas(k),uesEkfComb.slipAngle.Q,uesEkfComb.slipAngle.R,uesEkfComb.slipAngle.JacobW,uesEkfComb.slipAngle.JacobV);

 
[uesEkfComb.vLat.ekf(k),uesEkfComb.vLat.P(k)]=extendedkalman(Ts,uesEkfComb.vLat.P(k-1),'ekfVyState',uesEkfComb.vLat.ekf(k-1),uesEkfComb.vLat.inpS,uesEkfComb.vLat.constS,uesEkfComb.vLat.JacobX,'ekfVyMeas2',uesEkfComb.vLat.inpM,uesEkfComb.vLat.constM,uesEkfComb.vLat.JacobY,uesEkfComb.vLat.meas(k),uesEkfComb.vLat.Q,uesEkfComb.vLat.R,uesEkfComb.vLat.JacobW,uesEkfComb.vLat.JacobV);



[uesEkfComb.MLatFront.ekf(k),uesEkfComb.MLatFront.P(k)]=extendedkalman(Ts,uesEkfComb.MLatFront.P(k-1),'ekfMyfState',uesEkfComb.MLatFront.ekf(k-1),uesEkfComb.MLatFront.inpS,uesEkfComb.MLatFront.constS,uesEkfComb.MLatFront.JacobX,'ekfMyfMeas1',uesEkfComb.MLatFront.inpM,uesEkfComb.MLatFront.constM,uesEkfComb.MLatFront.JacobY,uesEkfComb.MLatFront.meas(k),uesEkfComb.MLatFront.Q,uesEkfComb.MLatFront.R,uesEkfComb.MLatFront.JacobW,uesEkfComb.MLatFront.JacobV);

  

[uesEkfComb.MLatRear.ekf(k),uesEkfComb.MLatRear.P(k)]=extendedkalman(Ts,uesEkfComb.MLatRear.P(k-1),'ekfMyrState',uesEkfComb.MLatRear.ekf(k-1),uesEkfComb.MLatRear.inpS,uesEkfComb.MLatRear.constS,uesEkfComb.MLatRear.JacobX,'ekfMyrMeas1',uesEkfComb.MLatRear.inpM,uesEkfComb.MLatRear.constM,uesEkfComb.MLatRear.JacobY,uesEkfComb.MLatRear.meas(k),uesEkfComb.MLatRear.Q,uesEkfComb.MLatRear.R,uesEkfComb.MLatRear.JacobW,uesEkfComb.MLatRear.JacobV);

 

[uesEkfComb.yawRate.ekf(k),uesEkfComb.yawRate.P(k)]=extendedkalman(Ts,uesEkfComb.yawRate.P(k-1),'ekfrYawState',uesEkfComb.yawRate.ekf(k-1),uesEkfComb.yawRate.inpS,uesEkfComb.yawRate.constS,uesEkfComb.yawRate.JacobX,'ekfrYawMeas1',uesEkfComb.yawRate.inpM,uesEkfComb.yawRate.constM,uesEkfComb.yawRate.JacobY,uesEkfComb.yawRate.meas(k),uesEkfComb.yawRate.Q,uesEkfComb.yawRate.R,uesEkfComb.yawRate.JacobW,uesEkfComb.yawRate.JacobV);

%-------------------Interdependency Declaration--------------------------%
uesEkfYawR(k)=uesEkfComb.yawRate.ekf(k);
uesEkfFyf(k)=(uesEkfComb.MLatFront.ekf(k)-uesEkfComb.MLatFront.ekf(k-1))/Ts;
uesEkfFyr(k)=(uesEkfComb.MLatRear.ekf(k)-uesEkfComb.MLatRear.ekf(k-1))/Ts;
uesEkfBetaR(k)=(uesEkfComb.slipAngle.ekf(k)-uesEkfComb.slipAngle.ekf(k-1))/Ts;%BetaR(k);%
%estYawAcc(k)=(uesEkfComb.yawRate.ekf(k)-uesEkfComb.yawRate.ekf(k-1))/Ts;
%EstYawAcc(k)=(uesEkfComb.yawRate.ekf(k)-uesEkfComb.yawRate.ekf(k-1))/Ts;
%plot(Time(k),EstYawAcc(k),Time(k),uesEkfComb.yawAcc.trueS(k))
%hold on
end

% subplot(3,2,1)
% uesEkfComb.uesEkfYawRate=uesEkfComb.yawRate.ekf;
% uesEkfComb.trueyawRate=uesEkfComb.yawRate.trueS;
% plot(Time,uesEkfComb.uesEkfYawRate,Time,uesEkfComb.trueyawRate)
% title('Yaw Rate(rad/s)')
% subplot(3,2,2)
% uesEkfComb.estMLatRear=uesEkfComb.MLatRear.ekf;
% uesEkfComb.trueMLatRear=uesEkfComb.MLatRear.trueS;
% plot(Time,uesEkfComb.estMLatRear,Time,uesEkfComb.trueMLatRear)
% title('MLat Rear(kg-m/s)')
% subplot(3,2,3)
% uesEkfComb.estMLatFront=uesEkfComb.MLatFront.ekf;
% uesEkfComb.trueMLatFront=uesEkfComb.MLatFront.trueS;
% plot(Time,uesEkfComb.estMLatFront,Time,uesEkfComb.trueMLatFront)
% title('MLat Front(kg-m/s)')
% subplot(3,2,4)
% uesEkfComb.estvLat=uesEkfComb.vLat.ekf;
% uesEkfComb.truevLat=uesEkfComb.vLat.trueS;
% plot(Time,-uesEkfComb.estvLat,Time,uesEkfComb.truevLat)
% title('Vy(m/s)')
% subplot(3,2,5)
% uesEkfComb.estslipAngle=uesEkfComb.slipAngle.ekf;
% uesEkfComb.trueslipAngle=uesEkfComb.slipAngle.trueS;
% plot(Time,uesEkfComb.estslipAngle,Time,uesEkfComb.trueslipAngle)
% title('Slip Angle(rad)')
% 
% subplot(3,2,6)
% plot(Time,delta)
% xlabel = "Time";
% title('Steering Angle(rad)')








