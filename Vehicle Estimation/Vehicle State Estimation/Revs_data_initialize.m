clc; clear all;
uiopen
rad=pi/180;
%% ----------------- Extract values from revs dataset --------------------
m=params.mass.value;
lf=params.trackFront.value;
lr=params.trackRear.value;
l=lf+lr;
mf=params.massFront.value;
Iz=mf*lf^2;
Time=insData.sideSlip.time;
Beta=insData.sideSlip.value*rad;
phi=insData.rollAngle.value*rad;
Vx=insData.vxCG.value;
Vy=insData.vyCG.value;
YawRfilt=insData.yawRateFilt.value*rad;
%rPhi_meas=insData.rollRate.value*rad;
%aPhi=insData.rollAngAcc.value*rad;
% Ax=insData.axCGFilt.value;
Ayfilt=insData.ayCGFilt.value;
YawAccfilt=insData.yawAngAccFilt.value*rad;
Ax=insData.axCG.value;
Ay=insData.ayCG.value;
YawAcc=insData.yawAngAcc.value*rad;
YawR=insData.yawRate.value*rad;

%% ---------------- Segregating values from revs dataset -----------------
Beta=Beta(5000:5:30000);
phi=phi(5000:5:30000);
Vx=Vx(5000:5:30000);
Vy=Vy(5000:5:30000);
YawR=YawR(5000:5:30000);
Time=Time(1:5:25001);
BetaR=[0;diff(Beta)/Time(2)];
YawAcc=YawAcc(5000:5:30000);
Ax=Ax(5000:5:30000);
Ay=Ay(5000:5:30000);
AyFilt = Ayfilt(5000:5:30000);
YawRFilt = YawRfilt(5000:5:30000);
YawAccFilt = YawAccfilt(5000:5:30000);
% Ax=[0;diff(Vx)/Time(2)];
% Ay=[0;diff(Vy)/Time(2)];
% YawAcc=[0;diff(YawR)/Time(2)];

k=1;
for i=0:50:length(tireData.roadWheelAngle.time)
    if i==0
        j=1;
    else
        j=i;
    end
delta_L1(k,1)=tireData.roadWheelAngleFL.value(j,1);
delta_R1(k,1)=tireData.roadWheelAngleFR.value(j,1);
deltan(k,1)=tireData.roadWheelAngle.value(j,1);
k=k+1;
end
deltaFL=delta_L1(1:length(Time))*rad;
deltaFR=delta_R1(1:length(Time))*rad;
delta=deltan(5000:5:30000)*rad;

alpha_1=lr.*m.*Vx./(l.*cos(delta));
Fyf=(alpha_1.*BetaR+alpha_1.*YawR+alpha_1.*(Iz./(lr.*m.*Vx)).*YawAcc);
alpha_2=lf.*m.*Vx./l;
Fyr=(alpha_2.*BetaR+alpha_2.*YawR-(Iz./l).*YawAcc);
Tsample=Time(2)-Time(1);
Fyf(isnan(Fyf))=0;
Fyr(isnan(Fyr))=0;
g = 9.8;
Ts = Time(2);
%% Calcultaing Myf and Myr
TStop=Time(length(Time));
FYF=[Time Fyf];
FYR=[Time Fyr];
options=simset('Solver','FixedStepDiscrete','FixedStep','Tsample');
[x,y]=sim('myf_myr_extract',TStop,simset('Solver','ode1','FixedStep','Tsample'));


filename='REVS_Grandsport01';
save(filename,'Time','Vx','Vy','Ax','Ay','YawR','YawAcc','Beta','BetaR','phi',...
    'm','lf','lr','l','mf','Iz','delta','Fyf','Fyr','Myf','Myr','AyFilt',...
    'YawRFilt','YawAccFilt','g','Ts');
clear all
load('REVS_Grandsport01')
AY = Ay;
VY = Vy;

