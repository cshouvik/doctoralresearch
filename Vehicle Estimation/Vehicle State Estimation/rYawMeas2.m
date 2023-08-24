
function YawR = rYawMeas(Ts,rYaw,inp,const)

%% Description
% Ts=>Sampling Time
% inp=>vector containing all inp variables
% const=> vector containing all constant terms
% xin=> prev state information as inp
m=const;
Fyf=inp(1);
Fyr=inp(2);
delta=inp(3);
Vx=inp(4);

%Ay=(1/m)*(-Fyf.*cos(delta)+Fyr+m*rYaw.*Vx);
YawR = rYaw;
