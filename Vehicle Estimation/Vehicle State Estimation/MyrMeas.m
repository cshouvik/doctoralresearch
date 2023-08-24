
function Ay = MyrMeas1(Ts,Myr,inp,const)

%% Description
% Ts=>Sampling Time
% inp=>vector containing all inp variables
% const=> vector containing all constant terms
% xin=> prev state information as inp
m=const;
Myrprev=inp(1);
Fyf=inp(2);
delta=inp(3);
Vx=inp(4);
rYaw=inp(5);

Fyr=(Myr-Myrprev)./Ts;

Ay=(1/m)*(-Fyf.*cos(delta)+Fyr+m*rYaw.*Vx);  
