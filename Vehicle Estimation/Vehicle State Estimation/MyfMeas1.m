
function Ay = MyfMeas1(Ts,Myf,inp,const)

%% Description
% Ts=>Sampling Time
% inp=>vector containing all inp variables
% const=> vector containing all constant terms
% xin=> prev state information as inp
m=const;
Myfprev=inp(1);
Fyr=inp(2);
delta=inp(3);
Vx=inp(4);
rYaw=inp(5);

Fyf=(Myf-Myfprev)./Ts;

Ay=(1/m)*(-Fyf.*cos(delta)+Fyr+m*rYaw.*Vx);  
