function Vy = VyState(Ts,Vyin,inp,const)
  
%% Description
% Ts=>Sampling Time
% inp=>[Ay phi Vx YawR]vector containing all inp variables
% const=> [] vector containing all constant terms
% xin=> prev state information as inp
%%
Ay=inp(1);
phi=inp(2);
Vx=inp(3);
YawR=inp(4);
g=const(1);
 
Vy=Vyin+Ts*((((Ay-g*sin(phi)./Vx))./cos(phi))-Vx.*YawR);