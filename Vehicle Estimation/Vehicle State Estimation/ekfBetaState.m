function Beta = ekfBetaState(Ts,Betain,inp,const)
  
%% Description
% Ts=>Sampling Time
% inp=>[delta phi YawR Vx Fyf Fyr]vector containing all inp variables
% const=> [g m] vector containing all constant terms
% xin=> prev state information as inp
%%
delta=inp(1);
phi=inp(2);
YawR=inp(3);
Vx=inp(4);
Fyf=inp(5);
Fyr=inp(6);
g=const(1);
m=const(2);
 
Beta=Betain+Ts*((1./(m*Vx)).*(Fyf.*cos(delta)+Fyr+m*g*sin(-phi))-YawR);