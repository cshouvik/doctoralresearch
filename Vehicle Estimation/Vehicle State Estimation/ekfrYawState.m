function rYaw = ekfrYawState(Ts,rYawin,inp,const)
  
%% Description
% Ts=>Sampling Time
% inp=>[Fyf Fyr delta]vector containing all inp variables
% const=> [Iz lf lr] vector containing all constant terms
% xin=> prev state information as inp
%%
Iz=const(1);
lf=const(2);
lr=const(3);
Fyf=inp(1);
Fyr=inp(2);
delta=inp(3);

aYaw=(1/Iz)*(lf*Fyf.*cos(delta)-lr.*Fyr);
 
rYaw=rYawin+Ts*aYaw;