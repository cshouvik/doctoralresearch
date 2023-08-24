function Myf = ekfMyfState(Ts,Myfin,inp,const)
  
%% Description
% Ts=>Sampling Time
% inp=>[delta Vx Betar YawR YawAcc]vector containing all inp variables
% const=> [Iz m lr l] vector containing all constant terms
% xin=> prev state information as inp
%%
Iz=const(1);
m=const(2);
lr=const(3);
l=const(4);
delta=inp(1);
Vx=inp(2);
Betar=inp(3);
YawR=inp(4);
YawAcc=inp(5);

alpha_1=lr.*m.*Vx./(l.*cos(delta));

Myf=Myfin+Ts*(alpha_1.*Betar+alpha_1.*YawR+alpha_1.*(Iz./(lr.*m.*Vx)).*YawAcc);
