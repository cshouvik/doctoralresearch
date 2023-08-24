function Myr = ekfMyrState(Ts,Myrin,inp,const)
  
%% Description
% Ts=>Sampling Time
% inp=>[Vx Betar YawR YawAcc]vector containing all inp variables
% const=> [Iz m lf l] vector containing all constant terms
% xin=> prev state information as inp
%%
Iz=const(1);
m=const(2);
lf=const(3);
l=const(4);
Vx=inp(1);
Betar=inp(2);
YawR=inp(3);
YawAcc=inp(4);

alpha_2=lf.*m.*Vx./l;

Myr=Myrin+Ts*(alpha_2.*Betar+alpha_2.*YawR-(Iz./l).*YawAcc);
