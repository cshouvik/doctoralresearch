
function Ay = ekfVyMeas2(Ts,Vyin,inp,const)

%% Description
% Ts=>Sampling Time
% inp=>vector containing all inp variables
% const=> vector containing all constant terms
% xin=> prev state information as inp
VyPrev=inp(1);
Vx=inp(2);
YawR=inp(3);
Ay=((Vyin-VyPrev)/Ts)+Vx.*YawR;  