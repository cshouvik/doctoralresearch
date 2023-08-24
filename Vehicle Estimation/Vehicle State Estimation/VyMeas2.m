
function Vx = VyMeas2(Ts,Vyin,inp,const)

%% Description
% Ts=>Sampling Time
% inp=>vector containing all inp variables
% const=> vector containing all constant terms
% xin=> prev state information as inp
Beta=inp(1);
Vx=Vyin./Beta;
Vx(isnan(Vx)) = 0;

