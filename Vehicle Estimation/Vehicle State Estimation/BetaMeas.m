
function Vy = BetaMeas(Ts,Betain,inp,const)

%% Description
% Ts=>Sampling Time
% inp=>vector containing all inp variables
% const=> vector containing all constant terms
% xin=> prev state information as inp
Vx=inp(1);
Vy=Vx.*Betain;  
