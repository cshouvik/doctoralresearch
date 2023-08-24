function state = ekfState2(Ts,states,inp,const)
g=const(1);
Iz=const(2);
lf=const(3);
lr=const(4);
m=const(5);
l=lf+lr;

Ay=inp(1);
phi=inp(2);
Fyf=inp(3);
Fyr=inp(4);
delta=inp(5);
Vx=inp(6);
YawAcc=inp(7);

%%Previous time step estimated inputs
YawR=inp(8);
BetaR=inp(9);

Myfin=states(1,:);
Myrin=states(2,:);
Vyin=states(3,:);
rYawin=states(4,:);
Betain=states(5,:);




Vy=Vyin+Ts*((((Ay-g*sin(phi)./Vx))./cos(phi))-Vx.*YawR);
aYaw=(1/Iz)*(lf*Fyf.*cos(delta)-lr.*Fyr);
rYaw=rYawin+Ts*aYaw;
Beta=Betain+Ts*((1./(m*Vx)).*(Fyf.*cos(delta)+Fyr+m*g*sin(-phi))-YawR);

alpha_2=lf.*m.*Vx./l;
Myr=Myrin+Ts*(alpha_2.*BetaR+alpha_2.*YawR-(Iz./l).*YawAcc);

alpha_1=lr.*m.*Vx./(l.*cos(delta));
Myf=Myfin+Ts*(alpha_1.*BetaR+alpha_1.*YawR+alpha_1.*(Iz./(lr.*m.*Vx)).*YawAcc);

state=[Myf;Myr;Vy;rYaw;Beta];
