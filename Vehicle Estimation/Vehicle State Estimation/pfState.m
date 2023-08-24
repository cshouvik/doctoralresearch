function state = pfState(Ts,particles,inp,const)
g = const(1);
Iz = const(2);
lf = const(3);
lr = const(4);
m = const(5);
l = lf + lr;

Ay = inp(1);
phi = inp(2);
Fyf = inp(3);
Fyr = inp(4);
delta = inp(5);
Vx = inp(6);
YawAcc = inp(7);

%%Previous time step estimated inputs
YawR = inp(8);
BetaR = inp(9);

MyfP = particles(1);
MyrP = particles(2);
VyP = particles(3);
rYawP = particles(4);
BetaP = particles(5);

% Myf State
alpha_2 = lf.*m.*Vx./l;
Myr = MyrP+Ts*(alpha_2.*BetaR+alpha_2.*YawR-(Iz./l).*YawAcc);

alpha_1 = lr.*m.*Vx./(l.*cos(delta));
Myf = MyfP+Ts*(alpha_1.*BetaR+alpha_1.*YawR+alpha_1.*(Iz./(lr.*m.*Vx)).*YawAcc);

Vy = VyP+Ts*((((Ay-g*sin(phi)./Vx))./cos(phi))-Vx.*YawR);
aYaw = (1/Iz)*(lf*Fyf.*cos(delta)-lr.*Fyr);
rYaw = rYawP+Ts * aYaw;
Beta = BetaP+Ts * ((1./(m*Vx)).*(Fyf.*cos(delta)+Fyr+m*g*sin(-phi))-YawR);



state=[Myf';Myr';Vy';rYaw';Beta'];
