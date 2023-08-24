function meas = ekfMeas2(Ts,state,inp,const)
m=const;
Myfprev=inp(1);
Myrprev=inp(2);
VyPrev=inp(5);
delta=inp(3);
Vx=inp(4);

Myf=state(1,:);
Myr=state(2,:);
Vy=state(3,:);
rYaw=state(4,:);
Beta=state(5,:);

Fyf=(Myf-Myfprev)./Ts;
Fyr=(Myr-Myrprev)./Ts;
dVy=(Vy-VyPrev)./Ts;

Vy=Vx.*Beta;
Ay2=dVy+Vx.*rYaw;
Ay=(1/m)*(-Fyf.*cos(delta)+Fyr+m*rYaw.*Vx);

meas=[Ay;Ay;Ay2;Ay;Vy];