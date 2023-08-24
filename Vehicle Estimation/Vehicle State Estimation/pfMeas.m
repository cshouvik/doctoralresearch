function meas = pfMeas(Ts,stateparticles,inp,const)
m=const;
Myfprev=inp(1);
Myrprev=inp(2);
delta=inp(3);
Vx=inp(4);

Myf=stateparticles(1,:);
Myr=stateparticles(2,:);
Vy=stateparticles(3,:);
rYaw=stateparticles(4,:);
Beta=stateparticles(5,:);



Fyf=(Myf-Myfprev)./Ts;
Fyr=(Myr-Myrprev)./Ts;
Vy=Vx.*Beta;
Vxm=Vy./Beta;
Ay=(1/m)*(-Fyf.*cos(delta)+Fyr+m*rYaw.*Vx);

meas=[Ay';Ay';Vxm';Ay';Vy'];
  
