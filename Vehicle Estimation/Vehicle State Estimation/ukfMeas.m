function meas = ukfMeas(Ts,sgpts,inp,const)
m=const;
Myfprev=inp(1);
Myrprev=inp(2);
delta=inp(3);
Vx=inp(4);

Myf=sgpts(1,:);
Myr=sgpts(2,:);
Vy=sgpts(3,:);
rYaw=sgpts(4,:);
Beta=sgpts(5,:);



Fyf=(Myf-Myfprev)./Ts;
Fyr=(Myr-Myrprev)./Ts;
Vy=Vx.*Beta;
Vxm=Vy./Beta;
Ay=(1/m)*(-Fyf.*cos(delta)+Fyr+m*rYaw.*Vx);

meas=[Ay;Ay;Vxm;Ay;Vy];
%lmeas=length(measVector);
%meas=repmat(measVector,1,lmeas).*eye(lmeas);
  
