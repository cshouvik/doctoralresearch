%% State Space Modelling
clc
T_final=Time(length(Time));
T_sample=Time(2);
Cf=mean(Cf_fit);
Cr=mean(Cr_fit);
Vx=mean(Vx);
lf=mean(lf);
lr=mean(lr);
m=mean(m);
Iz=mean(Iz);

A=[-(Cf+Cr)/(m*Vx), -1-(lf*Cf-lr*Cr)/(m*Vx^2);
    -(lf*Cf-lr*Cr)/Iz, -(lf^2*Cf+lr^2*Cr)/(Iz*Vx)];

B=[Cf/(m*Vx) 0;(lf*Cf)/Iz 1/Iz];

C=eye(2);

D=[0];

sys=ss(A,B,C,D);

%plot(step(sys,T_final))
% legend('Side Slip','Yaw Rate')