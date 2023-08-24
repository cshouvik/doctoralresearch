% Model Initialization 
A=[-(Cf+Cr)/(m*Vx), -1-(lf*Cf-lr*Cr)/(m*Vx^2);
    -(lf*Cf-lr*Cr)/Iz, -(lf^2*Cf+lr^2*Cr)/(Iz*Vx)];
B=[Cf/(m*Vx);(lf*Cf)/Iz];
C=eye(2);
D=[0];