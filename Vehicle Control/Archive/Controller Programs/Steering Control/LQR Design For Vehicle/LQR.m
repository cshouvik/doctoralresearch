function Klqr = LQR(Cf, Cr, Vx, w_slip, w_yawRate, w_errSlip, w_errYawRate, w_steer)

% Vehicle Configuration parameters
lf=	;
lr=	;
m=	;
Iz=	;

% Code for System Modelling(Continous Time)

A=[-(Cf+Cr)/(m*Vx), -1-(lf*Cf-lr*Cr)/(m*Vx^2);
    -(lf*Cf-lr*Cr)/Iz, -(lf^2*Cf+lr^2*Cr)/(Iz*Vx)];
B=[Cf/(m*Vx);(lf*Cf)/Iz];
C=eye(2);
D=[0];
sys=ss(A,B,C,D);

% Weights for Tuning
Q_lqi=diag([w_slip,w_yawRate,w_errSlip,w_errYawRate]);
R_lqi=[w_steer];

%LQI formulation for Continous Time State space Model.
Klqr=lqi(ss(A,B,C_lqi,D),Q_lqi,R_lqi);