function [A, B, C, E] = vehicleDeviationModel(params,Fy,alpha,Vx)
% LINEAR VEHICLE MODEL [AUTHOR: SHOUVIK CHAKRABORTY]
% This function generates a Linear Vehicle Model[Single Track Bicycle Model]
% Inputs:   params => [m, lf, lr, Iz] m-> Mass (kg), lf,lr-> distance between  
%                                     axles and CG (m), Iz = Yaw Moment of
%                                     inertia accross (Kg m^2)
%           steer  => delta (Steering angle)(rad)
%           Fy     => [FyFL, FyFR, FyRL, FyRR] Lateral Tire Fources (N)
%           LTslip => [AlphaFL, AlphaFR, AlphaRL, AlphaRR] Slip Angle (rad)
%           u      => Vx Longitudinal Velocity (m/s)
%           v      => Vy Lateral Velocity (m/s)
%           r      => Yaw Rate (rad/s)
%           s      => Slip Angle (rad)
%  Outputs: State matrices (A,B,C,D)
% 
% 
%         [     0                       1                      0                       0                    0]   [e1 ]   [              0               ]          [   Cf/m  ]        
%         [     0                  -(Cf+Cr)/(m*Vx)         (Cf+Cr)/m       (1/(m*Vx))*(-Cf*lf+Cr*lr)        0]   [de1]   [  (1/(m*Vx))*(-Cf*lf+Cr*lr)   ]          [lf*Cf/iz ] 
% dE/dt = [     0                       0                     0                        1                    0] * [e2 ] + [              0               ] * rdes + [Cf/(m*Vx)]*delta,
%         [     0               (lr*Cr-lf*Cf)/(Iz*Vx) -(lr*Cr-lf*Cf)/Iz  (1/(Iz*Vx))*(-Cf*lf^2-Cr*lr^2)     0]   [de2]   [(1/(Iz*Vx))*(-Cf*lf^2-Cr*lr^2)]          [lf*Cf/iz ] 
%         [-(Cf+Cr)/(m*Vx^2)            0               (Cf+Cr)/(m*Vx)     (1/(m*Vx))*(lr*Cr-lf*Cf)-1       0]   [e3 ]   [  (1/(m*Vx))*(lr*Cr-lf*Cf)-1  ]          [Cf/(m*Vx)]
%           
%         [1    0     0     0       0] [e1 ]
%         [0    1     0     0       0]*[de1]
%     Y = [0    0     1     0       0] [e2 ]
%         [0    0     0     1       0] [de2]
%         [0    0     0     0       1]*[e3 ]

%% PARAMETER INITIALIZATION

% CONSTANTS
m = params(1);
lf = params(2);
lr = params(3);
Iz = params(4);

% DETERMINING CORNERING STIFFNESS 
Cf = (Fy(1)+Fy(2))/(alpha(1)+alpha(2));
Cr = (Fy(3)+Fy(4))/(alpha(3)+alpha(4));


%% STATE SPACE FORMULATION

% 'A' MATRIX FORMULATION

a1 = -(Cf+Cr)/(m*Vx);
a2 = (Cf+Cr)/m;             %((lr*Cr-lf*Cf)/(m*Vx))-Vx;
a3 = (1/(m*Vx))*(-Cf*lf+Cr*lr);


b1 = (lr*Cr-lf*Cf)/(Iz*Vx);
b2 = -(lr*Cr-lf*Cf)/Iz;
b3 = (1/(Iz*Vx))*(-Cf*lf^2-Cr*lr^2);

c1 = -(Cf+Cr)/(m*Vx^2);
c2 = (Cf+Cr)/(m*Vx);
c3 = (1/(m*Vx))*(lr*Cr-lf*Cf)-1;

A =[0  1  0  0 0; 
    0 a1 a2 a3 0; 
    0  0  0  1 0; 
    0 b1 b2 b3 0; 
    c1 0 c2 c3 0];
 
% 'B' MATRIX FORMULATION

b1 = Cf/m;
b2 = lf*Cf/Iz;
b3 = Cf/(m*Vx);

B = [0;b1;0;b2;b3];

% 'C' MATRIX FORMULATION

C = eye(5,5);

% 'D' MATRIX FORMULATION 

E = [0;a3;0;b3;c3];
end

