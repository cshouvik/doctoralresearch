function [A,B,C,D,sys] = linearVehicleModel(params,Fy,alpha,Vx)
% LINEAR VEHICLE MODEL [AUTHOR: SHOUVIK CHAKRABORTY]
% This function generates a Linear Vehicle Model[Single Track Bicycle Model]
% Inputs:   params => [m, lf, lr, Iz] m-> Mass (kg), lf,lr-> distance between  
%                                     axles and CG (m), Iz = Yaw Moment of
%                                     inertia accross (Kg m^2)
%           steer  => delta (Steering angle)(rad)
%           Fy     => [FyFL, FyFR, FyRL, FyRR] Lateral Tire Fources (N)
%           LTslip => [AlphaFL, AlphaFR, AlphaRL, AlphaRR] Slip Angle (rad)
%           Vx     => Vx Longitudinal Velocity (m/s)
%           Vy     => Vy Lateral Velocity (m/s)
%           r      => Yaw Rate (rad/s)
%           s      => Slip Angle (rad)
%  Outputs: State matrices (A,B,C,D)
% 
% 
%         [-(Cf+Cr)/(m*Vx)         ((lr*Cr-lf*Cf)/(m*Vx))-Vx        0        ] [Vy]    [   Cf/m  ]
% dX/dt = [(lr*Cr-lf*Cf)/(Iz*Vx)  -(lr^2*Cr+lf^2*Cf)/(Iz*Vx)        0        ]*[r ] +  [lf*Cf/iz ]*delta, 
%         [     0                  ((lr*Cr-lf*Cf)/(m*Vx))-1   -(Cf+Cr)/(m*Vx)] [s ]    [Cf/(m*Vx)]
%           
%         [1    0     0] [Vy]
%     Y = [0    1     0]*[r ]
%         [0    0     1] [s ]

%% PARAMETER INITIALIZATION

% CONSTANTS
m = params(1);
lf = params(2);
lr = params(3);
Iz = params(4);

% DETERMINING CORNERING STIFFNESS 
Cf = [Fy(1)+Fy(2)]/[alpha(1)+alpha(2)];
Cr = [Fy(3)+Fy(4)]/[alpha(3)+alpha(4)];

%% STATE SPACE FORMULATION

% 'A' MATRIX FORMULATION

a11 = -(Cf+Cr)/(m*Vx);
a12 = ((lr*Cr-lf*Cf)/(m*Vx))-Vx;
a13 = 0;

a21 = (lr*Cr-lf*Cf)/(Iz*Vx);
a22 = -(lr^2*Cr+lf^2*Cf)/(Iz*Vx);
a23 = 0;

a31 = 0;
a32 = ((lr*Cr-lf*Cf)/(m*Vx))-1;
a33 = -(Cf+Cr)/(m*Vx);

A = [a11 a12 a13;
     a21 a22 a23;
     a31 a32 a33];
 
% 'B' MATRIX FORMULATION

b1 = Cf/m;
b2 = lf*Cf/Iz;
b3 = Cf/(m*Vx);

B = [b1;b2;b3];

% 'C' MATRIX FORMULATION

C = eye(3,3);

% 'D' MATRIX FORMULATION 

D = 0;

% STATE SPACE OUTPUT

sys = ss(A,B,C,D);
end

