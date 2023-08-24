% Sample time
Ts = 0.1;

% Model parameters
m = 1650;
Iz = 3234;
lf = 1.4;
lr = 1.65;

% Continuous-time model
Ac=[-(Cf+Cr)/(m*Vx), -1-(lf*Cf-lr*Cr)/(m*Vx^2);
    -(lf*Cf-lr*Cr)/Iz, -(lf^2*Cf+lr^2*Cr)/(Iz*Vx)];

Bc=[Cf/(m*Vx);(lf*Cf)/Iz];

Cc=eye(2);

Dc=[0];
%% % Generate discrete-time model
nx = size(Ac,1);
nu = size(Bc,2);
M = expm([[Ac Bc]*Ts; zeros(nu,nx+nu)]);
A = M(1:nx,1:nx);
B = M(1:nx,nx+1:nx+nu);
C = Cc;
D = Dc;

% Nominal conditions for discrete-time plant
X = x;
U = steer;
Y = C*x + D*steer;
DX = A*x+B*steer-x;