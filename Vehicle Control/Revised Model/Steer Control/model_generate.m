%% Form 1
l = lf + lr;
% a11 = (-Cf-Cr)/(m*Vx);
% a12 = -1 + ((Cr*lr - Cf*lf)/(m*Vx^2));
% a13 = 0;
% a14 = 0;
% 
% a21 = (Cr*lr - Cf*lf)/Iz;
% a22 = (-Cf*lf^2-Cr*lr^2)/(Iz*Vx);
% a23 = 0;
% a24 = 0;
% 
% a31 = (Cf.^2 + Cf*Cr)/(m*Vx);
% a32 = (1 - (Cr*lr - Cf*lf)/(m*Vx*2))*Cf;
% a33 = -lf^2*Cf/(Vx*Iz);
% a34 = lf*lr*Cf/(Vx*Iz);
% 
% a41 = (Cf*Cr + Cr^2)/(m*Vx);
% a42 = (1 - ((Cr*lr - Cf*lf)/(m*Vx^2)))*Cr;
% a43 = Cr*lf*lr/(Vx*Iz);
% a44 = -Cr*lr^2/(Vx*Iz);
% 
% b11 = Cf/(m*Vx);
% b12 = 0;
% b13 = 0;
% b14 = 0;
% 
% b21 = Cf*lf/Iz;
% b22 = 0;
% b23 = 1/Iz;
% b24 = 0;
% 
% b31 = -Cf^2/(m*Vx);
% b32 = Cf;
% b33 = -lf*Cf/(Vx*Iz);
% b34 = 1/l;
% 
% b41 = -Cf*Cr/(m*Vx);
% b42 = 0;
% b43 = Cr*lr/(Vx*Iz);
% b44 = -1/l;
% 
% A1 = [a11 a12 a13 a14;
%       a21 a22 a23 a24;
%       a31 a32 a33 a34;
%       a41 a42 a43 a44];
% 
% % B1 = [b11 b12 b13 b14;
% %       b21 b22 b23 b24;
% %       b31 b32 b33 b34;
% %       b41 b42 b43 b44];     % Integrated Control
% B1 = [b11 b12;
%       b21 b22;
%       b31 b32;
%       b41 b42];     % Steering Control
%   
% C1 = eye(4);
% D1 = zeros(4,2);

% sys = ss(A,B,C,D);
% A = sys.A;
% B = sys.B;
% C = sys.C;
% D = sys.D;

%% Form 2
l = lf + lr;
aa11 = -Cf/(m*Vx);
aa12 = -(1 + (1/(m*Vx))+(lf*Cf/(m*Vx^2)));
aa13 = 0;
aa14 = 1/(m*Vx);

aa21 = Cf*lf/Iz;
aa22 = -Cf*lf^2/(Iz*Vx);
aa23 = 0;
aa24 = -lr/Iz;

aa31 = (Cf.^2 + Cf*Cr)/(m*Vx);
aa32 = (1 - (Cr*lr - Cf*lf)/(m*Vx*2))*Cf;
aa33 = -lf^2*Cf/(Vx*Iz);
aa34 = lf*lr*Cf/(Vx*Iz);

aa41 = (Cf*Cr + Cr^2)/(m*Vx);
aa42 = (1 - (Cr*lr - Cf*lf)/(m*Vx^2))*Cr;
aa43 = Cr*lf*lr/(Vx*Iz);
aa44 = -Cr*lr^2/(Vx*Iz);

bb11 = Cf/(m*Vx);
bb12 = 0;
bb13 = 0;
bb14 = 0;

bb21 = Cf*lf/Iz;
bb22 = 0;
bb23 = 1/Iz;
bb24 = 0;

bb31 = -Cf^2/(m*Vx);
bb32 = Cf;
bb33 = -lf*Cf/(Vx*Iz);
bb34 = 1/l;

bb41 = -Cf*Cr/(m*Vx);
bb42 = 0;
bb43 = Cr*lr/(Vx*Iz);
bb44 = -1/l;

Aa = [aa11 aa12 aa13 aa14;
     aa21 aa22 aa23 aa24;
     aa31 aa32 aa33 aa34;
     aa41 aa42 aa43 aa44];

% Bb = [bb11 bb12 bb13 bb14;
%      bb21 bb22 bb23 bb24;
%      bb31 bb32 bb33 bb34;
%      bb41 bb42 bb43 bb44];          %Integrated Control
Bb = [bb11 bb12;
      bb21 bb22;
      bb31 bb32;
      bb41 bb42];          % Steering Control

Cc = [1 0 0 0; 0 1 0 0];
Dd = zeros(2);

% sys = ss(A,B,C,D);
% A = sys.A;
% B = sys.B;
% C = sys.C;
% D = sys.D;