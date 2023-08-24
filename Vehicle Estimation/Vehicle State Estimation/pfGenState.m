function particles = pfGenState(Ts,inp,const,particles)
g=const(1);
Iz=const(2);
lf=const(3);
lr=const(4);
m=const(5);
l=lf+lr;

Ay=inp(1);
phi=inp(2);
Fyf=inp(3);
Fyr=inp(4);
delta=inp(5);
Vx=inp(6);
YawAcc=inp(7);

%%Previous time step estimated inputs
YawR=inp(8);
BetaR=inp(9);

alpha_1=lr.*m.*Vx./(l.*cos(delta));
alpha_2=lf.*m.*Vx./l;

tDx_Myf = Ts*(alpha_1.*BetaR+alpha_1.*YawR+alpha_1.*(Iz./(lr.*m.*Vx)).*YawAcc);
tDx_Myr = Ts*(alpha_2.*BetaR+alpha_2.*YawR-(Iz./l).*YawAcc);
tDx_Vy = Ts*((((Ay-g*sin(phi)./Vx))./cos(phi))-Vx.*YawR);
tDx_rYaw = (1/Iz)*(lf*Fyf.*cos(delta)-lr.*Fyr);
tDx_beta = Ts*((1./(m*Vx)).*(Fyf.*cos(delta)+Fyr+m*g*sin(-phi))-YawR);


[numberOfStates, numberOfParticles] = size(particles);

dt = Ts; % [s] Sample time
for kk=1:numberOfParticles
    particles(:,kk) = particles(:,kk) + vdpStateFcnContinuous(particles(:,kk))*dt;
end

% Add Gaussian noise with variance 0.025 on each state variable
processNoise = 0.025*eye(numberOfStates);
particles = particles + processNoise * randn(size(particles));
end

function dxdt = vdpStateFcnContinuous(x)
%vdpStateFcnContinuous Evaluate the van der Pol ODEs for mu = 1
dxdt = [x(1) + tDx_Myf; x(1) + tDx_Myr; x(1) + tDx_Vy; x(1) + tDx_rYaw; x(1) + tDx_beta];
end





Myf = Myfin + tDx_Myf;
Myr = Myrin + tDx_Myr;
Vy = Vyin + tDx_Vy;
rYaw = rYawin + tDx_rYaw;
Beta = Betain + tDx_beta;

state=[Myf;Myr;Vy;rYaw;Beta];
