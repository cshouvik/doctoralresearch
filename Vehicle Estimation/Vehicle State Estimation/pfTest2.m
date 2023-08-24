pf_xEst(:,1) = [Myf(1); Myr(1); Vy(1); YawR(1); Beta(1)]; 	% initial actual state
Q = ones(5,1) * 0.00001; 								% Process noise variance
R = ones(5,1) * 0.01; 								% Measurement noise variance
T = Time(length(Time)); 							% Number of iterations.
N = 10; 											% Number of particles the system generates.
P = ones(5,1) * 0.1;									% Initial Covarience
Ts = Time(2);
x_Particle = [];											% Initial particles vector
w_k_prev = repmat(1/N,5,N);                  %Initialize weight
tic

for n = 2:length(Time)
    
    for i = 1:N
        x_Particle(:,i) = pf_xEst(:,n-1) + sqrt(P) .* randn(5,1);		% Generate Particles Randomly
    end
    sInp = [AY(n-1);phi(n-1);Fyf(n-1);Fyr(n-1);delta(n-1);Vx(n-1);YawAcc(n-1);YawR(n-1);BetaR(n-1)];
    sConst = [g;Iz;lf;lr;m];
    mInp = [Myf(n-1); Myr(n-1);delta(n);Vx(n)];
    mConst = m;
    meas = [AY(n);AY(n);Vx(n);AY(n);Vy(n)]+R;
    x_particle_update = zeros(5,N);
    
    % Propagate particles
    for i = 1:N
        x_particle_update(:,i) = pfState(Ts,x_Particle(:,i),sInp,sConst) + sqrt(Q);
        meas_update(:,i) = pfMeas(Ts,x_particle_update(:,1),mInp,mConst);
        P_w(:,i) = (1./sqrt(2*pi*R)) .* exp(-(meas - meas_update(:,i)).^2./(2*R));
    end 

% Normalize to form probability distribution

    for i = 1:length(P_w(:,1))
        P_w_norm(i,:) = P_w(i,:)/sum(P_w(i,:));
    end

    for i = 1 : N
        x_particle(:,i) = x_particle_update(find(rand(length(x_Particle(:,1))) <= cumsum(P_w(:,i)),1));
    end

    state = mean(x_Particle());

    pf_xEst(:,n) = state;
end

toc