
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
        
        innovation = meas - meas_update(:,i);
        
        w_k(:,i) =  w_k_prev(:,i) + normpdf(innovation,0,sqrt(R));            %(1./sqrt(2*pi*R)) .* exp(-(meas - meas_update(i)).^2./(2*R));
    end
    
    
    for i = 1:5
        wsum = sum(w_k(i,:));
        if wsum == 0
            k = 1;
        else
            k = wsum;
        end
         w_k_normalized(i,:) = w_k(i,:)./k;
    end
    
    % Resampling
    resample = zeros(5,10);
    for j = 1:5
         resample(j,:) = randsample(1:N,N,true,w_k_normalized(j,:));
         %The final estimate is the mean value or variance
         xresampled(j,:) = x_particle_update(j,resample(j,:));
    end
    
    for j=1:5
       pf_xEst(j,n) = w_k_normalized(j,:)*xresampled(j,:)';
    end
    
    w_k_prev = w_k;
    %     
%     for i = 1:5
%     x(i,n) = mean(x_Particle_resample(i,:));
%     end

end

toc