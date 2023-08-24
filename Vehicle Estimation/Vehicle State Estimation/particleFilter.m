function state = particleFilter(N, Ts, particles, fState, sConst, sInp, fMeas, mConst, mInp, Q, R, meas)

for i = 1:N
	x_particle_update(:,i) = feval(fState,N,Ts,particles(:,i),sInp,sConst)+Q;

	meas_update(:,i) = feval(fMeas,N,Ts,x_particle_update(:,1),inp,const);

	P_w(:,i) = (1./sqrt(2*pi*R)) .* exp(-(meas - meas_update(i)).^2./(2*R));
end 

% Normalize to form probability distribution

for i = 1:length(P_w(:,1))
	P_w_norm(i,:) = P_w(i,:)/sum(P_w(i,:));
end

for i = 1 : N
    x_particle(:,i) = x_particle_update(find(rand(length(particle(:,1))) <= cumsum(P_w(:,i)),1));
end

state = mean(x_Particle())


	


