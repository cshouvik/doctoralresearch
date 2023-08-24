ymeas = [AY';AY';AY';AY';Vy'];
myPF = particleFilter(@pfGenState,@MeasurementLikelihoodFcn);

initialize(myPF,1000,[Myf(1);Myr(1);Vy(1);YawR(1);Beta(1)],eye(5));

myPF.StateEstimationMethod = 'mean';
myPF.ResamplingMethod = 'systematic';

xEst = zeros(size(xTrue));
for k=1:size(xTrue,1)
    xEst(k,:) = correct(myPF,yMeas(:,k));
    predict(myPF);
end

figure(1)
plot(xTrue(:,1),xTrue(:,2),'x',xEst(:,1),xEst(:,2),'ro')
legend('True','Estimated')