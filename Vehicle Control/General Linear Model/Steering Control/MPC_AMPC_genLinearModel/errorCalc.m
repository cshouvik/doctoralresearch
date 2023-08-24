error.yawRate = goodnessOfFit(out.yawRateOut(:,1),out.yawRateOut(:,2),'nmse');
error.beta =  goodnessOfFit(out.betaOut(:,1),out.betaOut(:,2),'mse');
errorTable =[error.yawRate, error.beta]
