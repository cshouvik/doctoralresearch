clc
clear all
mul = 1;
nMul = linspace(0.1,10,100);
index = 0
uiopen
for i=1:1:100
    scale = nMul(i);
    index = index+1;
    run UKF_Complete_dataGen.m;
    run EKF_Complete_dataGen.m;
    run IEKF_Complete_dataGen.m;
    run AIEKF_Complete_dataGen.m;
    run PF_Complete_dataGen.m;
    % Save Data
    run monteCarlo_dataStore_complete.m
    index
end
uiopen
for i=1:1:100
    scale = nMul(i);
    index = index+1;
    run UKF_Complete_dataGen.m;
    run EKF_Complete_dataGen.m;
    run IEKF_Complete_dataGen.m;
    run AIEKF_Complete_dataGen.m;
    run PF_Complete_dataGen.m;
    % Save Data
    run monteCarlo_dataStore_complete.m
    index
end
uiopen
for i=1:1:100
    scale = nMul(i);
    index = index+1;
    run UKF_Complete_dataGen.m;
    run EKF_Complete_dataGen.m;
    run IEKF_Complete_dataGen.m;
    run AIEKF_Complete_dataGen.m;
    run PF_Complete_dataGen.m;
    % Save Data
    run monteCarlo_dataStore_complete.m
    index
end
uiopen
for i=1:1:100
    scale = nMul(i);
    index = index+1;
    run UKF_Complete_dataGen.m;
    run EKF_Complete_dataGen.m;
    run IEKF_Complete_dataGen.m;
    run AIEKF_Complete_dataGen.m;
    run PF_Complete_dataGen.m;
    % Save Data
    run monteCarlo_dataStore_complete.m
    index
end
disp('complete')
