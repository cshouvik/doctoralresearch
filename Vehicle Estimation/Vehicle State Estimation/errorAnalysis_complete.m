clc
clear all
uiopen
mul = 1;
 run UKF_Complete_dataGen.m;
 run EKF_Complete_dataGen.m;
 run IEKF_Complete_dataGen.m;
 run AIEKF_Complete_dataGen.m;
 run PF_Complete_dataGen.m;

% Additional variables
true.steer = delta;
true.Vx = Vx;
true.YawAcc  = YawAcc;
true.BetaR = BetaR;
true.Ay = [0; diff(VY)]+Vx.*YawR;
true.time = Time;
true.Beta=Beta;
true.YawR=YawR;
true.Fyf = Fyf;
true.Fyr = Fyr;
true.Vy=VY;

save('moduleEstimation_DataSetName','true', 'modules','ues');
clear all
load('moduleEstimation_DataSetName');
T = true.time(1:(length(true.time)-20));
%% -------------------- Variable Assignment True -------------------------
Fyf = true.Fyf(1:length(T));
Fyr = true.Fyr(1:length(T));
Vy = true.Vy(1:length(T));
Ay = true.Ay(1:length(T));
YawR = true.YawR(1:length(T));
YawAcc = true.YawAcc(1:length(T));
Beta = true.Beta(1:length(T));
BetaR = true.BetaR(1:length(T));
%% ----------- Variable Assignment and Error Analysis EKF ----------------
ekf_mod_Fyf = modules.ekf.Fyf(1:length(T));
ekf_noKF_Fyf = ues.noKF.ekf.FyF(1:length(T))';
ekf_KF_Fyf = ues.KF.ekf.Fyf(1:length(T))';
     
ekf_mod_Fyr = modules.ekf.Fyr(1:length(T));
ekf_noKF_Fyr = ues.noKF.ekf.FyR(1:length(T))';
ekf_KF_Fyr = ues.KF.ekf.Fyr(1:length(T))';
     
ekf_mod_Vy = modules.ekf.Vy(1:length(T))';
ekf_noKF_Vy = ues.noKF.ekf.Vy(1:length(T))';
ekf_KF_Vy = ues.KF.ekf.Vy(1:length(T))';

ekf_mod_Ay = modules.ekf.Ay(1:length(T)); 
ekf_noKF_Ay = ues.noKF.ekf.Ay(1:length(T));
ekf_KF_Ay = ues.KF.ekf.Ay(1:length(T))';

ekf_mod_YawR = modules.ekf.YawR(1:length(T))';
ekf_noKF_YawR = ues.noKF.ekf.YawR(1:length(T))';
ekf_KF_YawR = ues.KF.ekf.YawR(1:length(T))';
     
ekf_mod_YawAcc = modules.ekf.YawAcc(1:length(T));
ekf_noKF_YawAcc = ues.noKF.ekf.YawAcc(1:length(T));
ekf_KF_YawAcc = ues.KF.ekf.YawAcc(1:length(T))';
     
ekf_mod_Beta = modules.ekf.Beta(1:length(T))';
ekf_noKF_Beta = ues.noKF.ekf.Beta(1:length(T))';
ekf_KF_Beta = ues.KF.ekf.Beta(1:length(T))';
     
ekf_mod_BetaR = modules.ekf.Betar(1:length(T));
ekf_noKF_BetaR = ues.noKF.ekf.Betar(1:length(T));
ekf_KF_BetaR = ues.KF.ekf.BetaR(1:length(T))';

nmse.ekf.mod.Fyf = goodnessOfFit( Fyf,ekf_mod_Fyf,'nmse');
nmse.ekf.mod.Fyr = goodnessOfFit( Fyr,ekf_mod_Fyr,'nmse');
nmse.ekf.mod.Vy = goodnessOfFit( Vy,ekf_mod_Vy,'nmse');
nmse.ekf.mod.Ay = goodnessOfFit( Ay,ekf_mod_Ay,'nmse');
nmse.ekf.mod.YawR = goodnessOfFit( YawR,ekf_mod_YawR,'nmse');
nmse.ekf.mod.YawAcc = goodnessOfFit( YawAcc,ekf_mod_YawAcc,'nmse');
nmse.ekf.mod.Beta = goodnessOfFit( Beta,ekf_mod_Beta,'nmse');
nmse.ekf.mod.BetaR = goodnessOfFit( BetaR,ekf_mod_BetaR,'nmse');

nmse.ekf.noKF.Fyf = goodnessOfFit( Fyf,ekf_noKF_Fyf,'nmse');
nmse.ekf.noKF.Fyr = goodnessOfFit( Fyr,ekf_noKF_Fyr,'nmse');
nmse.ekf.noKF.Vy = goodnessOfFit( Vy,ekf_noKF_Vy,'nmse');
nmse.ekf.noKF.Ay = goodnessOfFit( Ay,ekf_noKF_Ay,'nmse');
nmse.ekf.noKF.YawR = goodnessOfFit( YawR,ekf_noKF_YawR,'nmse');
nmse.ekf.noKF.YawAcc = goodnessOfFit( YawAcc,ekf_noKF_YawAcc,'nmse');
nmse.ekf.noKF.Beta = goodnessOfFit( Beta,ekf_noKF_Beta,'nmse');
nmse.ekf.noKF.BetaR = goodnessOfFit( BetaR,ekf_noKF_BetaR,'nmse');

nmse.ekf.KF.Fyf = goodnessOfFit( Fyf,ekf_KF_Fyf,'nmse');
nmse.ekf.KF.Fyr = goodnessOfFit( Fyr,ekf_KF_Fyr,'nmse');
nmse.ekf.KF.Vy = goodnessOfFit( Vy,ekf_KF_Vy,'nmse');
nmse.ekf.KF.Ay = goodnessOfFit( Ay,ekf_KF_Ay,'nmse');
nmse.ekf.KF.YawR = goodnessOfFit( YawR,ekf_KF_YawR,'nmse');
nmse.ekf.KF.YawAcc = goodnessOfFit( YawAcc,ekf_KF_YawAcc,'nmse');
nmse.ekf.KF.Beta = goodnessOfFit( Beta,ekf_KF_Beta,'nmse');
nmse.ekf.KF.BetaR = goodnessOfFit( BetaR,ekf_KF_BetaR,'nmse');

nmse.ekf.cumulative = ...
    [nmse.ekf.mod.Fyf, nmse.ekf.noKF.Fyf, nmse.ekf.KF.Fyf,...
     nmse.ekf.mod.Fyr, nmse.ekf.noKF.Fyr, nmse.ekf.KF.Fyr,...
     nmse.ekf.mod.Vy, nmse.ekf.noKF.Vy, nmse.ekf.KF.Vy,...
     nmse.ekf.mod.Ay, nmse.ekf.noKF.Ay, nmse.ekf.KF.Ay,...
     nmse.ekf.mod.YawR, nmse.ekf.noKF.YawR, nmse.ekf.KF.YawR,...
     nmse.ekf.mod.YawAcc, nmse.ekf.noKF.YawAcc, nmse.ekf.KF.YawAcc,...
     nmse.ekf.mod.Beta, nmse.ekf.noKF.Beta, nmse.ekf.KF.Beta,...
     nmse.ekf.mod.BetaR, nmse.ekf.noKF.BetaR, nmse.ekf.KF.BetaR];
%% ----------- Variable Assignment and Error Analysis IEKF ---------------
iekf_mod_Fyf = modules.iekf.Fyf(1:length(T));
iekf_noKF_Fyf = ues.noKF.iekf.FyF(1:length(T))';
iekf_KF_Fyf = ues.KF.iekf.Fyf(1:length(T))';
     
iekf_mod_Fyr = modules.iekf.Fyr(1:length(T));
iekf_noKF_Fyr = ues.noKF.iekf.FyR(1:length(T))';
iekf_KF_Fyr = ues.KF.iekf.Fyr(1:length(T))';
     
iekf_mod_Vy = modules.iekf.Vy(1:length(T))';
iekf_noKF_Vy = ues.noKF.iekf.Vy(1:length(T))';
iekf_KF_Vy = ues.KF.iekf.Vy(1:length(T))';

iekf_mod_Ay = modules.iekf.Ay(1:length(T)); 
iekf_noKF_Ay = ues.noKF.iekf.Ay(1:length(T));
iekf_KF_Ay = ues.KF.iekf.Ay(1:length(T))';

iekf_mod_YawR = modules.iekf.YawR(1:length(T))';
iekf_noKF_YawR = ues.noKF.iekf.YawR(1:length(T))';
iekf_KF_YawR = ues.KF.iekf.YawR(1:length(T))';
     
iekf_mod_YawAcc = modules.iekf.YawAcc(1:length(T));
iekf_noKF_YawAcc = ues.noKF.iekf.YawAcc(1:length(T));
iekf_KF_YawAcc = ues.KF.iekf.YawAcc(1:length(T))';
     
iekf_mod_Beta = modules.iekf.Beta(1:length(T))';
iekf_noKF_Beta = ues.noKF.iekf.Beta(1:length(T))';
iekf_KF_Beta = ues.KF.iekf.Beta(1:length(T))';
     
iekf_mod_BetaR = modules.iekf.Betar(1:length(T));
iekf_noKF_BetaR = ues.noKF.iekf.Betar(1:length(T));
iekf_KF_BetaR = ues.KF.iekf.BetaR(1:length(T))';

nmse.iekf.mod.Fyf = goodnessOfFit( Fyf,iekf_mod_Fyf,'nmse');
nmse.iekf.mod.Fyr = goodnessOfFit( Fyr,iekf_mod_Fyr,'nmse');
nmse.iekf.mod.Vy = goodnessOfFit( Vy,iekf_mod_Vy,'nmse');
nmse.iekf.mod.Ay = goodnessOfFit( Ay,iekf_mod_Ay,'nmse');
nmse.iekf.mod.YawR = goodnessOfFit( YawR,iekf_mod_YawR,'nmse');
nmse.iekf.mod.YawAcc = goodnessOfFit( YawAcc,iekf_mod_YawAcc,'nmse');
nmse.iekf.mod.Beta = goodnessOfFit( Beta,iekf_mod_Beta,'nmse');
nmse.iekf.mod.BetaR = goodnessOfFit( BetaR,iekf_mod_BetaR,'nmse');

nmse.iekf.noKF.Fyf = goodnessOfFit( Fyf,iekf_noKF_Fyf,'nmse');
nmse.iekf.noKF.Fyr = goodnessOfFit( Fyr,iekf_noKF_Fyr,'nmse');
nmse.iekf.noKF.Vy = goodnessOfFit( Vy,iekf_noKF_Vy,'nmse');
nmse.iekf.noKF.Ay = goodnessOfFit( Ay,iekf_noKF_Ay,'nmse');
nmse.iekf.noKF.YawR = goodnessOfFit( YawR,iekf_noKF_YawR,'nmse');
nmse.iekf.noKF.YawAcc = goodnessOfFit( YawAcc,iekf_noKF_YawAcc,'nmse');
nmse.iekf.noKF.Beta = goodnessOfFit( Beta,iekf_noKF_Beta,'nmse');
nmse.iekf.noKF.BetaR = goodnessOfFit( BetaR,iekf_noKF_BetaR,'nmse');

nmse.iekf.KF.Fyf = goodnessOfFit( Fyf,iekf_KF_Fyf,'nmse');
nmse.iekf.KF.Fyr = goodnessOfFit( Fyr,iekf_KF_Fyr,'nmse');
nmse.iekf.KF.Vy = goodnessOfFit( Vy,iekf_KF_Vy,'nmse');
nmse.iekf.KF.Ay = goodnessOfFit( Ay,iekf_KF_Ay,'nmse');
nmse.iekf.KF.YawR = goodnessOfFit( YawR,iekf_KF_YawR,'nmse');
nmse.iekf.KF.YawAcc = goodnessOfFit( YawAcc,iekf_KF_YawAcc,'nmse');
nmse.iekf.KF.Beta = goodnessOfFit( Beta,iekf_KF_Beta,'nmse');
nmse.iekf.KF.BetaR = goodnessOfFit( BetaR,iekf_KF_BetaR,'nmse');

nmse.iekf.cumulative = ...
    [nmse.iekf.mod.Fyf, nmse.iekf.noKF.Fyf, nmse.iekf.KF.Fyf,...
     nmse.iekf.mod.Fyr, nmse.iekf.noKF.Fyr, nmse.iekf.KF.Fyr,...
     nmse.iekf.mod.Vy, nmse.iekf.noKF.Vy, nmse.iekf.KF.Vy,...
     nmse.iekf.mod.Ay, nmse.iekf.noKF.Ay, nmse.iekf.KF.Ay,...
     nmse.iekf.mod.YawR, nmse.iekf.noKF.YawR, nmse.iekf.KF.YawR,...
     nmse.iekf.mod.YawAcc, nmse.iekf.noKF.YawAcc, nmse.iekf.KF.YawAcc,...
     nmse.iekf.mod.Beta, nmse.iekf.noKF.Beta, nmse.iekf.KF.Beta,...
     nmse.iekf.mod.BetaR, nmse.iekf.noKF.BetaR, nmse.iekf.KF.BetaR];
%% ---------- Variable Assignment and Error Analysis AIEKF ---------------
aiekf_mod_Fyf = modules.aiekf.Fyf(1:length(T));
aiekf_noKF_Fyf = ues.noKF.aiekf.FyF(1:length(T))';
aiekf_KF_Fyf = ues.KF.aiekf.Fyf(1:length(T))';
     
aiekf_mod_Fyr = modules.aiekf.Fyr(1:length(T));
aiekf_noKF_Fyr = ues.noKF.aiekf.FyR(1:length(T))';
aiekf_KF_Fyr = ues.KF.aiekf.Fyr(1:length(T))';
     
aiekf_mod_Vy = modules.aiekf.Vy(1:length(T))';
aiekf_noKF_Vy = ues.noKF.aiekf.Vy(1:length(T))';
aiekf_KF_Vy = ues.KF.aiekf.Vy(1:length(T))';

aiekf_mod_Ay = modules.aiekf.Ay(1:length(T)); 
aiekf_noKF_Ay = ues.noKF.aiekf.Ay(1:length(T));
aiekf_KF_Ay = ues.KF.aiekf.Ay(1:length(T))';

aiekf_mod_YawR = modules.aiekf.YawR(1:length(T))';
aiekf_noKF_YawR = ues.noKF.aiekf.YawR(1:length(T))';
aiekf_KF_YawR = ues.KF.aiekf.YawR(1:length(T))';
     
aiekf_mod_YawAcc = modules.aiekf.YawAcc(1:length(T));
aiekf_noKF_YawAcc = ues.noKF.aiekf.YawAcc(1:length(T));
aiekf_KF_YawAcc = ues.KF.aiekf.YawAcc(1:length(T))';
     
aiekf_mod_Beta = modules.aiekf.Beta(1:length(T))';
aiekf_noKF_Beta = ues.noKF.aiekf.Beta(1:length(T))';
aiekf_KF_Beta = ues.KF.aiekf.Beta(1:length(T))';
     
aiekf_mod_BetaR = modules.aiekf.Betar(1:length(T));
aiekf_noKF_BetaR = ues.noKF.aiekf.Betar(1:length(T));
aiekf_KF_BetaR = ues.KF.aiekf.BetaR(1:length(T))';

nmse.aiekf.mod.Fyf = goodnessOfFit( Fyf,aiekf_mod_Fyf,'nmse');
nmse.aiekf.mod.Fyr = goodnessOfFit( Fyr,aiekf_mod_Fyr,'nmse');
nmse.aiekf.mod.Vy = goodnessOfFit( Vy,aiekf_mod_Vy,'nmse');
nmse.aiekf.mod.Ay = goodnessOfFit( Ay,aiekf_mod_Ay,'nmse');
nmse.aiekf.mod.YawR = goodnessOfFit( YawR,aiekf_mod_YawR,'nmse');
nmse.aiekf.mod.YawAcc = goodnessOfFit( YawAcc,aiekf_mod_YawAcc,'nmse');
nmse.aiekf.mod.Beta = goodnessOfFit( Beta,aiekf_mod_Beta,'nmse');
nmse.aiekf.mod.BetaR = goodnessOfFit( BetaR,aiekf_mod_BetaR,'nmse');

nmse.aiekf.noKF.Fyf = goodnessOfFit( Fyf,aiekf_noKF_Fyf,'nmse');
nmse.aiekf.noKF.Fyr = goodnessOfFit( Fyr,aiekf_noKF_Fyr,'nmse');
nmse.aiekf.noKF.Vy = goodnessOfFit( Vy,aiekf_noKF_Vy,'nmse');
nmse.aiekf.noKF.Ay = goodnessOfFit( Ay,aiekf_noKF_Ay,'nmse');
nmse.aiekf.noKF.YawR = goodnessOfFit( YawR,aiekf_noKF_YawR,'nmse');
nmse.aiekf.noKF.YawAcc = goodnessOfFit( YawAcc,aiekf_noKF_YawAcc,'nmse');
nmse.aiekf.noKF.Beta = goodnessOfFit( Beta,aiekf_noKF_Beta,'nmse');
nmse.aiekf.noKF.BetaR = goodnessOfFit( BetaR,aiekf_noKF_BetaR,'nmse');

nmse.aiekf.KF.Fyf = goodnessOfFit( Fyf,aiekf_KF_Fyf,'nmse');
nmse.aiekf.KF.Fyr = goodnessOfFit( Fyr,aiekf_KF_Fyr,'nmse');
nmse.aiekf.KF.Vy = goodnessOfFit( Vy,aiekf_KF_Vy,'nmse');
nmse.aiekf.KF.Ay = goodnessOfFit( Ay,aiekf_KF_Ay,'nmse');
nmse.aiekf.KF.YawR = goodnessOfFit( YawR,aiekf_KF_YawR,'nmse');
nmse.aiekf.KF.YawAcc = goodnessOfFit( YawAcc,aiekf_KF_YawAcc,'nmse');
nmse.aiekf.KF.Beta = goodnessOfFit( Beta,aiekf_KF_Beta,'nmse');
nmse.aiekf.KF.BetaR = goodnessOfFit( BetaR,aiekf_KF_BetaR,'nmse');

nmse.aiekf.cumulative = ...
    [nmse.aiekf.mod.Fyf, nmse.aiekf.noKF.Fyf, nmse.aiekf.KF.Fyf,...
     nmse.aiekf.mod.Fyr, nmse.aiekf.noKF.Fyr, nmse.aiekf.KF.Fyr,...
     nmse.aiekf.mod.Vy, nmse.aiekf.noKF.Vy, nmse.aiekf.KF.Vy,...
     nmse.aiekf.mod.Ay, nmse.aiekf.noKF.Ay, nmse.aiekf.KF.Ay,...
     nmse.aiekf.mod.YawR, nmse.aiekf.noKF.YawR, nmse.aiekf.KF.YawR,...
     nmse.aiekf.mod.YawAcc, nmse.aiekf.noKF.YawAcc, nmse.aiekf.KF.YawAcc,...
     nmse.aiekf.mod.Beta, nmse.aiekf.noKF.Beta, nmse.aiekf.KF.Beta,...
     nmse.aiekf.mod.BetaR, nmse.aiekf.noKF.BetaR, nmse.aiekf.KF.BetaR];
%% ----------- Variable Assignment and Error Analysis UKF ----------------
ukf_mod_Fyf = modules.ukf.Fyf(1:length(T));
ukf_noKF_Fyf = ues.noKF.ukf.FyF(1:length(T));
ukf_KF_Fyf = ues.KF.ukf.Fyf(1:length(T))';
     
ukf_mod_Fyr = modules.ukf.Fyr(1:length(T));
ukf_noKF_Fyr = ues.noKF.ukf.FyR(1:length(T));
ukf_KF_Fyr = ues.KF.ukf.Fyr(1:length(T))';
     
ukf_mod_Vy = modules.ukf.Vy(1:length(T))';
ukf_noKF_Vy = ues.noKF.ukf.Vy(1:length(T))';
ukf_KF_Vy = ues.KF.ukf.Vy(1:length(T))';

ukf_mod_Ay = modules.ukf.Ay(1:length(T)); 
ukf_noKF_Ay = ues.noKF.ukf.Ay(1:length(T));
ukf_KF_Ay = ues.KF.ukf.Ay(1:length(T))';

ukf_mod_YawR = modules.ukf.YawR(1:length(T))';
ukf_noKF_YawR = ues.noKF.ukf.YawR(1:length(T))';
ukf_KF_YawR = ues.KF.ukf.YawR(1:length(T))';
     
ukf_mod_YawAcc = modules.ukf.YawAcc(1:length(T));
ukf_noKF_YawAcc = ues.noKF.ukf.YawAcc(1:length(T));
ukf_KF_YawAcc = ues.KF.ukf.YawAcc(1:length(T))';
     
ukf_mod_Beta = modules.ukf.Beta(1:length(T))';
ukf_noKF_Beta = ues.noKF.ukf.Beta(1:length(T))';
ukf_KF_Beta = ues.KF.ukf.Beta(1:length(T))';
     
ukf_mod_BetaR = modules.ukf.Betar(1:length(T));
ukf_noKF_BetaR = ues.noKF.ukf.Betar(1:length(T));
ukf_KF_BetaR = ues.KF.ukf.BetaR(1:length(T))';

nmse.ukf.mod.Fyf = goodnessOfFit( Fyf,ukf_mod_Fyf,'nmse');
nmse.ukf.mod.Fyr = goodnessOfFit( Fyr,ukf_mod_Fyr,'nmse');
nmse.ukf.mod.Vy = goodnessOfFit( Vy,ukf_mod_Vy,'nmse');
nmse.ukf.mod.Ay = goodnessOfFit( Ay,ukf_mod_Ay,'nmse');
nmse.ukf.mod.YawR = goodnessOfFit( YawR,ukf_mod_YawR,'nmse');
nmse.ukf.mod.YawAcc = goodnessOfFit( YawAcc,ukf_mod_YawAcc,'nmse');
nmse.ukf.mod.Beta = goodnessOfFit( Beta,ukf_mod_Beta,'nmse');
nmse.ukf.mod.BetaR = goodnessOfFit( BetaR,ukf_mod_BetaR,'nmse');

nmse.ukf.noKF.Fyf = goodnessOfFit( Fyf,ukf_noKF_Fyf,'nmse');
nmse.ukf.noKF.Fyr = goodnessOfFit( Fyr,ukf_noKF_Fyr,'nmse');
nmse.ukf.noKF.Vy = goodnessOfFit( Vy,ukf_noKF_Vy,'nmse');
nmse.ukf.noKF.Ay = goodnessOfFit( Ay,ukf_noKF_Ay,'nmse');
nmse.ukf.noKF.YawR = goodnessOfFit( YawR,ukf_noKF_YawR,'nmse');
nmse.ukf.noKF.YawAcc = goodnessOfFit( YawAcc,ukf_noKF_YawAcc,'nmse');
nmse.ukf.noKF.Beta = goodnessOfFit( Beta,ukf_noKF_Beta,'nmse');
nmse.ukf.noKF.BetaR = goodnessOfFit( BetaR,ukf_noKF_BetaR,'nmse');

nmse.ukf.KF.Fyf = goodnessOfFit( Fyf,ukf_KF_Fyf,'nmse');
nmse.ukf.KF.Fyr = goodnessOfFit( Fyr,ukf_KF_Fyr,'nmse');
nmse.ukf.KF.Vy = goodnessOfFit( Vy,ukf_KF_Vy,'nmse');
nmse.ukf.KF.Ay = goodnessOfFit( Ay,ukf_KF_Ay,'nmse');
nmse.ukf.KF.YawR = goodnessOfFit( YawR,ukf_KF_YawR,'nmse');
nmse.ukf.KF.YawAcc = goodnessOfFit( YawAcc,ukf_KF_YawAcc,'nmse');
nmse.ukf.KF.Beta = goodnessOfFit( Beta,ukf_KF_Beta,'nmse');
nmse.ukf.KF.BetaR = goodnessOfFit( BetaR,ukf_KF_BetaR,'nmse');

nmse.ukf.cumulative = ...
    [nmse.ukf.mod.Fyf, nmse.ukf.noKF.Fyf, nmse.ukf.KF.Fyf,...
     nmse.ukf.mod.Fyr, nmse.ukf.noKF.Fyr, nmse.ukf.KF.Fyr,...
     nmse.ukf.mod.Vy, nmse.ukf.noKF.Vy, nmse.ukf.KF.Vy,...
     nmse.ukf.mod.Ay, nmse.ukf.noKF.Ay, nmse.ukf.KF.Ay,...
     nmse.ukf.mod.YawR, nmse.ukf.noKF.YawR, nmse.ukf.KF.YawR,...
     nmse.ukf.mod.YawAcc, nmse.ukf.noKF.YawAcc, nmse.ukf.KF.YawAcc,...
     nmse.ukf.mod.Beta, nmse.ukf.noKF.Beta, nmse.ukf.KF.Beta,...
     nmse.ukf.mod.BetaR, nmse.ukf.noKF.BetaR, nmse.ukf.KF.BetaR];
%% ------------ Variable Assignment and Error Analysis PF ----------------
pf_mod_Fyf = modules.pf.Fyf(1:length(T));
pf_KF_Fyf = ues.KF.pf.Fyf(1:length(T))';
     
pf_mod_Fyr = modules.pf.Fyr(1:length(T));
pf_KF_Fyr = ues.KF.pf.Fyr(1:length(T))';
     
pf_mod_Vy = modules.pf.Vy(1:length(T))';
pf_KF_Vy = ues.KF.pf.Vy(1:length(T))';

pf_mod_Ay = modules.pf.Ay(1:length(T)); 
pf_KF_Ay = ues.KF.pf.Ay(1:length(T))';

pf_mod_YawR = modules.pf.YawR(1:length(T))';
pf_KF_YawR = ues.KF.pf.YawR(1:length(T))';
     
pf_mod_YawAcc = modules.pf.YawAcc(1:length(T));
pf_KF_YawAcc = ues.KF.pf.YawAcc(1:length(T))';
     
pf_mod_Beta = modules.pf.Beta(1:length(T))';
pf_KF_Beta = ues.KF.pf.Beta(1:length(T))';
     
pf_mod_BetaR = modules.pf.Betar(1:length(T));
pf_KF_BetaR = ues.KF.pf.BetaR(1:length(T))';
     
nmse.pf.mod.Fyf = goodnessOfFit( Fyf,pf_mod_Fyf,'nmse');
nmse.pf.mod.Fyr = goodnessOfFit( Fyr,pf_mod_Fyr,'nmse');
nmse.pf.mod.Vy = goodnessOfFit( Vy,pf_mod_Vy,'nmse');
nmse.pf.mod.Ay = goodnessOfFit( Ay,pf_mod_Ay,'nmse');
nmse.pf.mod.YawR = goodnessOfFit( YawR,pf_mod_YawR,'nmse');
nmse.pf.mod.YawAcc = goodnessOfFit( YawAcc,pf_mod_YawAcc,'nmse');
nmse.pf.mod.Beta = goodnessOfFit( Beta,pf_mod_Beta,'nmse');
nmse.pf.mod.BetaR = goodnessOfFit( BetaR,pf_mod_BetaR,'nmse');


nmse.pf.KF.Fyf = goodnessOfFit( Fyf,pf_KF_Fyf,'nmse');
nmse.pf.KF.Fyr = goodnessOfFit( Fyr,pf_KF_Fyr,'nmse');
nmse.pf.KF.Vy = goodnessOfFit( Vy,pf_KF_Vy,'nmse');
nmse.pf.KF.Ay = goodnessOfFit( Ay,pf_KF_Ay,'nmse');
nmse.pf.KF.YawR = goodnessOfFit( YawR,pf_KF_YawR,'nmse');
nmse.pf.KF.YawAcc = goodnessOfFit( YawAcc,pf_KF_YawAcc,'nmse');
nmse.pf.KF.Beta = goodnessOfFit( Beta,pf_KF_Beta,'nmse');
nmse.pf.KF.BetaR = goodnessOfFit( BetaR,pf_KF_BetaR,'nmse');

nmse.pf.cumulative = ...
    [nmse.pf.mod.Fyf, nmse.pf.KF.Fyf,...
     nmse.pf.mod.Fyr, nmse.pf.KF.Fyr,...
     nmse.pf.mod.Vy, nmse.pf.KF.Vy,...
     nmse.pf.mod.Ay, nmse.pf.KF.Ay,...
     nmse.pf.mod.YawR, nmse.pf.KF.YawR,...
     nmse.pf.mod.YawAcc, nmse.pf.KF.YawAcc,...
     nmse.pf.mod.Beta, nmse.pf.KF.Beta,...
     nmse.pf.mod.BetaR, nmse.pf.KF.BetaR];




