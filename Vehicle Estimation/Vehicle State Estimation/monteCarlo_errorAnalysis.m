
T = Time(1:800);

%% ----------------- Variable Assignment EKF -----------------------------
error.ekf.mod.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.ekf.mod_Fyf(1:800,:);
error.ekf.noKF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.ekf.noKF_Fyf(1:800,:);
error.ekf.KF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.ekf.KF_Fyf(1:800,:);
     
error.ekf.mod.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.ekf.mod_Fyr(1:800,:);
error.ekf.noKF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.ekf.noKF_Fyr(1:800,:);
error.ekf.KF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.ekf.KF_Fyr(1:800,:);
     
error.ekf.mod.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.ekf.mod_Vy(1:800,:);
error.ekf.noKF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.ekf.noKF_Vy(1:800,:);
error.ekf.KF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.ekf.KF_Vy(1:800,:);

error.ekf.mod.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.ekf.mod_Ay(1:800,:);
error.ekf.noKF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.ekf.noKF_Ay(1:800,:);
error.ekf.KF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.ekf.KF_Ay(1:800,:);

error.ekf.mod.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.ekf.mod_YawR(1:800,:);
error.ekf.noKF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.ekf.noKF_YawR(1:800,:);
error.ekf.KF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.ekf.KF_YawR(1:800,:);
     
error.ekf.mod.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.ekf.mod_YawAcc(1:800,:);
error.ekf.noKF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.ekf.noKF_YawAcc(1:800,:);
error.ekf.KF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.ekf.KF_YawAcc(1:800,:);
     
error.ekf.mod.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.ekf.mod_Beta(1:800,:);
error.ekf.noKF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.ekf.noKF_Beta(1:800,:);
error.ekf.KF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.ekf.KF_Beta(1:800,:);
     
error.ekf.mod.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.ekf.mod_BetaR(1:800,:);
error.ekf.noKF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.ekf.noKF_BetaR(1:800,:);
error.ekf.KF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.ekf.KF_BetaR(1:800,:);

rmeansqerror.ekf.mod.Fyf = sqrt(mean(error.ekf.mod.Fyf.^2,2));
rmeansqerror.ekf.noKF.Fyf = sqrt(mean(error.ekf.noKF.Fyf.^2,2));
rmeansqerror.ekf.KF.Fyf = sqrt(mean(error.ekf.KF.Fyf.^2,2));
     
rmeansqerror.ekf.mod.Fyr = sqrt(mean(error.ekf.mod.Fyr.^2,2));
rmeansqerror.ekf.noKF.Fyr = sqrt(mean(error.ekf.noKF.Fyr.^2,2));
rmeansqerror.ekf.KF.Fyr = sqrt(mean(error.ekf.KF.Fyr.^2,2));
     
rmeansqerror.ekf.mod.Vy = sqrt(mean(error.ekf.mod.Vy.^2,2));
rmeansqerror.ekf.noKF.Vy = sqrt(mean(error.ekf.noKF.Vy.^2,2));
rmeansqerror.ekf.KF.Vy = sqrt(mean(error.ekf.KF.Vy.^2,2));

rmeansqerror.ekf.mod.Ay = sqrt(mean(error.ekf.mod.Ay.^2,2));
rmeansqerror.ekf.noKF.Ay = sqrt(mean(error.ekf.noKF.Ay.^2,2));
rmeansqerror.ekf.KF.Ay = sqrt(mean(error.ekf.KF.Ay.^2,2));

rmeansqerror.ekf.mod.YawR = sqrt(mean(error.ekf.mod.YawR.^2,2));
rmeansqerror.ekf.noKF.YawR = sqrt(mean(error.ekf.noKF.YawR.^2,2));
rmeansqerror.ekf.KF.YawR = sqrt(mean(error.ekf.KF.YawR.^2,2));
     
rmeansqerror.ekf.mod.YawAcc = sqrt(mean(error.ekf.mod.YawAcc.^2,2));
rmeansqerror.ekf.noKF.YawAcc= sqrt(mean(error.ekf.noKF.YawAcc.^2,2));
rmeansqerror.ekf.KF.YawAcc = sqrt(mean(error.ekf.KF.YawAcc.^2,2));
     
rmeansqerror.ekf.mod.Beta = sqrt(mean(error.ekf.mod.Beta.^2,2));
rmeansqerror.ekf.noKF.Beta = sqrt(mean(error.ekf.noKF.Beta.^2,2));
rmeansqerror.ekf.KF.Beta = sqrt(mean(error.ekf.KF.Beta.^2,2));

rmeansqerror.ekf.mod.BetaR = sqrt(mean(error.ekf.mod.BetaR.^2,2));
rmeansqerror.ekf.noKF.BetaR = sqrt(mean(error.ekf.noKF.BetaR.^2,2));
rmeansqerror.ekf.KF.BetaR = sqrt(mean(error.ekf.KF.BetaR.^2,2));
%% --------------------- Variable Assignment IEKF ------------------------
error.iekf.mod.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.iekf.mod_Fyf(1:800,:);
error.iekf.noKF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.iekf.noKF_Fyf(1:800,:);
error.iekf.KF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.iekf.KF_Fyf(1:800,:);
     
error.iekf.mod.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.iekf.mod_Fyr(1:800,:);
error.iekf.noKF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.iekf.noKF_Fyr(1:800,:);
error.iekf.KF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.iekf.KF_Fyr(1:800,:);
     
error.iekf.mod.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.iekf.mod_Vy(1:800,:);
error.iekf.noKF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.iekf.noKF_Vy(1:800,:);
error.iekf.KF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.iekf.KF_Vy(1:800,:);

error.iekf.mod.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.iekf.mod_Ay(1:800,:);
error.iekf.noKF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.iekf.noKF_Ay(1:800,:);
error.iekf.KF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.iekf.KF_Ay(1:800,:);

error.iekf.mod.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.iekf.mod_YawR(1:800,:);
error.iekf.noKF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.iekf.noKF_YawR(1:800,:);
error.iekf.KF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.iekf.KF_YawR(1:800,:);
     
error.iekf.mod.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.iekf.mod_YawAcc(1:800,:);
error.iekf.noKF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.iekf.noKF_YawAcc(1:800,:);
error.iekf.KF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.iekf.KF_YawAcc(1:800,:);
     
error.iekf.mod.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.iekf.mod_Beta(1:800,:);
error.iekf.noKF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.iekf.noKF_Beta(1:800,:);
error.iekf.KF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.iekf.KF_Beta(1:800,:);
     
error.iekf.mod.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.iekf.mod_BetaR(1:800,:);
error.iekf.noKF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.iekf.noKF_BetaR(1:800,:);
error.iekf.KF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.iekf.KF_BetaR(1:800,:);

rmeansqerror.iekf.mod.Fyf = sqrt(mean(error.iekf.mod.Fyf.^2,2));
rmeansqerror.iekf.noKF.Fyf = sqrt(mean(error.iekf.noKF.Fyf.^2,2));
rmeansqerror.iekf.KF.Fyf = sqrt(mean(error.iekf.KF.Fyf.^2,2));
     
rmeansqerror.iekf.mod.Fyr = sqrt(mean(error.iekf.mod.Fyr.^2,2));
rmeansqerror.iekf.noKF.Fyr = sqrt(mean(error.iekf.noKF.Fyr.^2,2));
rmeansqerror.iekf.KF.Fyr = sqrt(mean(error.iekf.KF.Fyr.^2,2));
     
rmeansqerror.iekf.mod.Vy = sqrt(mean(error.iekf.mod.Vy.^2,2));
rmeansqerror.iekf.noKF.Vy = sqrt(mean(error.iekf.noKF.Vy.^2,2));
rmeansqerror.iekf.KF.Vy = sqrt(mean(error.iekf.KF.Vy.^2,2));

rmeansqerror.iekf.mod.Ay = sqrt(mean(error.iekf.mod.Ay.^2,2));
rmeansqerror.iekf.noKF.Ay = sqrt(mean(error.iekf.noKF.Ay.^2,2));
rmeansqerror.iekf.KF.Ay = sqrt(mean(error.iekf.KF.Ay.^2,2));

rmeansqerror.iekf.mod.YawR = sqrt(mean(error.iekf.mod.YawR.^2,2));
rmeansqerror.iekf.noKF.YawR = sqrt(mean(error.iekf.noKF.YawR.^2,2));
rmeansqerror.iekf.KF.YawR = sqrt(mean(error.iekf.KF.YawR.^2,2));
     
rmeansqerror.iekf.mod.YawAcc = sqrt(mean(error.iekf.mod.YawAcc.^2,2));
rmeansqerror.iekf.noKF.YawAcc= sqrt(mean(error.iekf.noKF.YawAcc.^2,2));
rmeansqerror.iekf.KF.YawAcc = sqrt(mean(error.iekf.KF.YawAcc.^2,2));
     
rmeansqerror.iekf.mod.Beta = sqrt(mean(error.iekf.mod.Beta.^2,2));
rmeansqerror.iekf.noKF.Beta = sqrt(mean(error.iekf.noKF.Beta.^2,2));
rmeansqerror.iekf.KF.Beta = sqrt(mean(error.iekf.KF.Beta.^2,2));

rmeansqerror.iekf.mod.BetaR = sqrt(mean(error.iekf.mod.BetaR.^2,2));
rmeansqerror.iekf.noKF.BetaR = sqrt(mean(error.iekf.noKF.BetaR.^2,2));
rmeansqerror.iekf.KF.BetaR = sqrt(mean(error.iekf.KF.BetaR.^2,2));
%% -------------------- Variable Assignment AIEKF ------------------------
error.aiekf.mod.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.aiekf.mod_Fyf(1:800,:);
error.aiekf.noKF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.aiekf.noKF_Fyf(1:800,:);
error.aiekf.KF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.aiekf.KF_Fyf(1:800,:);
     
error.aiekf.mod.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.aiekf.mod_Fyr(1:800,:);
error.aiekf.noKF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.aiekf.noKF_Fyr(1:800,:);
error.aiekf.KF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.aiekf.KF_Fyr(1:800,:);
     
error.aiekf.mod.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.aiekf.mod_Vy(1:800,:);
error.aiekf.noKF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.aiekf.noKF_Vy(1:800,:);
error.aiekf.KF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.aiekf.KF_Vy(1:800,:);

error.aiekf.mod.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.aiekf.mod_Ay(1:800,:);
error.aiekf.noKF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.aiekf.noKF_Ay(1:800,:);
error.aiekf.KF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.aiekf.KF_Ay(1:800,:);

error.aiekf.mod.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.aiekf.mod_YawR(1:800,:);
error.aiekf.noKF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.aiekf.noKF_YawR(1:800,:);
error.aiekf.KF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.aiekf.KF_YawR(1:800,:);
     
error.aiekf.mod.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.aiekf.mod_YawAcc(1:800,:);
error.aiekf.noKF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.aiekf.noKF_YawAcc(1:800,:);
error.aiekf.KF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.aiekf.KF_YawAcc(1:800,:);
     
error.aiekf.mod.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.aiekf.mod_Beta(1:800,:);
error.aiekf.noKF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.aiekf.noKF_Beta(1:800,:);
error.aiekf.KF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.aiekf.KF_Beta(1:800,:);
     
error.aiekf.mod.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.aiekf.mod_BetaR(1:800,:);
error.aiekf.noKF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.aiekf.noKF_BetaR(1:800,:);
error.aiekf.KF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.aiekf.KF_BetaR(1:800,:);

rmeansqerror.aiekf.mod.Fyf = sqrt(mean(error.aiekf.mod.Fyf.^2,2));
rmeansqerror.aiekf.noKF.Fyf = sqrt(mean(error.aiekf.noKF.Fyf.^2,2));
rmeansqerror.aiekf.KF.Fyf = sqrt(mean(error.aiekf.KF.Fyf.^2,2));
     
rmeansqerror.aiekf.mod.Fyr = sqrt(mean(error.aiekf.mod.Fyr.^2,2));
rmeansqerror.aiekf.noKF.Fyr = sqrt(mean(error.aiekf.noKF.Fyr.^2,2));
rmeansqerror.aiekf.KF.Fyr = sqrt(mean(error.aiekf.KF.Fyr.^2,2));
     
rmeansqerror.aiekf.mod.Vy = sqrt(mean(error.aiekf.mod.Vy.^2,2));
rmeansqerror.aiekf.noKF.Vy = sqrt(mean(error.aiekf.noKF.Vy.^2,2));
rmeansqerror.aiekf.KF.Vy = sqrt(mean(error.aiekf.KF.Vy.^2,2));

rmeansqerror.aiekf.mod.Ay = sqrt(mean(error.aiekf.mod.Ay.^2,2));
rmeansqerror.aiekf.noKF.Ay = sqrt(mean(error.aiekf.noKF.Ay.^2,2));
rmeansqerror.aiekf.KF.Ay = sqrt(mean(error.aiekf.KF.Ay.^2,2));

rmeansqerror.aiekf.mod.YawR = sqrt(mean(error.aiekf.mod.YawR.^2,2));
rmeansqerror.aiekf.noKF.YawR = sqrt(mean(error.aiekf.noKF.YawR.^2,2));
rmeansqerror.aiekf.KF.YawR = sqrt(mean(error.aiekf.KF.YawR.^2,2));
     
rmeansqerror.aiekf.mod.YawAcc = sqrt(mean(error.aiekf.mod.YawAcc.^2,2));
rmeansqerror.aiekf.noKF.YawAcc= sqrt(mean(error.aiekf.noKF.YawAcc.^2,2));
rmeansqerror.aiekf.KF.YawAcc = sqrt(mean(error.aiekf.KF.YawAcc.^2,2));
     
rmeansqerror.aiekf.mod.Beta = sqrt(mean(error.aiekf.mod.Beta.^2,2));
rmeansqerror.aiekf.noKF.Beta = sqrt(mean(error.aiekf.noKF.Beta.^2,2));
rmeansqerror.aiekf.KF.Beta = sqrt(mean(error.aiekf.KF.Beta.^2,2));

rmeansqerror.aiekf.mod.BetaR = sqrt(mean(error.aiekf.mod.BetaR.^2,2));
rmeansqerror.aiekf.noKF.BetaR = sqrt(mean(error.aiekf.noKF.BetaR.^2,2));
rmeansqerror.aiekf.KF.BetaR = sqrt(mean(error.aiekf.KF.BetaR.^2,2));
%% ---------------------- Variable Assignment UKF ------------------------
error.ukf.mod.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.ukf.mod_Fyf(1:800,:);
error.ukf.noKF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.ukf.noKF_Fyf(1:800,:);
error.ukf.KF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.ukf.KF_Fyf(1:800,:);
     
error.ukf.mod.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.ukf.mod_Fyr(1:800,:);
error.ukf.noKF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.ukf.noKF_Fyr(1:800,:);
error.ukf.KF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.ukf.KF_Fyr(1:800,:);
     
error.ukf.mod.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.ukf.mod_Vy(1:800,:);
error.ukf.noKF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.ukf.noKF_Vy(1:800,:);
error.ukf.KF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.ukf.KF_Vy(1:800,:);

error.ukf.mod.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.ukf.mod_Ay(1:800,:);
error.ukf.noKF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.ukf.noKF_Ay(1:800,:);
error.ukf.KF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.ukf.KF_Ay(1:800,:);

error.ukf.mod.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.ukf.mod_YawR(1:800,:);
error.ukf.noKF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.ukf.noKF_YawR(1:800,:);
error.ukf.KF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.ukf.KF_YawR(1:800,:);
     
error.ukf.mod.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.ukf.mod_YawAcc(1:800,:);
error.ukf.noKF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.ukf.noKF_YawAcc(1:800,:);
error.ukf.KF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.ukf.KF_YawAcc(1:800,:);
     
error.ukf.mod.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.ukf.mod_Beta(1:800,:);
error.ukf.noKF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.ukf.noKF_Beta(1:800,:);
error.ukf.KF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.ukf.KF_Beta(1:800,:);
     
error.ukf.mod.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.ukf.mod_BetaR(1:800,:);
error.ukf.noKF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.ukf.noKF_BetaR(1:800,:);
error.ukf.KF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.ukf.KF_BetaR(1:800,:);

rmeansqerror.ukf.mod.Fyf = sqrt(mean(error.ukf.mod.Fyf.^2,2));
rmeansqerror.ukf.noKF.Fyf = sqrt(mean(error.ukf.noKF.Fyf.^2,2));
rmeansqerror.ukf.KF.Fyf = sqrt(mean(error.ukf.KF.Fyf.^2,2));
     
rmeansqerror.ukf.mod.Fyr = sqrt(mean(error.ukf.mod.Fyr.^2,2));
rmeansqerror.ukf.noKF.Fyr = sqrt(mean(error.ukf.noKF.Fyr.^2,2));
rmeansqerror.ukf.KF.Fyr = sqrt(mean(error.ukf.KF.Fyr.^2,2));
     
rmeansqerror.ukf.mod.Vy = sqrt(mean(error.ukf.mod.Vy.^2,2));
rmeansqerror.ukf.noKF.Vy = sqrt(mean(error.ukf.noKF.Vy.^2,2));
rmeansqerror.ukf.KF.Vy = sqrt(mean(error.ukf.KF.Vy.^2,2));

rmeansqerror.ukf.mod.Ay = sqrt(mean(error.ukf.mod.Ay.^2,2));
rmeansqerror.ukf.noKF.Ay = sqrt(mean(error.ukf.noKF.Ay.^2,2));
rmeansqerror.ukf.KF.Ay = sqrt(mean(error.ukf.KF.Ay.^2,2));

rmeansqerror.ukf.mod.YawR = sqrt(mean(error.ukf.mod.YawR.^2,2));
rmeansqerror.ukf.noKF.YawR = sqrt(mean(error.ukf.noKF.YawR.^2,2));
rmeansqerror.ukf.KF.YawR = sqrt(mean(error.ukf.KF.YawR.^2,2));
     
rmeansqerror.ukf.mod.YawAcc = sqrt(mean(error.ukf.mod.YawAcc.^2,2));
rmeansqerror.ukf.noKF.YawAcc= sqrt(mean(error.ukf.noKF.YawAcc.^2,2));
rmeansqerror.ukf.KF.YawAcc = sqrt(mean(error.ukf.KF.YawAcc.^2,2));
     
rmeansqerror.ukf.mod.Beta = sqrt(mean(error.ukf.mod.Beta.^2,2));
rmeansqerror.ukf.noKF.Beta = sqrt(mean(error.ukf.noKF.Beta.^2,2));
rmeansqerror.ukf.KF.Beta = sqrt(mean(error.ukf.KF.Beta.^2,2));

rmeansqerror.ukf.mod.BetaR = sqrt(mean(error.ukf.mod.BetaR.^2,2));
rmeansqerror.ukf.noKF.BetaR = sqrt(mean(error.ukf.noKF.BetaR.^2,2));
rmeansqerror.ukf.KF.BetaR = sqrt(mean(error.ukf.KF.BetaR.^2,2));
%% ---------------------- Variable Assignment PF -------------------------
error.pf.mod.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.pf.mod_Fyf(1:800,:);
error.pf.KF.Fyf = mCarlo.true.Fyf(1:800,:) - mCarlo.pf.KF_Fyf(1:800,:);
     
error.pf.mod.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.pf.mod_Fyr(1:800,:);
error.pf.KF.Fyr = mCarlo.true.Fyr(1:800,:) - mCarlo.pf.KF_Fyr(1:800,:);
     
error.pf.mod.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.pf.mod_Vy(1:800,:);
error.pf.KF.Vy = mCarlo.true.Vy(1:800,:) - mCarlo.pf.KF_Vy(1:800,:);

error.pf.mod.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.pf.mod_Ay(1:800,:);
error.pf.KF.Ay = mCarlo.true.Ay(1:800,:) - mCarlo.pf.KF_Ay(1:800,:);

error.pf.mod.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.pf.mod_YawR(1:800,:);
error.pf.KF.YawR = mCarlo.true.YawR(1:800,:) - mCarlo.pf.KF_YawR(1:800,:);
     
error.pf.mod.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.pf.mod_YawAcc(1:800,:);
error.pf.KF.YawAcc = mCarlo.true.YawAcc(1:800,:) - mCarlo.pf.KF_YawAcc(1:800,:);
     
error.pf.mod.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.pf.mod_Beta(1:800,:);
error.pf.KF.Beta = mCarlo.true.Beta(1:800,:) - mCarlo.pf.KF_Beta(1:800,:);
     
error.pf.mod.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.pf.mod_BetaR(1:800,:);
error.pf.KF.BetaR = mCarlo.true.BetaR(1:800,:) - mCarlo.pf.KF_BetaR(1:800,:);

rmeansqerror.pf.mod.Fyf = sqrt(mean(error.pf.mod.Fyf.^2,2));
rmeansqerror.pf.KF.Fyf = sqrt(mean(error.pf.KF.Fyf.^2,2));
     
rmeansqerror.pf.mod.Fyr = sqrt(mean(error.pf.mod.Fyr.^2,2));
rmeansqerror.pf.KF.Fyr = sqrt(mean(error.pf.KF.Fyr.^2,2));
     
rmeansqerror.pf.mod.Vy = sqrt(mean(error.pf.mod.Vy.^2,2));
rmeansqerror.pf.KF.Vy = sqrt(mean(error.pf.KF.Vy.^2,2));

rmeansqerror.pf.mod.Ay = sqrt(mean(error.pf.mod.Ay.^2,2));
rmeansqerror.pf.KF.Ay = sqrt(mean(error.pf.KF.Ay.^2,2));

rmeansqerror.pf.mod.YawR = sqrt(mean(error.pf.mod.YawR.^2,2));
rmeansqerror.pf.KF.YawR = sqrt(mean(error.pf.KF.YawR.^2,2));
     
rmeansqerror.pf.mod.YawAcc = sqrt(mean(error.pf.mod.YawAcc.^2,2));
rmeansqerror.pf.KF.YawAcc = sqrt(mean(error.pf.KF.YawAcc.^2,2));
     
rmeansqerror.pf.mod.Beta = sqrt(mean(error.pf.mod.Beta.^2,2));
rmeansqerror.pf.KF.Beta = sqrt(mean(error.pf.KF.Beta.^2,2));

rmeansqerror.pf.mod.BetaR = sqrt(mean(error.pf.mod.BetaR.^2,2));
rmeansqerror.pf.KF.BetaR = sqrt(mean(error.pf.KF.BetaR.^2,2));


%% --------------------------- Plotting ----------------------------------
% ------------------------------------------------------------------------
% ------------------------ Plotting Modules ------------------------------
% ------------------------------------------------------------------------
figure(1)
subplot(4,2,1)
plot(T, rmeansqerror.ekf.mod.Fyf,'b-', ...
     T, rmeansqerror.iekf.mod.Fyf,'g.-', ...
     T, rmeansqerror.aiekf.mod.Fyf,'r',...
     T, rmeansqerror.ukf.mod.Fyf,'k--',...
     T, rmeansqerror.pf.mod.Fyf,'m')
xlim([0,T(end)]);
ylabel('A_a');
ylim([0,max(rmeansqerror.ekf.mod.Fyf)*2])
xlabel('T')

subplot(4,2,2)
plot(T, rmeansqerror.ekf.mod.Fyr,'b-', ...
     T, rmeansqerror.iekf.mod.Fyr,'g.-', ...
     T, rmeansqerror.aiekf.mod.Fyr,'r',...
     T, rmeansqerror.ukf.mod.Fyr,'k--',...
     T, rmeansqerror.pf.mod.Fyr,'m')
xlim([0,T(end)]);
ylim([0,max(rmeansqerror.ekf.mod.Fyr)*2])
ylabel('A_a');
xlabel('T')

subplot(4,2,3)
plot(T, rmeansqerror.ekf.mod.Vy,'b-', ...
     T, rmeansqerror.iekf.mod.Vy,'g.-', ...
     T, rmeansqerror.aiekf.mod.Vy,'r',...
     T, rmeansqerror.ukf.mod.Vy,'k--',...
     T, rmeansqerror.pf.mod.Vy,'m')
xlim([0,T(end)]);
ylim([0,max(rmeansqerror.ekf.mod.Vy)*2])
ylabel('A_a');
xlabel('T')

subplot(4,2,4)
plot(T, rmeansqerror.ekf.mod.Ay,'b-', ...
     T, rmeansqerror.iekf.mod.Ay,'g.-', ...
     T, rmeansqerror.aiekf.mod.Ay,'r',...
     T, rmeansqerror.ukf.mod.Ay,'k--',...
     T, rmeansqerror.pf.mod.Ay,'m')
 ylim([0,max(rmeansqerror.ekf.mod.Ay)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,5)
plot(T, rmeansqerror.ekf.mod.YawR,'b-', ...
     T, rmeansqerror.iekf.mod.YawR,'g.-', ...
     T, rmeansqerror.aiekf.mod.YawR,'r',...
     T, rmeansqerror.ukf.mod.YawR,'k--',...
     T, rmeansqerror.pf.mod.YawR,'m')
 ylim([0,max(rmeansqerror.ekf.mod.YawR)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,6)
plot(T, rmeansqerror.ekf.mod.YawAcc,'b-', ...
     T, rmeansqerror.iekf.mod.YawAcc,'g.-', ...
     T, rmeansqerror.aiekf.mod.YawAcc,'r',...
     T, rmeansqerror.ukf.mod.YawAcc,'k--',...
     T, rmeansqerror.pf.mod.YawAcc,'m')
 ylim([0,max(rmeansqerror.ekf.mod.YawAcc)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,7)
plot(T, rmeansqerror.ekf.mod.Beta,'b-', ...
     T, rmeansqerror.iekf.mod.Beta,'g.-', ...
     T, rmeansqerror.aiekf.mod.Beta,'r',...
     T, rmeansqerror.ukf.mod.Beta,'k--',...
     T, rmeansqerror.pf.mod.Beta,'m')
 ylim([0,max(rmeansqerror.ekf.mod.Beta)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,8)
plot(T, rmeansqerror.ekf.mod.BetaR,'b-', ...
     T, rmeansqerror.iekf.mod.BetaR,'g.-', ...
     T, rmeansqerror.aiekf.mod.BetaR,'r',...
     T, rmeansqerror.ukf.mod.BetaR,'k--',...
     T, rmeansqerror.pf.mod.BetaR,'m')
 ylim([0,max(rmeansqerror.ekf.mod.BetaR)])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

sgtitle('RMSE Modules')


%% ------------------------------------------------------------------------
% -------------------------- Plotting noKF -------------------------------
% ------------------------------------------------------------------------
figure(2)
subplot(4,2,1)
plot(T, rmeansqerror.ekf.noKF.Fyf,'b-', ...
     T, rmeansqerror.iekf.noKF.Fyf,'g--', ...
     T, rmeansqerror.aiekf.noKF.Fyf,'r',...
     T, rmeansqerror.ukf.noKF.Fyf,'k--')
xlim([0,T(end)]);
ylabel('A_a');
ylim([0,max(rmeansqerror.ekf.noKF.Fyf)*2])
xlabel('T')

subplot(4,2,2)
plot(T, rmeansqerror.ekf.noKF.Fyr,'b-', ...
     T, rmeansqerror.iekf.noKF.Fyr,'g--', ...
     T, rmeansqerror.aiekf.noKF.Fyr,'r',...
     T, rmeansqerror.ukf.noKF.Fyr,'k--')
xlim([0,T(end)]);
ylim([0,max(rmeansqerror.ekf.noKF.Fyr)*2])
ylabel('A_a');
xlabel('T')

subplot(4,2,3)
plot(T, rmeansqerror.ekf.noKF.Vy,'b-', ...
     T, rmeansqerror.iekf.noKF.Vy,'g--', ...
     T, rmeansqerror.aiekf.noKF.Vy,'r',...
     T, rmeansqerror.ukf.noKF.Vy,'k--')
xlim([0,T(end)]);
ylim([0,max(rmeansqerror.ekf.noKF.Vy)*2])
ylabel('A_a');
xlabel('T')

subplot(4,2,4)
plot(T, rmeansqerror.ekf.noKF.Ay,'b-', ...
     T, rmeansqerror.iekf.noKF.Ay,'g--', ...
     T, rmeansqerror.aiekf.noKF.Ay,'r',...
     T, rmeansqerror.ukf.noKF.Ay,'k--')
 ylim([0,max(rmeansqerror.ekf.noKF.Ay)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,5)
plot(T, rmeansqerror.ekf.noKF.YawR,'b-', ...
     T, rmeansqerror.iekf.noKF.YawR,'g--', ...
     T, rmeansqerror.aiekf.noKF.YawR,'r',...
     T, rmeansqerror.ukf.noKF.YawR,'k--')
 ylim([0,max(rmeansqerror.ekf.noKF.YawR)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,6)
plot(T, rmeansqerror.ekf.noKF.YawAcc,'b-', ...
     T, rmeansqerror.iekf.noKF.YawAcc,'g--', ...
     T, rmeansqerror.aiekf.noKF.YawAcc,'r',...
     T, rmeansqerror.ukf.noKF.YawAcc,'k--')
 ylim([0,max(rmeansqerror.ekf.noKF.YawAcc)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,7)
plot(T, rmeansqerror.ekf.noKF.Beta,'b-', ...
     T, rmeansqerror.iekf.noKF.Beta,'g--', ...
     T, rmeansqerror.aiekf.noKF.Beta,'r',...
     T, rmeansqerror.ukf.noKF.Beta,'k--')
 ylim([0,max(rmeansqerror.ekf.noKF.Beta)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,8)
plot(T, rmeansqerror.ekf.noKF.BetaR,'b-', ...
     T, rmeansqerror.iekf.noKF.BetaR,'g--', ...
     T, rmeansqerror.aiekf.noKF.BetaR,'r',...
     T, rmeansqerror.ukf.noKF.BetaR,'k--')
 ylim([0,max(rmeansqerror.ekf.noKF.BetaR)])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

sgtitle('RMSE noKF')

%% ------------------------------------------------------------------------
% --------------------------- Plotting KF --------------------------------
% ------------------------------------------------------------------------
figure(3)
subplot(4,2,1)
plot(T, rmeansqerror.ekf.KF.Fyf,'b-', ...
     T, rmeansqerror.iekf.KF.Fyf,'g.-', ...
     T, rmeansqerror.aiekf.KF.Fyf,'r',...
     T, rmeansqerror.ukf.KF.Fyf,'k--',...
     T, rmeansqerror.pf.KF.Fyf,'m')
xlim([0,T(end)]);
ylabel('A_a');
ylim([0,max(rmeansqerror.ekf.KF.Fyf)*2])
xlabel('T')

subplot(4,2,2)
plot(T, rmeansqerror.ekf.KF.Fyr,'b-', ...
     T, rmeansqerror.iekf.KF.Fyr,'g.-', ...
     T, rmeansqerror.aiekf.KF.Fyr,'r',...
     T, rmeansqerror.ukf.KF.Fyr,'k--',...
     T, rmeansqerror.pf.KF.Fyr,'m')
xlim([0,T(end)]);
ylim([0,max(rmeansqerror.ekf.KF.Fyr)*2])
ylabel('A_a');
xlabel('T')

subplot(4,2,3)
plot(T, rmeansqerror.ekf.KF.Vy,'b-', ...
     T, rmeansqerror.iekf.KF.Vy,'g.-', ...
     T, rmeansqerror.aiekf.KF.Vy,'r',...
     T, rmeansqerror.ukf.KF.Vy,'k--',...
     T, rmeansqerror.pf.KF.Vy,'m')
xlim([0,T(end)]);
ylim([0,max(rmeansqerror.ekf.KF.Vy)*2])
ylabel('A_a');
xlabel('T')

subplot(4,2,4)
plot(T, rmeansqerror.ekf.KF.Ay,'b-', ...
     T, rmeansqerror.iekf.KF.Ay,'g.-', ...
     T, rmeansqerror.aiekf.KF.Ay,'r',...
     T, rmeansqerror.ukf.KF.Ay,'k--',...
     T, rmeansqerror.pf.KF.Ay,'m')
 ylim([0,max(rmeansqerror.ekf.KF.Ay)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,5)
plot(T, rmeansqerror.ekf.KF.YawR,'b-', ...
     T, rmeansqerror.iekf.KF.YawR,'g.-', ...
     T, rmeansqerror.aiekf.KF.YawR,'r',...
     T, rmeansqerror.ukf.KF.YawR,'k--',...
     T, rmeansqerror.pf.KF.YawR,'m')
 ylim([0,max(rmeansqerror.ekf.KF.YawR)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,6)
plot(T, rmeansqerror.ekf.KF.YawAcc,'b-', ...
     T, rmeansqerror.iekf.KF.YawAcc,'g.-', ...
     T, rmeansqerror.aiekf.KF.YawAcc,'r',...
     T, rmeansqerror.ukf.KF.YawAcc,'k--',...
     T, rmeansqerror.pf.KF.YawAcc,'m')
 ylim([0,max(rmeansqerror.ekf.KF.YawAcc)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,7)
plot(T, rmeansqerror.ekf.KF.Beta,'b-', ...
     T, rmeansqerror.iekf.KF.Beta,'g.-', ...
     T, rmeansqerror.aiekf.KF.Beta,'r',...
     T, rmeansqerror.ukf.KF.Beta,'k--',...
     T, rmeansqerror.pf.KF.Beta,'m')
 ylim([0,max(rmeansqerror.ekf.KF.Beta)*2])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

subplot(4,2,8)
plot(T, rmeansqerror.ekf.KF.BetaR,'b-', ...
     T, rmeansqerror.iekf.KF.BetaR,'g.-', ...
     T, rmeansqerror.aiekf.KF.BetaR,'r',...
     T, rmeansqerror.ukf.KF.BetaR,'k--',...
     T, rmeansqerror.pf.KF.BetaR,'m')
 ylim([0,max(rmeansqerror.ekf.KF.BetaR)])
xlim([0,T(end)]);
ylabel('A_a');
xlabel('T')

sgtitle('RMSE KF')