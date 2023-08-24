
% Call this file from monteCarlo_dataGen
%% ----------------- Variable Assignment EKF -----------------------------
mCarlo.true.YawAcc(:,index)  = YawAcc;
mCarlo.true.BetaR(:,index) = BetaR;
mCarlo.true.Ay(:,index) = ([0; diff(Vy)]+Vx.*YawR);
mCarlo.true.time(:,index) = Time;
mCarlo.true.Beta(:,index) = Beta;
mCarlo.true.YawR(:,index) = YawR;
mCarlo.true.Fyf(:,index) = Fyf;
mCarlo.true.Fyr(:,index) = Fyr;
mCarlo.true.Vy(:,index) = Vy;

%% ----------------- Variable Assignment EKF -----------------------------
mCarlo.ekf.mod_Fyf(:,index) = modules.ekf.Fyf;
mCarlo.ekf.noKF_Fyf(:,index) = ues.noKF.ekf.FyF';
mCarlo.ekf.KF_Fyf(:,index) = ues.KF.ekf.Fyf';
     
mCarlo.ekf.mod_Fyr(:,index) = modules.ekf.Fyr;
mCarlo.ekf.noKF_Fyr(:,index) = ues.noKF.ekf.FyR';
mCarlo.ekf.KF_Fyr(:,index) = ues.KF.ekf.Fyr';
     
mCarlo.ekf.mod_Vy(:,index) = modules.ekf.Vy';
mCarlo.ekf.noKF_Vy(:,index) = ues.noKF.ekf.Vy';
mCarlo.ekf.KF_Vy(:,index) = ues.KF.ekf.Vy';

mCarlo.ekf.mod_Ay(:,index) = modules.ekf.Ay; 
mCarlo.ekf.noKF_Ay(:,index) = ues.noKF.ekf.Ay;
mCarlo.ekf.KF_Ay(:,index) = ues.KF.ekf.Ay';

mCarlo.ekf.mod_YawR(:,index) = modules.ekf.YawR';
mCarlo.ekf.noKF_YawR(:,index) = ues.noKF.ekf.YawR';
mCarlo.ekf.KF_YawR(:,index) = ues.KF.ekf.YawR';
     
mCarlo.ekf.mod_YawAcc(:,index) = modules.ekf.YawAcc;
mCarlo.ekf.noKF_YawAcc(:,index) = ues.noKF.ekf.YawAcc;
mCarlo.ekf.KF_YawAcc(:,index) = ues.KF.ekf.YawAcc';
     
mCarlo.ekf.mod_Beta(:,index) = modules.ekf.Beta';
mCarlo.ekf.noKF_Beta(:,index) = ues.noKF.ekf.Beta';
mCarlo.ekf.KF_Beta(:,index) = ues.KF.ekf.Beta';
     
mCarlo.ekf.mod_BetaR(:,index) = modules.ekf.Betar;
mCarlo.ekf.noKF_BetaR(:,index) = ues.noKF.ekf.Betar;
mCarlo.ekf.KF_BetaR(:,index) = ues.KF.ekf.BetaR';

%% --------------------- Variable Assignment IEKF ------------------------
mCarlo.iekf.mod_Fyf(:,index) = modules.iekf.Fyf;
mCarlo.iekf.noKF_Fyf(:,index) = ues.noKF.iekf.FyF';
mCarlo.iekf.KF_Fyf(:,index) = ues.KF.iekf.Fyf';
     
mCarlo.iekf.mod_Fyr(:,index) = modules.iekf.Fyr;
mCarlo.iekf.noKF_Fyr(:,index) = ues.noKF.iekf.FyR';
mCarlo.iekf.KF_Fyr(:,index) = ues.KF.iekf.Fyr';
     
mCarlo.iekf.mod_Vy(:,index) = modules.iekf.Vy';
mCarlo.iekf.noKF_Vy(:,index) = ues.noKF.iekf.Vy';
mCarlo.iekf.KF_Vy(:,index) = ues.KF.iekf.Vy';

mCarlo.iekf.mod_Ay(:,index) = modules.iekf.Ay; 
mCarlo.iekf.noKF_Ay(:,index) = ues.noKF.iekf.Ay;
mCarlo.iekf.KF_Ay(:,index) = ues.KF.iekf.Ay';

mCarlo.iekf.mod_YawR(:,index) = modules.iekf.YawR';
mCarlo.iekf.noKF_YawR(:,index) = ues.noKF.iekf.YawR';
mCarlo.iekf.KF_YawR(:,index) = ues.KF.iekf.YawR';
     
mCarlo.iekf.mod_YawAcc(:,index) = modules.iekf.YawAcc;
mCarlo.iekf.noKF_YawAcc(:,index) = ues.noKF.iekf.YawAcc;
mCarlo.iekf.KF_YawAcc(:,index) = ues.KF.iekf.YawAcc';
     
mCarlo.iekf.mod_Beta(:,index) = modules.iekf.Beta';
mCarlo.iekf.noKF_Beta(:,index) = ues.noKF.iekf.Beta';
mCarlo.iekf.KF_Beta(:,index) = ues.KF.iekf.Beta';
     
mCarlo.iekf.mod_BetaR(:,index) = modules.iekf.Betar;
mCarlo.iekf.noKF_BetaR(:,index) = ues.noKF.iekf.Betar;
mCarlo.iekf.KF_BetaR(:,index) = ues.KF.iekf.BetaR';

%% -------------------- Variable Assignment AIEKF ------------------------
mCarlo.aiekf.mod_Fyf(:,index) = modules.aiekf.Fyf;
mCarlo.aiekf.noKF_Fyf(:,index) = ues.noKF.aiekf.FyF';
mCarlo.aiekf.KF_Fyf(:,index) = ues.KF.aiekf.Fyf';
     
mCarlo.aiekf.mod_Fyr(:,index) = modules.aiekf.Fyr;
mCarlo.aiekf.noKF_Fyr(:,index) = ues.noKF.aiekf.FyR';
mCarlo.aiekf.KF_Fyr(:,index) = ues.KF.aiekf.Fyr';
     
mCarlo.aiekf.mod_Vy(:,index) = modules.aiekf.Vy';
mCarlo.aiekf.noKF_Vy(:,index) = ues.noKF.aiekf.Vy';
mCarlo.aiekf.KF_Vy(:,index) = ues.KF.aiekf.Vy';

mCarlo.aiekf.mod_Ay(:,index) = modules.aiekf.Ay; 
mCarlo.aiekf.noKF_Ay(:,index) = ues.noKF.aiekf.Ay;
mCarlo.aiekf.KF_Ay(:,index) = ues.KF.aiekf.Ay';

mCarlo.aiekf.mod_YawR(:,index) = modules.aiekf.YawR';
mCarlo.aiekf.noKF_YawR(:,index) = ues.noKF.aiekf.YawR';
mCarlo.aiekf.KF_YawR(:,index) = ues.KF.aiekf.YawR';
     
mCarlo.aiekf.mod_YawAcc(:,index) = modules.aiekf.YawAcc;
mCarlo.aiekf.noKF_YawAcc(:,index) = ues.noKF.aiekf.YawAcc;
mCarlo.aiekf.KF_YawAcc(:,index) = ues.KF.aiekf.YawAcc';
     
mCarlo.aiekf.mod_Beta(:,index) = modules.aiekf.Beta';
mCarlo.aiekf.noKF_Beta(:,index) = ues.noKF.aiekf.Beta';
mCarlo.aiekf.KF_Beta(:,index) = ues.KF.aiekf.Beta';
     
mCarlo.aiekf.mod_BetaR(:,index) = modules.aiekf.Betar;
mCarlo.aiekf.noKF_BetaR(:,index) = ues.noKF.aiekf.Betar;
mCarlo.aiekf.KF_BetaR(:,index) = ues.KF.aiekf.BetaR';

%% ---------------------- Variable Assignment UKF ------------------------
mCarlo.ukf.mod_Fyf(:,index) = modules.ukf.Fyf;
mCarlo.ukf.noKF_Fyf(:,index) = ues.noKF.ukf.FyF;
mCarlo.ukf.KF_Fyf(:,index) = ues.KF.ukf.Fyf';
     
mCarlo.ukf.mod_Fyr(:,index) = modules.ukf.Fyr;
mCarlo.ukf.noKF_Fyr(:,index) = ues.noKF.ukf.FyR;
mCarlo.ukf.KF_Fyr(:,index) = ues.KF.ukf.Fyr';
     
mCarlo.ukf.mod_Vy(:,index) = modules.ukf.Vy';
mCarlo.ukf.noKF_Vy(:,index) = ues.noKF.ukf.Vy';
mCarlo.ukf.KF_Vy(:,index) = ues.KF.ukf.Vy';

mCarlo.ukf.mod_Ay(:,index) = modules.ukf.Ay; 
mCarlo.ukf.noKF_Ay(:,index) = ues.noKF.ukf.Ay;
mCarlo.ukf.KF_Ay(:,index) = ues.KF.ukf.Ay';

mCarlo.ukf.mod_YawR(:,index) = modules.ukf.YawR';
mCarlo.ukf.noKF_YawR(:,index) = ues.noKF.ukf.YawR';
mCarlo.ukf.KF_YawR(:,index) = ues.KF.ukf.YawR';
     
mCarlo.ukf.mod_YawAcc(:,index) = modules.ukf.YawAcc;
mCarlo.ukf.noKF_YawAcc(:,index) = ues.noKF.ukf.YawAcc;
mCarlo.ukf.KF_YawAcc(:,index) = ues.KF.ukf.YawAcc';
     
mCarlo.ukf.mod_Beta(:,index) = modules.ukf.Beta';
mCarlo.ukf.noKF_Beta(:,index) = ues.noKF.ukf.Beta';
mCarlo.ukf.KF_Beta(:,index) = ues.KF.ukf.Beta';
     
mCarlo.ukf.mod_BetaR(:,index) = modules.ukf.Betar;
mCarlo.ukf.noKF_BetaR(:,index) = ues.noKF.ukf.Betar;
mCarlo.ukf.KF_BetaR(:,index) = ues.KF.ukf.BetaR';

%% ---------------------- Variable Assignment PF -------------------------
mCarlo.pf.mod_Fyf(:,index) = modules.pf.Fyf;
mCarlo.pf.KF_Fyf(:,index) = ues.KF.pf.Fyf';
     
mCarlo.pf.mod_Fyr(:,index) = modules.pf.Fyr;
mCarlo.pf.KF_Fyr(:,index) = ues.KF.pf.Fyr';
     
mCarlo.pf.mod_Vy(:,index) = modules.pf.Vy';
mCarlo.pf.KF_Vy(:,index) = ues.KF.pf.Vy';

mCarlo.pf.mod_Ay(:,index) = modules.pf.Ay; 
mCarlo.pf.KF_Ay(:,index) = ues.KF.pf.Ay';

mCarlo.pf.mod_YawR(:,index) = modules.pf.YawR';
mCarlo.pf.KF_YawR(:,index) = ues.KF.pf.YawR';
     
mCarlo.pf.mod_YawAcc(:,index) = modules.pf.YawAcc;
mCarlo.pf.KF_YawAcc(:,index) = ues.KF.pf.YawAcc';
     
mCarlo.pf.mod_Beta(:,index) = modules.pf.Beta';
mCarlo.pf.KF_Beta(:,index) = ues.KF.pf.Beta';
     
mCarlo.pf.mod_BetaR(:,index) = modules.pf.Betar;
mCarlo.pf.KF_BetaR(:,index) = ues.KF.pf.BetaR';
