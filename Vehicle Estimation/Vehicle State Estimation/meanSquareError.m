IEKF_MAE_Myf = mean(abs(iekfGen.estimate(1,:)-Myf'));
IEKF_MAE_Myr = mean(abs(iekfGen.estimate(2,:)-Myr'));
IEKF_MAE_Fyf = mean(abs([0 diff(iekfGen.estimate(1,:))]/Time(2)-Fyf'));
IEKF_MAE_Fyr = mean(abs([0 diff(iekfGen.estimate(2,:))]/Time(2)-Fyr'));
IEKF_MAE_Vy = mean(abs(iekfGen.estimate(3,:)-Vy'));
IEKF_MAE_YawR = mean(abs(iekfGen.estimate(4,:)-YawR'));
IEKF_MAE_Slip = mean(abs(iekfGen.estimate(5,:)-Beta'));

EKF_MAE_Myf = mean(abs(ekfGen.estimate(1,:)-Myf'));
EKF_MAE_Myr = mean(abs(ekfGen.estimate(2,:)-Myr'));
EKF_MAE_Fyf = mean(abs([0 diff(ekfGen.estimate(1,:))]/Time(2)-Fyf'));
EKF_MAE_Fyr = mean(abs([0 diff(ekfGen.estimate(2,:))]/Time(2)-Fyr'));
EKF_MAE_Vy = mean(abs(ekfGen.estimate(3,:)-Vy'));
EKF_MAE_YawR = mean(abs(ekfGen.estimate(4,:)-YawR'));
EKF_MAE_Slip = mean(abs(ekfGen.estimate(5,:)-Beta'));

UKF_MAE_Myf = mean(abs(ukfGen.estimate(1,:)-Myf'));
UKF_MAE_Myr = mean(abs(ukfGen.estimate(2,:)-Myr'));
UKF_MAE_Fyf = mean(abs([0 diff(ukfGen.estimate(1,:))]/Time(2)-Fyf'));
UKF_MAE_Fyr = mean(abs([0 diff(ukfGen.estimate(2,:))]/Time(2)-Fyr'));
UKF_MAE_Vy = mean(abs(ukfGen.estimate(3,:)-Vy'));
UKF_MAE_YawR = mean(abs(ukfGen.estimate(4,:)-YawR'));
UKF_MAE_Slip = mean(abs(ukfGen.estimate(5,:)-Beta'));

MODEKF_MAE_Myf = mean(abs(modEKF.Myf.ekf-Myf'));
MODEKF_MAE_Myr = mean(abs(modEKF.Myr.ekf-Myr'));
MODEKF_MAE_Fyf = mean(abs([0 diff(modEKF.Myf.ekf)]/Time(2)-Fyf'));
MODEKF_MAE_Fyr = mean(abs([0 diff(modEKF.Myf.ekf)]/Time(2)-Fyr'));
MODEKF_MAE_Vy = mean(abs(modEKF.Vy.ekf-Vy'));
MODEKF_MAE_YawR = mean(abs(modEKF.rYaw.ekf-YawR'));
MODEKF_MAE_Slip = mean(abs(modEKF.slip.ekf-Beta'));