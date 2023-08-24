ModelRearFy.Time=Time;
ModelRearFy.Model=FyR;
ModelRearFy.CarSim=Fyr;
outFyr=struct2table(ModelRearFy);

ModelFrontFy.Time=Time;
ModelFrontFy.Model=FyF;
ModelFrontFy.CarSim=Fyf;
outFyf=struct2table(ModelFrontFy);

ModelFrontMy.Time=Time;
ModelFrontMy.Model=MyF;
ModelFrontMy.CarSim=Myf;
outMyf=struct2table(ModelFrontMy);

ModelRearMy.Time=Time;
ModelRearMy.Model=MyR;
ModelRearMy.CarSim=Myr;
outMyr=struct2table(ModelRearMy);


ModelVLateral.Time=Time;
ModelVLateral.Model=VLat;
ModelVLateral.CarSim=Vy;
outVy=struct2table(ModelVLateral);

ModelALateral.Time=Time;
ModelALateral.Model=ALat;
ModelALateral.CarSim=AY;
outAy=struct2table(ModelALateral);

ModelYawRate.Time=Time;
ModelYawRate.Model=RYaw;
ModelYawRate.CarSim=YawR;
outYawR=struct2table(ModelYawRate);

ModelYawAccl.Time=Time;
ModelYawAccl.Model=YAwAcc;
ModelYawAccl.CarSim=YawAcc;
outaYaw=struct2table(ModelYawAccl);

ModelSlipAngle.Time=Time;
ModelSlipAngle.Model=BETA;
ModelSlipAngle.CarSim=Beta;
outBeta=struct2table(ModelSlipAngle);

ModelSlipRate.Time=Time;
ModelSlipRate.Model=Betar;
ModelSlipRate.CarSim=BetaR;
outBetaR=struct2table(ModelSlipRate);

% input.Time=Time;
% input.Vx=Vx;
% input=struct2table(input);

Modelpath.Time=Time;
Modelpath.target=Y_Target;
Modelpath.delta=delta;
path=struct2table(Modelpath);

%% --------------------- Data Write----------------------------------------
DLC_u85_v90_ESCON_path=path;
DLC_u85_v90_ESCON_outBetaR=outBetaR;
DLC_u85_v90_ESCON_outMyf=outMyf;
DLC_u85_v90_ESCON_outMyr=outMyr;
DLC_u85_v90_ESCON_outYawR=outYawR;
DLC_u85_v90_ESCON_outVy=outVy;
DLC_u85_v90_ESCON_outBeta=outBeta;
DLC_u85_v90_ESCON_outFyf=outFyf;
DLC_u85_v90_ESCON_outFyr=outFyr;
DLC_u85_v90_ESCON_outaYaw=outaYaw;
DLC_u85_v90_ESCON_outAy=outAy;

writetable(DLC_u85_v90_ESCON_path)
writetable(DLC_u85_v90_ESCON_outMyf)
writetable(DLC_u85_v90_ESCON_outMyr)
writetable(DLC_u85_v90_ESCON_outYawR)
writetable(DLC_u85_v90_ESCON_outVy)
writetable(DLC_u85_v90_ESCON_outBeta)
writetable(DLC_u85_v90_ESCON_outFyf)
writetable(DLC_u85_v90_ESCON_outFyr)
writetable(DLC_u85_v90_ESCON_outBetaR)
writetable(DLC_u85_v90_ESCON_outaYaw)
writetable(DLC_u85_v90_ESCON_outAy)



% SLALOM_u85_v60_path=path;
% SLALOM_u85_v60_outBetaR=outBetaR;
% SLALOM_u85_v60_outMyf=outMyf;
% SLALOM_u85_v60_outMyr=outMyr;
% SLALOM_u85_v60_outYawR=outYawR;
% SLALOM_u85_v60_outVy=outVy;
% SLALOM_u85_v60_outBeta=outBeta;
% SLALOM_u85_v60_outFyf=outFyf;
% SLALOM_u85_v60_outFyr=outFyr;
% SLALOM_u85_v60_outaYaw=outaYaw;
% SLALOM_u85_v60_outAy=outAy;
% 
% writetable(SLALOM_u85_v60_path)
% writetable(SLALOM_u85_v60_outMyf)
% writetable(SLALOM_u85_v60_outMyr)
% writetable(SLALOM_u85_v60_outYawR)
% writetable(SLALOM_u85_v60_outVy)
% writetable(SLALOM_u85_v60_outBeta)
% writetable(SLALOM_u85_v60_outFyf)
% writetable(SLALOM_u85_v60_outFyr)
% writetable(SLALOM_u85_v60_outBetaR)
% writetable(SLALOM_u85_v60_outaYaw)
% writetable(SLALOM_u85_v60_outAy)


% SLALOM_u03_v60_path=path;
% SLALOM_u03_v60_outBetaR=outBetaR;
% SLALOM_u03_v60_outMyf=outMyf;
% SLALOM_u03_v60_outMyr=outMyr;
% SLALOM_u03_v60_outYawR=outYawR;
% SLALOM_u03_v60_outVy=outVy;
% SLALOM_u03_v60_outBeta=outBeta;
% SLALOM_u03_v60_outFyf=outFyf;
% SLALOM_u03_v60_outFyr=outFyr;
% SLALOM_u03_v60_outaYaw=outaYaw;
% SLALOM_u03_v60_outAy=outAy;
% 
% writetable(SLALOM_u03_v60_path)
% writetable(SLALOM_u03_v60_outMyf)
% writetable(SLALOM_u03_v60_outMyr)
% writetable(SLALOM_u03_v60_outYawR)
% writetable(SLALOM_u03_v60_outVy)
% writetable(SLALOM_u03_v60_outBeta)
% writetable(SLALOM_u03_v60_outFyf)
% writetable(SLALOM_u03_v60_outFyr)
% writetable(SLALOM_u03_v60_outBetaR)
% writetable(SLALOM_u03_v60_outaYaw)
% writetable(SLALOM_u03_v60_outAy)
% 
% 
% 
% 
% DLC_u85_v60_path=path;
% DLC_u85_v60_outBetaR=outBetaR;
% DLC_u85_v60_outMyf=outMyf;
% DLC_u85_v60_outMyr=outMyr;
% DLC_u85_v60_outYawR=outYawR;
% DLC_u85_v60_outVy=outVy;
% DLC_u85_v60_outBeta=outBeta;
% DLC_u85_v60_outFyf=outFyf;
% DLC_u85_v60_outFyr=outFyr;
% DLC_u85_v60_outaYaw=outaYaw;
% DLC_u85_v60_outAy=outAy;
% 
% writetable(DLC_u85_v60_path)
% writetable(DLC_u85_v60_outMyf)
% writetable(DLC_u85_v60_outMyr)
% writetable(DLC_u85_v60_outYawR)
% writetable(DLC_u85_v60_outVy)
% writetable(DLC_u85_v60_outBeta)
% writetable(DLC_u85_v60_outFyf)
% writetable(DLC_u85_v60_outFyr)
% writetable(DLC_u85_v60_outBetaR)
% writetable(DLC_u85_v60_outaYaw)
% writetable(DLC_u85_v60_outAy)
% 

% DLC_u03_v60_path=path;
% DLC_u03_v60_outBetaR=outBetaR;
% DLC_u03_v60_outMyf=outMyf;
% DLC_u03_v60_outMyr=outMyr;
% DLC_u03_v60_outYawR=outYawR;
% DLC_u03_v60_outVy=outVy;
% DLC_u03_v60_outBeta=outBeta;
% DLC_u03_v60_outFyf=outFyf;
% DLC_u03_v60_outFyr=outFyr;
% DLC_u03_v60_outaYaw=outaYaw;
% DLC_u03_v60_outAy=outAy;
% 
% writetable(DLC_u03_v60_path)
% writetable(DLC_u03_v60_outMyf)
% writetable(DLC_u03_v60_outMyr)
% writetable(DLC_u03_v60_outYawR)
% writetable(DLC_u03_v60_outVy)
% writetable(DLC_u03_v60_outBeta)
% writetable(DLC_u03_v60_outFyf)
% writetable(DLC_u03_v60_outFyr)
% writetable(DLC_u03_v60_outBetaR)
% writetable(DLC_u03_v60_outaYaw)
% writetable(DLC_u03_v60_outAy)
% 
% 
% 
% 
% SINE_u85_v60_path=path;
% SINE_u85_v60_outBetaR=outBetaR;
% SINE_u85_v60_outMyf=outMyf;
% SINE_u85_v60_outMyr=outMyr;
% SINE_u85_v60_outYawR=outYawR;
% SINE_u85_v60_outVy=outVy;
% SINE_u85_v60_outBeta=outBeta;
% SINE_u85_v60_outFyf=outFyf;
% SINE_u85_v60_outFyr=outFyr;
% SINE_u85_v60_outaYaw=outaYaw;
% SINE_u85_v60_outAy=outAy;
% 
% writetable(SINE_u85_v60_path)
% writetable(SINE_u85_v60_outMyf)
% writetable(SINE_u85_v60_outMyr)
% writetable(SINE_u85_v60_outYawR)
% writetable(SINE_u85_v60_outVy)
% writetable(SINE_u85_v60_outBeta)
% writetable(SINE_u85_v60_outFyf)
% writetable(SINE_u85_v60_outFyr)
% writetable(SINE_u85_v60_outBetaR)
% writetable(SINE_u85_v60_outaYaw)
% writetable(SINE_u85_v60_outAy)
% % 
% 
% SINE_u03_v60_path=path;
% SINE_u03_v60_outBetaR=outBetaR;
% SINE_u03_v60_outMyf=outMyf;
% SINE_u03_v60_outMyr=outMyr;
% SINE_u03_v60_outYawR=outYawR;
% SINE_u03_v60_outVy=outVy;
% SINE_u03_v60_outBeta=outBeta;
% SINE_u03_v60_outFyf=outFyf;
% SINE_u03_v60_outFyr=outFyr;
% SINE_u03_v60_outaYaw=outaYaw;
% SINE_u03_v60_outAy=outAy;
% 
% writetable(SINE_u03_v60_path)
% writetable(SINE_u03_v60_outMyf)
% writetable(SINE_u03_v60_outMyr)
% writetable(SINE_u03_v60_outYawR)
% writetable(SINE_u03_v60_outVy)
% writetable(SINE_u03_v60_outBeta)
% writetable(SINE_u03_v60_outFyf)
% writetable(SINE_u03_v60_outFyr)
% writetable(SINE_u03_v60_outBetaR)
% writetable(SINE_u03_v60_outaYaw)
% writetable(SINE_u03_v60_outAy)

% 
