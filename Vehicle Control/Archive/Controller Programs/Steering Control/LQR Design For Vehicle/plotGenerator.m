%% Simulink file LQR_Lateral_Stability
subplot(1,3,1)
plot(simTime,ControlEffort)
xlabel('Time(in sec)')
ylabel('Steering Angle(in rad)')

subplot(1,3,2)
plot(simTime,yawOut,'b-',simTime,yawRef,'r--')
xlabel('Time(in sec)')
ylabel('Yaw rate(in rad/s)')
title({'Yaw rate Output'})

subplot(1,3,3)
plot(simTime,slipOut,'b-',simTime,slipRef,'r--')
xlabel('Time(in sec)')
ylabel( 'Slip Angle(in rad)')
ylabel('Slip Angle(in rad)');
%% Simulink model Linear_SS_model_validation
subplot(1,2,1)
plot(simTime,yawOut,'b-',simTime,yawRef,'r--')
xlabel('Time(in sec)')
ylabel('Yaw rate(in rad/s)')

subplot(1,2,2)
plot(simTime,slipOut,'b-',simTime,slipRef,'r--')
xlabel('Time(in sec)')
ylabel( 'Slip Angle(in rad)')