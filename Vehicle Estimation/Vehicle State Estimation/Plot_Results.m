    subplot(2,3,1)
    plot(Time, delta, 'k-','linewidth',1)
    ylabel('Steering Angle');
    xlabel('Time');
    xlim([0,Time(length(Time))])

    subplot(2,3,2)
    plot(Time, Fyr,'k-','linewidth',1)
    hold on
    plot(Time, [0 diff(ekfGen.estimate(1,:))/Time(2)],'g-.')
    hold on
    plot(Time, [0 diff(ukfGen.estimate(1,:))/Time(2)],'r-.')
    hold on
    plot(Time, [0 diff(iekfGen.estimate(1,:))/Time(2)],'color',[0 0.4470 0.7410],'linestyle',:,'linewidth',1)
    hold on
    plot(Time, [0 diff(modEKF.Myr.ekf)/Time(2)],'b-','linewidth',0.5)
    xlabel('time(sec.)');
    ylabel('Rear Tire Force(N)')
    legend({'CarSim','General EKF','General UKF','Particle Filter','Modular EFK'},'Orientation','horizontal')
    xlim([0,Time(length(Time))])


    subplot(2,3,3)
    plot(Time, Fyf,'k-','linewidth',1)
    hold on
    plot(Time, [0 diff(ekfGen.estimate(2,:))/Time(2)],'g-.')
    hold on
    plot(Time, [0 diff(ukfGen.estimate(2,:))/Time(2)],'r-.')
    hold on
    plot(Time, [0 diff(iekfGen.estimate(2,:))/Time(2)],'color',[0 0.4470 0.7410],'linestyle',:,'linewidth',1)
    hold on
    plot(Time, [0 diff(modEKF.Myf.ekf)/Time(2)],'b-','linewidth',0.5)
    xlabel('time(sec.)');
    ylabel('Front Tire Force(N)')
    xlim([0,Time(length(Time))])
    

    subplot(2,3,4)
    plot(Time, Vy,'k-','linewidth',1)
    hold on
    plot(Time, ekfGen.estimate(3,:),'g-.')
    hold on
    plot(Time, ukfGen.estimate(3,:),'r-.')
    hold on
    plot(Time, iekfGen.estimate(3,:),'color',[0 0.4470 0.7410],'linestyle',:)
    hold on
    plot(Time, modEKF.Vy.ekf,'b-','linewidth',0.5)
    xlabel('time(sec.)');
    ylabel('Lateral Velocity(m/s)')
    xlim([0,Time(length(Time))])
   

    subplot(2,3,5)
    plot(Time, YawR,'k-','linewidth',1)
    hold on
    plot(Time, modEKF.rYaw.ekf,'g-.')
    hold on
    plot(Time, ukfGen.estimate(4,:),'r-.')
    hold on
    plot(Time, iekfGen.estimate(4,:),'color',[0 0.4470 0.7410],'linestyle',:)
    hold on
    plot(Time, ekfGen.estimate(4,:),'b-','linewidth',0.5)
    xlabel('time(sec.)');
    ylabel('Yaw Rate(rad/s)')
    xlim([0,Time(length(Time))])
   

    subplot(2,3,6)
    plot(Time, Beta,'k-','linewidth',1)
    hold on
    plot(Time, ekfGen.estimate(5,:),'g-.')
    hold on
    plot(Time, ukfGen.estimate(5,:),'r-.')
    hold on
    plot(Time, iekfGen.estimate(5,:),'color',[0 0.4470 0.7410],'linestyle',:)
    hold on
    plot(Time, modEKF.slip.ekf,'b-','linewidth',0.5)
    xlabel('time(sec.)');
    ylabel('Slip Angle(rad)')
    xlim([0,Time(length(Time))])
  