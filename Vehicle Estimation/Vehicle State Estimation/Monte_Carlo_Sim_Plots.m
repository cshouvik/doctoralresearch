    subplot(1,3,1)
    plot(Time, rmse.ekfGen_Vy,'g-.')
    hold on
    plot(Time, rmse.ukfGen_Vy,'r-.')
    hold on
    plot(Time, rmse.iekfGen_Vy,'color',[0 0.4470 0.7410],'linestyle',:)
    hold on
    plot(Time, rmse.modEKF_Vy,'b-','linewidth',0.5)
    xlabel('time(sec.)');
    ylabel('RMSE')
    xlim([0,Time(length(Time))])
    legend({'General EKF','General UKF','Paeticle Filter','ModularEKF'},'Orientation','horizontal')
   

    subplot(1,3,2)
    plot(Time, rmse.modEKF_YawR,'g-.')
    hold on
    plot(Time, rmse.ukfGen_YawR,'r-.')
    hold on
    plot(Time, rmse.iekfGen_YawR,'color',[0 0.4470 0.7410],'linestyle',:)
    hold on
    plot(Time, rmse.ekfGen_YawR,'b-','linewidth',0.5)
    xlabel('time(sec.)');
    ylabel('RMSE')
    xlim([0,Time(length(Time))])
   

    subplot(1,3,3)
    plot(Time, rmse.ekfGen_Beta,'g-.')
    hold on
    plot(Time, rmse.ukfGen_Beta,'r-.')
    hold on
    plot(Time, rmse.iekfGen_Beta,'color',[0 0.4470 0.7410],'linestyle',:)
    hold on
    plot(Time, rmse.modEKF_Beta,'b-','linewidth',0.5)
    xlabel('time(sec.)');
    ylabel('RMSE')
    xlim([0,Time(length(Time))])