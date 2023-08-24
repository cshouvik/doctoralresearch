
function [xEst,Px]= ukfgeneralest(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts)

% TITLE    :  UNSCENTED KALMAN FILTER  
%
% SYNTAX   :  [xEst]=ukf(tune,fstate,constS,inpS,instate,inPx,fmeas,measurment,inpM,constM,R,Q,Ts)
%
% INPUTS   :  - tune             : tuning parameters [slpha beta kappa] 
%             - constS/constM    : any constant term for state and measurement system
%                                  model equation [c1 c2 c3...cn]
%             - inpS/inpM        : vector of control inputs for state snd
%                                  measurement model
%             - measurement      : vector of sensor measurement  
%             - instate          : vector of initial state input at time k 
%             - inPx             : matrix of initial state covarience at time k
%             - R                : process noise covarience
%             - Q                : measurement noise covarience
%             - Ts               : time step (passed to ffun/hfun)   
%             - fstate           : process model function
%             - fmeas            : observation model function  
%
% OUTPUTS  :  - xEst             : updated estimate of state mean at time k+1

    
    
% INITIAL DECLARATION
    Px=inPx;
    UKF=instate;
    mean = (UKF);
    n = length(mean);
    ns=2*n+1;
    
%%Constant Declarations
    alpha=tune(1);
    beta=tune(2);
    kappa=(3);
    lambda = alpha*alpha*(n+kappa)-n;
    
% COMPUTATION OF SIGMA POINTS
    r=chol(((n+lambda).*Px));

    spts=repmat(mean,1,ns)+[ones(n,1) r -r];

% COMPUTATION OF WEIGHTS
    wm=[lambda/(n+lambda) ones(1,ns-1)*0.5/(n+lambda)];

    wc=[wm(1)+(1-alpha^2+beta) ones(1,ns-1)*0.5/(n+lambda)] ;

% TIME UPDATE EQUATIONS
 
    xTranSigma=feval(fstate,Ts,spts,inpS,constS); 

    meanstate=(wm*xTranSigma')';         % Recovery of mean

    Pxx=zeros(n);                       % State Covarience Calculation
    for i=1:ns
        Pxx=Pxx+wc(i).*((xTranSigma(:,i)-meanstate)*(xTranSigma(:,i)-meanstate)');
    end

    Pxx=Pxx+R;                          % Addition of process noise

% RESAMPLING OF SIGMA POINTS
    q=chol(abs((n+lambda).*Pxx));

    respts=repmat(meanstate,1,ns)+[ones(n,1) q -q];
    
% MEASUREMENT UPDATE EQUATIONS
    zTranSigma=feval(fmeas,Ts,respts,inpM,constM);

    meanobs=(wm*zTranSigma')';          % Recovery of mean

    nz=length(zTranSigma);
    Pyy=zeros(n);                       % Output Covarience Calculation
    for i=1:nz
        Pyy=Pyy+wc(i).*((zTranSigma(:,i)-meanobs)*(zTranSigma(:,i)-meanobs)');
    end

    Pyy=Pyy+Q;                          % Addition of Sensor noise

    Pxy=zeros(n);                       % Cross covariance between and State and Output
    for i=1:nz
        Pxy=Pxy+wc(i).*((xTranSigma(:,i)-meanstate)*(zTranSigma(:,i)-meanobs)');
    end

% CALCULATION OF FILTER OUTPUT
    Kt = Pxy/Pyy;                       % Kalman Gain 

    innovation=(measurment-meanobs);    % Innovation

    xEst = meanstate + Kt * innovation; % State Estimate

    sigma = Pxx - Kt * Pyy * Kt';       % Covarience Estimate

    Px=sigma;
end