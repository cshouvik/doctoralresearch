%% Iterative Extended Kalman Filter Estimation
function [state_ekf,P_ekf,adQ,adR]=adaptiveiterextendedkalman(n,Ts,pPrev,fstate,state,inpS,constS,JacobX,fmeas,inpM,constM,JacobY,meas,Q,R,JacobW,JacobV)

%    DESCRIPTION:
%	 System Equations:
%    x(k)= f(x(k-1),u(k),w(k));
%    z(k)= h(x(k),u(k),w(k));
%    
%   Ts        => Sampling Interval
%   pPrev     => Px at previous time instant
%   fstate    => State Function
%   state     => Previous state estimates
%   inpS      => Input to the state Functions
%   constS    => Constants in state function
%   JacobX    => Jacobian of f(x(k-1),u(k),w(k)) wrt x
%   fmeas     => Measurement Function
%   inpM      => Input to the measurement function
%   constM    => Constants in measurement function
%   JacobY    => Jacobian of h(x(k),u(k),w(k)) wrt x
%   meas      => Measurement
%   Q,R       => Process and Measurement noise covariences
%   JacobW    => Jacobian of f(x(k-1),u(k),w(k)) wrt w
%   Jacobv    => Jacobian of h(x(k),u(k),w(k)) wrt v
%	  n 		    => Number of Iterations
%   b         => Scaling factor for "d" (value: 0<b<1)
%   d         => weight for adaptive estimator

  % Time Update
  xPred = feval(fstate,Ts,state,inpS,constS);              
  pPred = JacobW*Q*JacobW' + JacobX*pPrev*JacobX'; 
  
  %Initialization for adative noise covarience "R"         

  innovation=[];
  kGain=[];

  xIter = xPred;
  pIter = pPred;
 	for i=1:n
  		%Measurement Update
  		S  = JacobV*R*JacobV' + JacobY*pIter*JacobY';	%In this case jacobian is Constant
  		kGain = (pIter*JacobY')*inv(S);
  
  		%Filter Outputs
  		yPred = feval(fmeas,Ts,xIter,inpM,constM);
  		innovation = (meas-yPred);
  		
  		xIter = xIter+kGain*(innovation-JacobY*(xPred-xIter));
  		pIter = pIter-kGain*JacobY*pIter;
  end 

  kGain = kGain(n);
  Ce = (1/n)*innovation*innovation';

  % Main Filter Outputs

  state_ekf  = xIter;  % State
  P_ekf = pIter;       % Covarience

  adR = Ce-JacobY*P_ekf*JacobY';  % Adaptive R 
  adQ = Q;          % Adaptive Q

   
