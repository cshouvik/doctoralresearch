% Iterative Extended Kalman Filter Estimation
function [state_ekf,P_ekf,S,kGain,innovation,pPred,yPred]=iterextendedkalman(n,Ts,pPrev,fstate,state,inpS,constS,JacobX,fmeas,inpM,constM,JacobY,meas,Q,R,JacobW,JacobV)

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
%	n 		  => Number of Iterations

  % Time Update
  xPred = feval(fstate,Ts,state,inpS,constS);              
  pPred = JacobW*Q*JacobW' + JacobX*pPrev*JacobX'; 
  
  %aPriori estimate of Y
               
 
  xIter=xPred;
  pIter=pPred;
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

  state_ekf  = xIter;
  P_ekf = pIter;
   
