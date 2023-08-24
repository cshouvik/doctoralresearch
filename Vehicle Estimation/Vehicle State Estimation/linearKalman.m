function [xEst] = linearKalman(initialState, initialPx, A, B, H, inp, meas, Q, R)
% A => State Matrix
% B => Control Matrix
% H => Output Matrix
% Q => Process Noise Covariance
% R => Measurement Noise Covariance
% inp => Control Inputs
% meas => measuement input

    persistent Px
        if(isempty(Px))
        Px=initialPx;
        end

    persistent xhat
        if(isempty(xhat))
        xhat=initialState;
        end
        
    X = A*xhat + B*inp;
    P = A*Px*A' + Q;
    KGain = P*H'*(H*P*H' + R)^(-1);
    xEst = X + KGain*(meas - H*X);
    KH = KGain*H;
    Pk = (eye(size(KH))-KH)*P;
    
    xhat = xEst;
    Px = Pk;