A  = [1 1; 0 1;];
B = [0.5;1];
initialState = [95;1];
initialPx = [10 0; 0 1];
H = [1 0];
inp = -9.8;
meas = [0; 100; 97.9;94.4;92.7;87.3];
Q = [0.001 0; 0 0.0001];
R = 0.1;

xEst(1,:) = initialState;  
for i = 2:6
xEst(i,:) = linearKalman(initialState, initialPx, A, B, H, inp, meas(i), Q, R);
end

%     i = 1;
%     X = A*initialState + B*inp;
%     P = A*initialPx*A' + Q;
%     KGain = P*H'*(H*P*H' + R)^(-1);
%     xEst = X + KGain*(meas(i) - H*X);
%     KH = KGain*H;
%     Pk = (eye(size(KH))-KH)*P;
%     i = i+1;
%     
%     initialState = xEst;
%     initialPx = Pk;
%     
%     xEst