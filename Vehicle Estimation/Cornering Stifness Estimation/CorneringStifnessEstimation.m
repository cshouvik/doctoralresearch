% Reference:
% Cornering stiffness estimation based on vehicle lateral dynamics
% C. Sierra a , E. Tseng b , A. Jain a & H. Peng a

%% Direct Method
Cf_direct=zeros(321,1);
Cr_direct=zeros(321,1);
for i=2:length(Time)
    if abs(Alphaf(i))*1000<5
        Cf_direct(i)=0;%Cf_direct(i-1);
    else
        Cf_direct(i)=Fyf(i)/Alphaf(i);
    end
    
     if abs(Alphar(i))*1000<2
        Cr_direct(i,1)=Cr_direct(i-1);
    else
        Cr_direct(i,1)=Fyr(i)/Alphar(i);
    end
    
end

%% Direct Method
Cf_direct2=zeros(321,1);
Cr_direct2=zeros(321,1);
for i=2:length(Time)
    if abs(Alphaf(i))<5
        Cf_direct2(i)=Cf_direct2(i-1);
        nAlphaf(i)=Alphaf(i-1);
        nFyf(i)=Fyf(i-1);
    else
        Cf_direct2(i)=Fyf(i)/Alphaf(i);
        nAlphaf(i)=Alphaf(i);
        nFyf(i)=Fyf(i);
    end
    
     if abs(Alphar(i))<2
        Cr_direct2(i,1)=Cr_direct2(i-1);
    else
        Cr_direct2(i,1)=Fyr(i)/Alphar(i);
     end
    
end
%% Ay Method Eq. 5
for i=1:length(Time)
X1=[delta(i)-((VY(i)+lf*YawR(i))/Vx(i)) -((VY(i)-lf*YawR(i))/Vx(i))];

Y1=m*AY(i);

CStiff_Ay(:,i)=inv(X1'*X1)*X1'*Y1;
Cf_Ay(i,1)=CStiff_Ay(1,i);
Cr_Ay(i,1)=CStiff_Ay(2,i);
end

%% Yaw Acc Method Eq. 7
for i=1:length(Time)
X2=[lf*(delta(i)-((VY(i)+lf*YawR(i))/Vx(i))) -lr*(-((VY(i)-lf*YawR(i))/Vx(i)))];

Y2=Iz*YawAcc(i);

CStiff_YawAcc(:,i)=inv(X2'*X2)*X2'*Y2;
Cf_YawAcc(i)=CStiff_YawAcc(1,i);
Cr_YawAcc(i)=CStiff_YawAcc(2,i);
end

%% Beta less method Eq. 11 
for i=1:length(Time)
X3=[m*l*AY(i) (delta(i)-l*YawR(i)/Vx(i))];

Y3=Iz*YawAcc(i)+m*lr*AY(i);

X=inv(X3'*X3)*X3'*Y3;
Cf_Beta(i)=X(2)/X(1);
Cr_Beta(i)=X(1)/(1-X(1));
end

%% Beta less method Eq. 14 
for i=1:length(Time)
X4=[Fyr(i) (delta(i)-l*YawR(i)/Vx(i))];

Y4=Fyf(i);

Xn=inv(X4'*X4)*X4'*Y4;
Cf_Beta2(i)=Xn(2);
Cr_Beta2(i)=Xn(2)/Xn(1);
end

subplot(6,2,1)
plot(Time,Cf_direct)
title('Cf (Direct Method)')
xlabel('time (secs)')
ylabel('Cf')

subplot(6,2,2)
plot(Time,Cr_direct)
title('Cr (Direct Method)')
xlabel('time (secs)')
ylabel('Cr')

subplot(6,2,3)
plot(Time,Cf_direct2)
title('Cf (Direct Method 2)')
xlabel('time (secs)')
ylabel('Cf')

subplot(6,2,4)
plot(Time,Cf_direct2)
title('Cr (Direct Method 2)')
xlabel('time (secs)')
ylabel('Cr')

subplot(6,2,5)
plot(Time,Cf_Ay)
title('Cf (Ay Method)')
xlabel('time (secs)')
ylabel('Cf')

subplot(6,2,6)
plot(Time,Cr_Ay)
title('Cr (Ay Method)')
xlabel('time (secs)')
ylabel('Cr')

subplot(6,2,7)
plot(Time,Cf_YawAcc)
title('Cf (Yaw Acc. Method)')
xlabel('time (secs)')
ylabel('Cf')

subplot(6,2,8)
plot(Time,Cr_YawAcc)
title('Cr (Yaw Acc. Method)')
xlabel('time (secs)')
ylabel('Cr')

subplot(6,2,9)
plot(Time,Cf_Beta)
title('Cf (Beta Method)')
xlabel('time (secs)')
ylabel('Cf')

subplot(6,2,10)
plot(Time,Cr_Beta)
title('Cr (Beta Method)')
xlabel('time (secs)')
ylabel('Cr')

subplot(6,2,11)
plot(Time,Cf_Beta2)
title('Cf (Beta Method 2)')
xlabel('time (secs)')
ylabel('Cf')

subplot(6,2,12)
plot(Time,Cr_Beta2)
title('Cr (Beta Method 2)')
xlabel('time (secs)')
ylabel('Cr')