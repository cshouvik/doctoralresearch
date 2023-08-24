% clear all
% uiopen
% Linear fit for Cf
% Linear model Poly1:
%      f(x) = p1*x + p2
%        where x is normalized by mean 0.2499 and std 5255
% Coefficients (with 95% confidence bounds):
%        p1 =       192.7  (-58.63, 444.1)
%        p2 =       -1179  (-1430, -928.2)
% 
% Goodness of fit:
%   SSE: 1.666e+09
%   R-square: 0.007084
%   Adjusted R-square: 0.003971
%   RMSE: 2286

Cf=Fyf./-Alphaf;

[xData, yData] = prepareCurveData( Fyf, Cf );

% Set up fittype and options.
ft = fittype( 'poly3' ); %%%In case of issue, replace by 'poly3'

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, 'Normalize', 'on' );

% Plot fit with data.
% figure( 'Name', 'Cf & Fyf Curve Fitting' );
% h = plot( fitresult, xData, yData );
% legend( h, 'Cf vs. Fyf', 'untitled fit 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% % Label axes
% xlabel( 'Fyf', 'Interpreter', 'none' );
% ylabel( 'Cf', 'Interpreter', 'none' );
% grid on

% Generate fitted Cf Data
Cf_fit=feval(fitresult,Fyf);

% figure( 'Name', 'Fitted Cf' );
% h = plot( Time,Cf_fit );
% xlabel( 'Time', 'Interpreter', 'none' );
% ylabel( 'Cf', 'Interpreter', 'none' );
% grid on
% 
% figure(1)
% subplot(1,2,1)
% plot( fitresult, xData, yData );
% grid on
%%
% Linear fit for Cr
%      f(x) = a*(sin(x-pi)) + b*((x-10)^2) + c
% Coefficients (with 95% confidence bounds):
%        a =        1032  (107.9, 1955)
%        b =       12.94  (-9.932, 35.82)
%        c =       -1352  (-2456, -247.3)
% 
% Goodness of fit:
%   SSE: 1.004e+10
%   R-square: 0.01568
%   Adjusted R-square: 0.009493
%   RMSE: 5618


Cr=Fyr./-Alphar;
[xData, yData] = prepareCurveData( Fyr, Cr );

% Set up fittype and options.
ft = fittype( 'poly3' );

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, 'Normalize', 'on' );

% Plot fit with data.
% figure( 'Name', 'Cr & Fyr Curve Fitting' );
% h = plot( fitresult, xData, yData );
% legend( h, 'Cr vs. Fyr', 'untitled fit 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% % Label axes
% xlabel( 'Fyf', 'Interpreter', 'none' );
% ylabel( 'Cr', 'Interpreter', 'none' );
% grid on

% Generate fitted Cf Data
Cr_fit=feval(fitresult,Fyr);

% figure( 'Name', 'Fitted Cr' );
% h = plot( Time,Cr_fit );
% xlabel( 'Time', 'Interpreter', 'none' );
% ylabel( 'Cr', 'Interpreter', 'none' );
% grid on
% 
% figure(1)
% subplot(1,2,2)
% plot( fitresult, xData, yData );
% grid on

