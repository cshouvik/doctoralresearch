clear all
uiopen
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
[fitresult_Cf, gof_Cf] = fit( xData, yData, ft, 'Normalize', 'on' );

% % Plot fit with data.
% figure( 'Name', 'Cf & Fyf Curve Fitting' );
% h = plot( fitresult, xData, yData );
% legend( h, 'Cf vs. Fyf', 'untitled fit 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% % Label axes
% xlabel( 'Fyf', 'Interpreter', 'none' );
% ylabel( 'Cf', 'Interpreter', 'none' );
% grid on

% Generate fitted Cf Data
fit_Cf=feval(fitresult_Cf,Fyf);
%%
% figure( 'Name', 'Fitted Cf' );
% h = plot( Time,Cf.fit );
% xlabel( 'Time', 'Interpreter', 'none' );
% ylabel( 'Cf', 'Interpreter', 'none' );
% grid on

% figure(1)
% subplot(1,2,1)
% plot( fitresult_Cf, xData, yData );
% % xlabel( 'F_yf (in Nm)', 'Interpreter', 'none' );
% % ylabel( 'Cf', 'Interpreter', 'none' );
% grid on
%% Coefficient, Statistics and error for Cf
s_Cf = struct(fitresult_Cf);
s_Cf.pValues = cell2mat(s_Cf.coeffValues);
err_Cf = [s_Cf.meanx, s_Cf.stdx, s_Cf.pValues(1), s_Cf.pValues(2),...
    s_Cf.pValues(3), s_Cf.pValues(4), gof_Cf.rmse, gof_Cf.rsquare,...
    gof_Cf.adjrsquare, gof_Cf.sse];
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
[fitresult_Cr, gof_Cr] = fit( xData, yData, ft, 'Normalize', 'on' );

% % Plot fit with data.
% figure( 'Name', 'Cr & Fyr Curve Fitting' );
% h = plot( fitresult, xData, yData );
% legend( h, 'Cr vs. Fyr', 'untitled fit 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% % Label axes
% xlabel( 'Fyf', 'Interpreter', 'none' );
% ylabel( 'Cr', 'Interpreter', 'none' );
% %grid on

% Generate fitted Cf Data
fit=feval(fitresult_Cr,Fyr);
%%
% figure( 'Name', 'Fitted Cr' );
% h = plot( Time,Cr.fit );
% xlabel( 'Time', 'Interpreter', 'none' );
% ylabel( 'Cr', 'Interpreter', 'none' );
% %grid on

% figure(1)
% subplot(1,2,2)
% plot( fitresult.Cr, xData, yData );
% % xlabel( 'F_yr (in Nm)', 'Interpreter', 'none' );
% % ylabel( 'Cr', 'Interpreter', 'none' );
% grid on
s_Cr = struct(fitresult_Cr);
s_Cr.pValues = cell2mat(s_Cr.coeffValues);
err_Cr = [s_Cr.meanx, s_Cr.stdx, s_Cr.pValues(1), s_Cr.pValues(2),...
    s_Cr.pValues(3), s_Cr.pValues(4), gof_Cr.rmse, gof_Cr.rsquare,...
    gof_Cr.adjrsquare, gof_Cr.sse];


errTot = [err_Cf, err_Cr];
