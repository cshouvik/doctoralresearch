function [rmse,mse,avgErr,pavgErr,pavgAlt] = rmseCalc(estimate,data)

rmse = sqrt(sum((data(:)-estimate(:)).^2)/numel(data));
mse = sum((data(:)-estimate(:)).^2)/numel(data);
avgErr = sum(abs(data(:)-estimate(:)))/numel(data);

x = abs((data(:)-estimate(:))./(data(:)));
x = x(~isnan(x));
x = x(~isinf(x));
pavgErr = (sum(x)/numel(data))*100;