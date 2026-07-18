function EMA_G = EMA_Gyros_MLFUNC(Y)

% Extract Gyros
persistent lastEMA
if isempty(lastEMA)
    lastEMA = zeros(3,1);
end
gyros = Y(4:6);

% Exponential Moving Avg Step
EMA_G = ExpMovingAvg(gyros, lastEMA, 0.3);
lastEMA = EMA_G;
