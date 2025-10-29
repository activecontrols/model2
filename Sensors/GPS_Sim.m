function GPS = GPS_Sim(Pos, t)

% Integrate rate limiting directly into GPS measurements instead of through
% SensorRates function. Keep Sensor Rates for IMU only.

GPS_Rate = 15;      %Hz
persistent error_pos lastGPS lastTime
if isempty(error_pos)
    error_pos = 0;
    lastGPS = zeros(6,1);
    lastTime = 0;  % Update last GPS timestamp
end

dT = t - lastTime;

if t - lastTime > 1 / GPS_Rate
    % Correlated noise measurements (GMP)
    gps_pos_covar = 0.1;
    decay = 15;
    beta = exp(-dT / decay);
    error_pos = beta * error_pos + sqrt(1 - beta^2) * gps_pos_covar * randn;
    
    % Fake GPS Measurements (derivate velocity from GPS_Pos)
    GPS = zeros(6,1);
    GPS(1:3) = Pos + error_pos;
    GPS(4:6) = (GPS(1:3) - lastGPS(1:3)) / dT;
    lastGPS = GPS;
    lastTime = t;
else
    GPS = lastGPS;
end
