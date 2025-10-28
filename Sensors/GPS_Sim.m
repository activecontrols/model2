function GPS = GPS_Sim(Pos, t)

% Integrate rate limiting directly into GPS measurements instead of through
% SensorRates function. Keep Sensor Rates for IMU only.

persistent error_pos lastT lastPos
if isempty(error_pos)
    error_pos = 0;
    lastT = 0;
    lastPos = zeros(3,1);
end

dT = t - lastT;
lastT = t;

% Correlated noise measurements (GMP)
gps_pos_covar = 0.075;
decay = 10;
beta = exp(-dT / decay);
error_pos = beta * error_pos + sqrt(1 - beta^2) * gps_pos_covar * randn;

% Fake GPS Measurements (derivate velocity from GPS_Pos)
GPS = zeros(6,1);
GPS(1:3) = Pos + error_pos;
GPS(4:6) = (GPS(1:3) - lastPos) / dT;
lastPos = GPS(1:3);