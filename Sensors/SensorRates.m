function z = SensorRates(Accel, Gyros, Mag, GPS, constantsASTRA, t)

% Simulates magnetometer, accelerometer, and gyros given the true state and
% a covariance vector. No biases for now.

% Update rates
IMU_Rate = 1000;     %Hz
GPS_Rate = 15;      %Hz
z = zeros(15,1);

persistent lastGPS lastIMU lastZ

if isempty(lastGPS)
    lastGPS = 0;  % Update last GPS timestamp
    lastIMU = 0;  % Update last IMU timestamp
    lastZ = [0; 0; 9.81; zeros(3,1); constantsASTRA.mag; zeros(6,1)];
end

% Fake IMU measurements
if t - lastIMU > 1 / IMU_Rate
    z(1:3) = Accel;
    z(4:6) = Gyros;
    z(7:9) = Mag;
    lastIMU = t;
else
    z(1:9) = lastZ(1:9);
end

% Fake GPS measurements
if t - lastGPS > 1 / GPS_Rate
    z(10:15) = GPS;
    lastGPS = t;
else
    z(10:15) = lastZ(10:15);
end
lastZ = z;