function GPS = GPS_Sim(Pos, Vel)

% Fake GPS Measurements
GPS = zeros(6,1);
gps_pos_covar = 0.02;
gps_vel_covar = 0.15;
GPS(1:3) = Pos + gps_pos_covar * randn;
GPS(4:6) = Vel + gps_vel_covar * randn;