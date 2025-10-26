function F = StateTransitionMat(accel, gyro, R_b2i)

% Remove angular rates from error-state. Make safety copies of all relevant
% files into Archive
F = zeros(15);
F(1:3,1:3) = -zetaCross(gyro);
F(1:3,10:12) = -eye(3);
F(4:6,7:9) = eye(3);
F(7:9,1:3) = -R_b2i * zetaCross(accel);
F(7:9,13:15) = -R_b2i;

