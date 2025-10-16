function F = StateTransitionMat(accel, gyro, R_b2i, J)


% Rederive angular rate dynamics using bias.
F = zeros(15);
F(1:3,1:3) = -zetaCross(gyro);
F(1:3,13:15) = -eye(3);
F(4:6,7:9) = eye(3);
F(7:9,1:3) = -R_b2i * zetaCross(accel);
F(10:12,10:12) = -J^(-1)*(zetaCross(gyro)*J - zetaCross(J*gyro));

