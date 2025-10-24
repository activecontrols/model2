function total_vect = wind(constantsASTRA, wind_vel, X)

ASTRA_vel = X(7:9); % Lateral velocities
% Quaternion components
q1 = X(1);
q2 = X(2);
q3 = X(3);
q0 = sqrt(1 - q1^2 - q2^2 - q3^2);

mass = constantsASTRA.m;
MoI = constantsASTRA.J;
moment = [0, 0, 0.120] * quatRot([q0, q1, q2, q3]); % Distance from CoM to CoP rotated by Earth to Body DCM

rel_vel = wind_vel + ASTRA_vel;

% Constants needed for force calculation
AREA = 0.07 * 0.8;          % Lateral surface area of ASTRA - m^2
AIR_DENSITY = 1.246656;     % Estimated air desnsity - kg/m^3
Cd = 0.95;                  % Estimated Drag coefficient

% Calculates component accelerations from drag equation and wind velocity
wind_force = sign(rel_vel) .* (1/2) .* rel_vel.^2 * AREA * AIR_DENSITY * Cd;


% Turning South-West Earth coordinate output of function to North-West Earth coordinate system
WtE = [-1, 0, 0; 
        0, 1, 0;
        0, 0, 0];
force_vect = transpose(wind_force) * WtE;

accel_vect = force_vect ./ mass;
torque_vect = cross(force_vect, moment);
ang_accel_vect = MoI\transpose(torque_vect);


% Turns the acceleration into a 15 state vector to add to xdot
total_vect = [zeros(6,1); transpose(accel_vect); ang_accel_vect; zeros(3,1)];

end