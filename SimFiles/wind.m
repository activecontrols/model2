function total_vect = wind(mass, wind_vel, ASTRA_vel)

rel_vel = wind_vel + ASTRA_vel;

% Constants needed for force calculation
AREA = 0.07 * 0.8;          % Lateral surface area of ASTRA - m^2
AIR_DENSITY = 1.246656;     % Estimated air desnsity - kg/m^3
Cd = 0.95;                  % Estimated Drag coefficient

% Calculates component accelerations from drag equation and wind velocity
wind_acel = sign(rel_vel) .* (1/2) .* rel_vel.^2 * AREA * AIR_DENSITY * Cd / mass;

% Turning South-West output of function to North-West coordinate system
WtE = [-1, 0; 
        0, 1];
accel_vec = transpose(wind_acel) * WtE;

% Turns the acceleration into a 15 state vector to add to xdot
total_vect = [zeros(6,1); transpose(accel_vec); zeros(7,1)];

end