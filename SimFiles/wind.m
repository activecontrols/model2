function final_xdot = wind(xdot, wind_vel)

% Constants needed for force calculation
AREA = 0.07; % Lateral surface are of ASTRA - m/s
AIR_DENSITY = 1.246656; % Estimated air desnsity - kg/m^3
% (plan to calculate more accuraately with sensor information like temperature)
mass = 0.9950; % ASTRA mass (couldn't figure out how to get it from constantsASTRA) - kg
Cd = 0.9; % Estimated Drag coefficient

% Calculates component accelerations from drag equation and wind velocity
wind_acel = (1/2) * wind_vel.^2 * AREA * AIR_DENSITY * Cd / mass;

% Turning South-West output of function to North-West coordinate system
WtE = [-1, 0; 
        0, 1];
acel_vect = transpose(wind_acel) * WtE;

% Turns the acceleration into a 15 state vector to add to xdot
total_vect = [zeros(6,1); transpose(acel_vect); zeros(7,1)];
final_xdot = xdot + total_vect;

end