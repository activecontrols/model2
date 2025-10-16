% Test code for actuator modeling responses
clear;

% Initialize time for simulation
res = 5000;
tmax = 5;
dt = tmax / res;
tsim = linspace(0,tmax,res);
norm = [pi/24; pi/20; 14; 15];

% Target step response to max values
start = 100;
utrg = [zeros(4, start) norm .* ones(4, res - start)];
usim = zeros(4,res);
xsim = zeros(6,res);

% Simulation loop
for i = 2:1:res
    xsim(:,i) = xsim(:,i-1) + ActuatorDynamics(xsim(:,i-1), utrg(:, i)) * dt;
    usim(:,i) = [xsim(1,i); xsim(3,i); xsim(5:6,i)];
end

% Plot results
figure;
plot(tsim, usim .* [180/pi; 180/pi; 1; 1]); hold on; grid on;
plot(tsim, utrg .* [180/pi; 180/pi; 1; 1], 'r--');
legend('Angle 1', 'Angle 2', 'Thrust', 'Torque');