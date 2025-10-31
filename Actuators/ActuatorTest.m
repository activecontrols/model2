% Test code for actuator modeling responses
clear;
close all;

% Initialize time for simulation
res = 5000;
tmax = 10;
dt = tmax / res;
tsim = linspace(0,tmax,res);
norm = [pi/24; pi/24; 14; 15];

% Target step response to max values
start = 100;
utrg = [zeros(4, start), norm .* ones(4, res - start)];
usim = zeros(4,res);
xsim = zeros(6,res);

% Initialize data tables
theta_target = zeros(1, res);
phi_target = zeros(1, res);
theta_actual = zeros(1, res);
phi_actual = zeros(1, res);

% Simulation loop
for i = 2:1:res
    theta_target(i) = norm(1) * cos(tsim(i) * 1);
    phi_target(i) = norm(2) * sin(tsim(i) * 1);
    utrg(1:2, i) = [theta_target(i); phi_target(i)];
    
    xsim(:,i) = xsim(:,i-1) + ActuatorDynamics(xsim(:,i-1), utrg(:, i)) * dt;
    usim(:,i) = [xsim(1,i); xsim(3,i); xsim(5:6,i)];

    theta_actual(:, i) = xsim(1, i);
    phi_actual(:, i) = xsim(3, i);
end

% Plot results
figure;
plot(tsim, usim .* [180/pi; 180/pi; 1; 1]); hold on; grid on;
plot(tsim, utrg .* [180/pi; 180/pi; 1; 1], 'r--');
% plot(tsim, (utrg - usim) .* [180/pi; 180/pi; 1; 1]);
legend('Angle 1', 'Angle 2', 'Thrust', 'Torque');

figure;
subplot(2, 1, 1)
plot(theta_target, theta_actual); grid on;
title("Theta Target vs Actual")

subplot(2, 1, 2)
plot(phi_target, phi_actual); grid on;
title("Phi Target vs Actual")

figure;
plot(xsim(1,:), xsim(3,:), "b*");
axis equal