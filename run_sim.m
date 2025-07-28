% This is the main runtime function

% Allows MATLAB to use functions across local file infrastructure
addpath('./params/');
addpath('./sim/');
addpath('./sim/lib');
addpath('./trajectory/');

% open_system('genSym');

%% Constants
constants = constructConstants; % Creates constant structure

%% Plant
[x, u, x_dot] = EoMGenerator(constants);
[linSys, disLinSys, plantFn, outFn] = dynamics(x, u, x_dot, constants);
x0 = zeros(12,1);

%% Sensor Modeling


%% Estimator/Observer


%% Controller

% Brysons Rule for Q and R
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [100, 50, 50, 5, 5, 5, 1, 1, 1, 5, 5, 5];
max_u = [pi/24, pi/24, 1500, 20];

Q = eye(size(linSys.A,1)) .* a_weights ./ max_x.^2;
R = eye(size(linSys.B,2)) .* b_weights ./ max_u.^2;

[K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);