% This is the main runtime function

% Allows MATLAB to use functions across local file infrastructure
addpath('./params/');
addpath('./sim/');
addpath('./trajectory/');

open_system('genSym');

%% Constants
constants = constructConstants; % Creates constant structure

%% Plant
[x, u, x_dot] = EoMGenerator(constants);

[linSys, disLinSys, plantFn, outFn] = dynamics(x, u, x_dot, constants);

%% Sensor Modeling


%% Estimator/Observer


%% Controller