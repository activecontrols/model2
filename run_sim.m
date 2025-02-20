% This is the main runtime function

% Allows MATLAB to use functions across local file infrastructure
addpath('./sim/');
addpath('./trajectory/');

%% Constants
%consts.

%% Plant
[x, u, x_dot] = EoMGenerator;

[linSys, disLinSys, plantFn, outFn] = dynamics(x, u, x_dot);

%% Sensor Modeling


%% Estimator/Observer


%% Controller