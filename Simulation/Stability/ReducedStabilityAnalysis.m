function StateSpace = ActuatorDelay
    %Creates a first order actuator model
    ActuatorModel = cell(3, 1);
    tau = [0.08; 0.08; 0.15];

    for i =1:size(tau, 1)
        tau_i = tau(i);
        ActuatorModel{i} = tf(1, [tau_i, 1]);
    end

    % Assemble actuator models
    Delay_MIMO = blkdiag(ActuatorModel{:});
    StateSpace = ss(Delay_MIMO);
end

function [DM, MM] = evalDiskMarginReduced(Q, R, linSys, constantsASTRA, thrustMax)
    [K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);
    persistent count
    if isempty(count)
        count = 0;
    end

    % First system linearization
    x0 = zeros(9,1);
    u0 = [0; 0; 0];
    A = linSys.A;
    B = linSys.B;
    C = eye(size(linSys.A, 1));
    D = zeros(size(linSys.A, 1), size(linSys.B, 2));
    
    % Plant TF
    P = ss(A, B, C, D);
    
    % Controller TF (set only once)
    K_ss = ss(K);
    
    % Feedback TF
    L = K_ss * P;
    
    % Delayed Feedback TF
    Delay_MIMO_ss = ActuatorDelay;
    L = L * Delay_MIMO_ss;
    
    % Digital Filter TF (Using the transfer function of the worst-case digital
    % filter we'll have on board)
    thrust = 80;
    [Filter_TF, ~] = FilterTF_Gen(thrust);
    Filter_ss = ss(Filter_TF);
    L = L * Filter_ss;
    
    % Final Disk Margin structs.
    [DM, MM] = diskmargin(L);
    fprintf('Disk Margin Analysized! Counter:  %i\n', count);
    count = count + 1;
end

clear;
clear evalDiskMarginReduced;
addpath('.\Filtering');
addpath('.\Filtering\ANF');
addpath('.\Simulation');
addpath('.\Simulation\Disturbances\');
addpath('.\Simulation\Helper\');
addpath('.\Simulation\Stability\');
addpath('.\Simulation\Vehicle Motion\');

%% Initial State
% Build constants array (You probably only wanna run this once at the
% beggining. Eats up a bit of performance but it is necessary to load in
% matrices and vehicle constants)
constantsASTRA = constructConstants;
[x, u2, x_dot] = EoMGenerator(constantsASTRA, 2);
[linSys, disLinSys] = dynamics(x, u2, x_dot, constantsASTRA);
linSys.A = linSys.A(1:12,1:12);
linSys.B = linSys.B(1:12,:);

% PABLO TEST
[~, linSys] = Controller2_Gen(constantsASTRA);

% Define the bounds for the actuators
thrustMax = 1.5 * 9.8;   
gimbalMax = pi/18;
InputBounds = [-gimbalMax       gimbalMax;
               -gimbalMax       gimbalMax;
               .4 * thrustMax   thrustMax;
               -pi/6            pi/6];

% PABLO GUESS
% Hand tuning for Q for now
a_weights = ones(6,1);
a_weights = a_weights / norm(a_weights);
max_x = [0.28, 0.28, 0.25, 40, 40, 1.0];
Q = eye(6) .* a_weights ./ max_x.^2;
R_g = diag([5, 5, 0.2]);

% Augment Q with integral states
Qi = diag([2, 2, 4]);
Q_g = [Q zeros(6,3);
     zeros(3,6) Qi];

DM_min = 1.0; % minimum disk margin, if disk margin is below this do not consider crossover freq

%% Genetic Algorithm
popSize = 20;
mut_rate_i = .7;
mut_rate_f = .1;
mut_factor_i = 100;
mut_factor_f = 1;
mut_func = @mut_func_fitBased;
gen_cut = .5;
elite_cut = 0;
task_func = @task_func;
fit_func = @fit_func;
paramArray = {linSys, constantsASTRA, thrustMax, @evalDiskMarginReduced, popSize, DM_min};

% Initialize population and root node
gene_seed = [Q_g(1,1); Q_g(3,3); Q_g(4,4); Q_g(6,6); Q_g(7,7); Q_g(9,9); R_g(1,1); R_g(3,3)];
popInitial = population(1, gene_seed, popSize, mut_rate_i, mut_rate_f, mut_factor_i, mut_factor_f, mut_func, gen_cut, elite_cut, task_func, fit_func, paramArray);

% Start parallel pool if not started yet
% n_cores = 6;
% if isempty(gcp('nocreate')), parpool(n_cores); end % start parallel pool (set number of workers depending on how CPU intensive you want the process to be

% RUN GENETIC ALGORITHM
%   Runs the genetic algorithm process until the population size
%   reaches 1.
pops = {popInitial};
while pops{end}.popSize > 1
    fprintf("\nPOPULATION: %d\n", pops{end}.generation)
    pops{end}.prefTask;
    pops{end}.fitEval;
    pops{end}.kill;
    pops{end+1} = pops{end}.reproduce;
end
popFinal = pops{end};

% Genetic algorithm results
a = popFinal.genes{1}.alleles;
Q = diag([ones(2,1) * a(1); a(2); ones(3,1) * a(3); ones(2,1) * a(4); a(5); ones(2,1) * a(6); a(7)]);
R = diag([ones(2,1) * a(8); a(9); a(10)]);
[K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);

% Save final pop and root as a struct
gaData.('populations') = pops;
% gaData.('root') = root;
gaData.('Q') = Q;
gaData.('R') = R;
gaData.('K') = K;
save('.\Controls\Controller Optimization\GA_' + ...
    string(datetime(now,'ConvertFrom','datenum', 'Format', 'yyyy-MM-dd_HH.mm.ss')) + ...
    '__popSize' + string(popSize) + ...
    '_fit' + string(popFinal.genes{1}.fitness) + '.mat', 'gaData')