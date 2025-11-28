function StateSpace = ActuatorDelay
    %Creates a first order actuator model
    ActuatorModel = cell(4, 1);
    tau = [0.08; 0.08; 0.15; 0.15];

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
    
    % First system linearization
    x0 = zeros(15,1);
    u0 = [0; 0; constantsASTRA.g * constantsASTRA.m; 0];
    A = JacobianX(x0, u0);
    A = A(1:12, 1:12);
    B = JacobianU(x0, u0);
    B = B(1:12, :);
    C = eye(12);
    D = zeros(12, 4);
    
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
    thrust = u0(3) / thrustMax;
    [Filter_TF, ~] = FilterTF_Gen(thrust);
    Filter_ss = ss(Filter_TF);
    L = L * Filter_ss;
    
    % Final Disk Margin structs.
    [DM, MM] = diskmargin(L);
end

clear;
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

% Define the bounds for the actuators
thrustMax = 1.5 * 9.8;   
gimbalMax = pi/18;
InputBounds = [-gimbalMax       gimbalMax;
               -gimbalMax       gimbalMax;
               .4 * thrustMax   thrustMax;
               -pi/6            pi/6];

% Load LQR tuning matrices for recomputing
% Brysons Rule for Q and R.
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [3, 3, 0.5, 1000, 1000, 1000, 1, 1, 0.4, pi/8, pi/8, 2];
max_u = [pi/18, pi/18, 6, 0.4];

% Q_g = eye(size(linSys.A,1)) .* a_weights ./ max_x.^2;
% R_g = diag([260, 260, 4, 10]);

Q_g = [0.590594299020442, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0.590594299020442, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 1.000000000000000e-08, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 1.000000000000000e-08, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 1.000000000000000e-08, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 1.000000000000000e-08, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 8.036356619219551e-08, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 8.036356619219551e-08, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 6.590156173100899e+02, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 84.750825308901454, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 84.750825308901454, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.490187042445407e-07];
R_g = [1.489332493744518e+04, 0, 0, 0;
    0, 1.489332493744518e+04, 0, 0;
    0, 0, 26.044672063044732, 0;
    0, 0, 0, 1.000000000000000e-08];
DM_min = 1.0; % minimum disk margin, if disk margin is below this do not consider crossover freq

% [DM, MM] = evalDiskMarginReduced(Q, R, linSys, constantsASTRA, thrustMax);

%% Genetic Algorithm
popSize = 1000;
mut_rate_i = .7;
mut_rate_f = .1;
mut_factor_i = 100;
mut_factor_f = 10;
mut_func = @mut_func_linear;
gen_cut = .5;
elite_cut = 0;
task_func = @task_func;
fit_func = @fit_func;
paramArray = {linSys, constantsASTRA, thrustMax, @evalDiskMarginReduced, popSize, DM_min};


% Initialize population and root node
allele_seed = [Q_g(1,1); Q_g(3,3); Q_g(4,4); Q_g(7,7); Q_g(9,9); Q_g(10,10); Q_g(12,12); R_g(1,1); R_g(3,3); R_g(4,4)];
popInitial = population(1, allele_seed, popSize, mut_rate_i, mut_rate_f, mut_factor_i, mut_factor_f, mut_func, gen_cut, elite_cut, task_func, fit_func, paramArray);

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
% root.gene = pop.nodes{1}.gene;
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
save('GA_' + ...
    string(datetime(now,'ConvertFrom','datenum', 'Format', 'yyyy-MM-dd_HH.mm.ss')) + ...
    '__popSize' + string(popSize) + ...
    '_fit' + string(popFinal.genes{1}.fitness) + '.mat', 'gaData')