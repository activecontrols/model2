%% GENETIC_ALGORITHM
%   Optimize control gains using genetic algorithm

%% Required sub-functions
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

clear;
addpath('.\Filtering');
addpath('.\Filtering\ANF');
addpath('.\Simulation');
addpath('.\Simulation\Disturbances\');
addpath('.\Simulation\Helper\');
addpath('.\Simulation\Stability\');
addpath('.\Simulation\Vehicle Motion\');


%% Genetic Algorithm Settings
popSize = ;
mut_rate_i = .7;
mut_rate_f = .1;
mut_factor_i = 100;
mut_factor_f = 1;
mut_func = @mut_func_fitBased;
gen_cut = .5;
elite_cut = 0;
task_func = @task_func;
fit_func = @fit_func;


%% Generate gene seed based on hand tuning
% % First loop K_P
% K_P1 = [0.5; 0.5; 0.65];
% 
% % Second loop K_P and K_I
% K_P2 = [2.2; 2.2; 3.5];
% K_I = [1.5; 1.5; 5];

% Q and R (based on Bryson's rule)
a_weights = ones(6,1);
a_weights = a_weights / norm(a_weights);
max_x = [0.28, 0.28, 0.25, 40, 40, 1.0];
Q = eye(6) .* a_weights ./ max_x.^2;
R = diag([5, 5, 0.2]);

% Augment Q with integral states
Qi = diag([2, 2, 4]);
Q = [Q zeros(6,3);
     zeros(3,6) Qi];

gene_seed = [diag(Q); diag(R)]; % TODO: reduce size of gene by considering symmetric weights

%% Get parameters required to perform task
constants = constructConstants;
[~, linSys] = Controller2_Gen(constants);


params = {constants, linSys, ActuatorDelay};

%% Run GA
popInit = population(0, gene_seed, popSize, mut_rate_i, mut_rate_f, ...
    mut_factor_i, mut_factor_f, mut_func, gen_cut, elite_cut, task_func, ...
    fit_func, params);

pops = {popInit};
while pops{end}.popSize > 1
    fprintf("\nGeneration:    %d\n", pops{end}.generation)
    pops{end}.prefTask;
    pops{end}.fitEval;
    pops{end}.kill;
    pops{end+1} = pops{end}.reproduce;
end

popFinal = pops{end};

%% Genetic algorithm results
% root.gene = pop.nodes{1}.gene;
a = popFinal.genes{1}.alleles;
Q = diag(a(1:9));
R = diag(a(10:12));
K = Controller2_Gen_GA(constants, Q, R);

% Save final pop and root as a struct
gaData.('populations') = pops;
% gaData.('root') = root;
gaData.('Q') = Q;
gaData.('R') = R;
gaData.('K') = K;
save('.\Controls\Controller Optimization\GA Runs\GA_' + ...
    string(datetime(now,'ConvertFrom','datenum', 'Format', 'yyyy-MM-dd_HH.mm.ss')) + ...
    '__popSize' + string(popSize) + ...
    '_fit' + string(popFinal.genes{1}.fitness) + '.mat', 'gaData')