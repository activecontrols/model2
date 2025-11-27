function [Q, R, root] = genetic_algorithm(x, Q_g, R_g, paramArray, popSize, mut_rates, mut_factor, cutoffs, mut_func, task_func, fit_func)
% GENETIC_ALGORITHM
%   Unique Inputs: popSize = initial population size
%                  mut_rate1 = initial mutation rate
%                  mut_rate2 = final mutation rate
%                      - mutation rate decreases on a power scale from 
%                        mut_rate1 to mut_rate2 as the population decreases
%                  gen_cut = cuttoff point for general population
%                  elite_cut = cuttoff point for elite population (top 1 
%                              solution will always be preserved regardless
%                              of the cuttoff rate)
%
%   General Info: The genetic algorithm has X stages. First, an initial 
%                 population is generated using Bryson's Rule as a basis
%                 then performing mutation and crossover on all but one 
%                 solution to create variation. Second, the dynamics of the
%                 system are modeled in order to evaluate fitness. Third,
%                 the population undergoes the culling/reproduction stage.
%                 The top gen_cut solutions are kept and undergo mutation
%                 and crossover with eachother. The top elite_cut solutions
%                 do not undergo mutation or crossover with any other
%                 solutions.
    mut_rate1 = mut_rates(1);
    mut_rate2 = mut_rates(2);
    gen_cut = cutoffs(1);
    elite_cut = cutoffs(2);
    
    % Initialize population and root node
    pop = population(1, [Q_g; R_g], popSize, mut_rate1, mut_rate2, mut_factor, mut_func, gen_cut, elite_cut, task_func, fit_func, paramArray);
    root = node.init_tree(int64(0), pop.nodes);
    
    % Run genetic algorithm on population
    pop.runGA;
    
    Q = diag(pop.nodes{1}.gene.alleles(1:size(x,1)));
    R = diag(pop.nodes{1}.gene.alleles(size(x,1) + 1:end));
    root.gene = pop.nodes{1}.gene;

end