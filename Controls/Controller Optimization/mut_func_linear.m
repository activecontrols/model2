function [mut_rate, mut_factor] = mut_func_linear(pop)
%MUT_FUNC
%   Function to define a mutation rate based on characteristics of the
%   current population.
arguments (Input)
    pop population
end

arguments (Output)
    mut_rate
    mut_factor
end

mut_rate_i = pop.mut_rate_i;
mut_rate_f = pop.mut_rate_f;
mut_factor_i = pop.mut_factor_i;
mut_factor_f = pop.mut_factor_f;
popSize = pop.popSize;
initialPop = pop.parameters{5};
% generation = pop.generation;


mut_rate = (mut_rate_f - mut_rate_i) / (2 - initialPop) * (popSize - initialPop) + mut_rate_i;
mut_factor = (mut_factor_f - mut_factor_i) / (2 - initialPop) * (popSize - initialPop) + mut_factor_i;

end