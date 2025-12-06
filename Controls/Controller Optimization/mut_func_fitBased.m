function mut_factor = mut_func_fitBased(pop, fit)
%MUT_FUNC
%   Function to define a mutation rate based on characteristics of the
%   current population.
arguments (Input)
    pop population
    fit float
end

arguments (Output)
    mut_factor
end

mut_factor_i = pop.mut_factor_i;
mut_factor_f = pop.mut_factor_f;

mut_factor = mut_factor_f + (fit - min(pop.fitnesses)) / (max(pop.fitnesses) - min(pop.fitnesses)) * (mut_factor_i - mut_factor_f);

end