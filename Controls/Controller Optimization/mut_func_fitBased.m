function child = mut_func_fitBased(pop, i)
%MUT_FUNC
%   Function to define a mutation rate based on characteristics of the
%   current population.
arguments (Input)
    pop population
    i
end

arguments (Output)
    child
end

    mut_factor_i = pop.mut_factor_i;
    mut_factor_f = pop.mut_factor_f;
    fit = pop.genes{i}.fitness;
    
    if pop.generation > 0
        mut_factor = mut_factor_f + (fit - min(pop.fitnesses)) / (max(pop.fitnesses) - min(pop.fitnesses)) * (mut_factor_i - mut_factor_f);
    else
        mut_factor = mut_factor_i;
    end

    a = ones(size(pop.genes{i}.alleles)) * 1.0e-12; % I didn't use zero to avoid making Q not positive-definite
    b = pop.genes{i}.alleles .* mut_factor;
    b = [b(1:6); 30; 3]; % b override
    sigma = (b - a) ./ 3;
    child = pop.genes{i}.gaussian_mutate(a, b, sigma);
end