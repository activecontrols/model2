function mut_rate = mut_func(pop)
%MUT_FUNC
%   Function to define a mutation rate based on characteristics of the
%   current population.
arguments (Input)
    pop population
end

arguments (Output)
    mut_rate
end

mut_rate1 = pop.mut_rate1;
% mut_rate2 = pop.mut_rate2;
% popSize = pop.popSize;

mut_rate = mut_rate1;

end