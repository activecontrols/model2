classdef gene < matlab.mixin.Copyable
    %GENE
    %   Genome class used to generate population within genetic algorithm.
    %   Properties:
    %       alleles    = Array of important identifying information associated
    %                    with the gene. Alleles determine genome performance
    %                    and alleles that optimize the fitness function are 
    %                    selected for by the algorithm.
    %
    %       states     = The states associated with the alleles of the gene.
    %
    %       constCheck = Boolean array of what allele constraints are met.

    properties
        alleles % array of gene specific data that is optimized for by GA
        fitness % fitness value of gene based on a fitness function defined at the population level
        error = false % boolean to record if there is an error performing the task
        odeStopped % boolean recording if the genome results in successful integration using ODE45
        states = {} % Cell array of variables associated with task performance 
        constCheck % logical array of which state constraints are met (might want to remove to make gene more generally applicable)
        parameters % Additional parameters specific to the system being optimized
        generation % Generation number that gene belongs to
        elite = false % If the gene is considered elite
        alive = true % If the gene is considered alive
        children = {} % Cell array of the gene's children
        parent1 % 
        parent2 % 
    end

    methods
        % CONSTRUCTOR
        %   Populates alleles with allele seed
        function obj = gene(allele_seed, parameters, generation)
            obj.alleles = allele_seed;
            obj.parameters = parameters;
            obj.generation = generation;
        end

        %CROSSOVER
        %   Performs crossover between two gene objects
        %   
        %   RETURNS: array of alleles for child1 and child2
        function [child1, child2] = crossover(obj1, obj2)
            p1 = randi(size(obj1.alleles,1));
            p2 = randi(size(obj1.alleles,1));
            
            child1 = obj1.alleles;
            child2 = obj2.alleles;
            
            temp = child1(p1:p2);
            child1(p1:p2) = child2(p1:p2);
            child2(p1:p2) = temp;
        end

        % MUTATE
        %   Performs mutation on alleles of gene. 
        %   Note: seed correpsonds to the seed value of the gene pool
        %         (initially this is just Bryson's Rule but could change as
        %         persistent data becomes avaliable).
        function child = mutate(obj, mut_rate, seed, mut_factor)
            maxVal = seed * mut_factor;
            minVal = ones(size(seed)) * 1.0e-08; % I didn't use zero to avoid making Q not positive-definite
            sigma = (maxVal - minVal)/3;
            child = obj.alleles;
            
            for i = 1:length(child)
                if rand < mut_rate
                    child(i) = child(i) + sigma(i) * randn;
                end
            end
            
            child(child > maxVal) = maxVal(child > maxVal);
            child(child < minVal) = minVal(child < minVal);
        end

        % GAUSSIAN MUTATION 
        %   Applies Gaussian mutation operator to the alleles of a gene,
        %   each allele will be bounded based on the associated index of a
        %   and b.
        %   INPUTS:
        %       a: lower bound of each allele, vector of the same size as 
        %           the gene
        %       b: upper bound of each allele, vector of the same size as
        %           the gene
        %       sigma: the standard deviation of the desired gaussian
        %           distribution from which the mutation will be generated
        %   OUTPUTS:
        %       child: a set of mutated alleles
        function child = gaussian_mutate(obj, a, b, sigma)
            child = obj.alleles;
            alpha = (8 * (pi - 3)) / (3 * pi * (4 - pi)); % constant used in approximating inverse of erf()

            % Iterate through each allele and apply the mutation operator
            for i = 1:length(obj.alleles)
                u = rand;
                sigma_nd = sigma(i) / (b(i) - a(i)); % create nondimensionalized sigma value

                u_L = 0.5 * (erf((a(i) - child(i)) / (sqrt(2) * (b(i) - a(i)) * sigma_nd)) + 1);
                u_R = 0.5 * (erf((b(i) - child(i)) / (sqrt(2) * (b(i) - a(i)) * sigma_nd)) - 1);

                if u <= 0.5 % Compute u_i' based on uniformly sampled u_i
                    u_prime = 2 * u_L * (1 - 2 * u);
                else
                    u_prime = 2 * u_R * (2 * u - 1);
                end

                erf_inv = sign(u) * sqrt(sqrt((2 / (pi * alpha) + ...
                    log(1 - u_prime^2) / 2) ^ 2 - ...
                    log(1 - u_prime^2) / alpha) - ...
                    (2 / (pi * alpha) + log(1 - u_prime^2) / 2));

                % Apply mutation operator
                child(i) = child(i) + sqrt(2) * sigma_nd * (b(i) - a(i)) * erf_inv;
            end
        end

        %CONSTRAINT CHECK
        %   Checks against constraint array to ensure solution meats pre-defined 
        %   constraints.
        function check = constraintCheck(obj, stateLimits)
            %check against state limits
            minCheck = obj.states > stateLimits(:, 1);
            maxCheck = obj.states < stateLimits(:, 2);
        
            % account for states with no limits (isnan is a logical array with a 1 
            % for any NaN in the passed array so adding it to constCheck will make 
            % all states with no limit true
            minCheck = minCheck + isnan(stateLimits(:, 1)) .* ones(size(minCheck));
            maxCheck = maxCheck + isnan(stateLimits(:, 2)) .* ones(size(maxCheck));
            check = [minCheck, maxCheck];
        end
    end
end