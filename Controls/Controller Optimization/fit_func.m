function fitness = fit_func(gene_instance)
%FIT_FUNC
%   
arguments (Input)
    gene_instance gene
end

    if ~gene_instance.error
        DM = gene_instance.states{1};
        
        minDiskMargin = inf; % initialize minimum disk margin
        minFrequency = inf; % initialize minimum crossover frequency
        for i = 1:length(DM)
            minDiskMargin = min(minDiskMargin, DM(i).DiskMargin);
            if isnan(DM(i).Frequency)
                a = 1;
            end
            minFrequency = min(minFrequency, DM(i).Frequency);
        end
    
        if minFrequency ~= inf && minDiskMargin > 1 % unstable system will set Frequency to NaN resulting in minFrequency still being inf
            fitness = 0.9 * minDiskMargin + 0.1 * minFrequency;
        elseif minFrequency ~= inf && minDiskMargin <= 1 % If disk margin below 1 ignore crossover freq so it doesn't dominate the solution
            fitness = minDiskMargin;
        else
            fitness = -1;
        end
    else
        fitness = -1;
    end
end