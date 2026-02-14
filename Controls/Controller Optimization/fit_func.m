function fitness = fit_func(gene_instance)
%FIT_FUNC
%   
arguments (Input)
    gene_instance gene
end

    DM_min = gene_instance.parameters{4};

    if ~gene_instance.error
        DM = gene_instance.states{1};
        
        minDiskMargin = inf; % initialize minimum disk margin
        minFrequency = inf; % initialize minimum crossover frequency
        for i = 1:length(DM)
            minDiskMargin = min(minDiskMargin, DM(i).DiskMargin);
            minFrequency = min(minFrequency, DM(i).Frequency);
        end
    
        if minFrequency ~= inf && minDiskMargin > DM_min % unstable system will set Frequency to NaN resulting in minFrequency still being inf
            fitness = 0.05 * minDiskMargin + 0.95 * minFrequency;
        elseif minFrequency ~= inf && minDiskMargin <= DM_min % If disk margin below 1 ignore crossover freq so it doesn't dominate the solution
            fitness = minDiskMargin;
        else
            fitness = -1;
        end
    else
        fitness = -1;
    end
end