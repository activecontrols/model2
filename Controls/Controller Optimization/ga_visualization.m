close all;

% Promissing run of 10,000 from 11/26 evening
% load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-11-26_19.06.02__popSize10000_fit1.4327.mat'

% Promissing run of 10,000 from 11/26 morning (unfortunately done before i
% restructured the GA to store a cell array of each population
% load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-11-26_10.32.35__popSize10000_fit1.4534.mat'

% Run of 250 but set mut_rate to .25 and mut_factor to 100
% load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-11-26_21.52.58__popSize250_fit1.2239.mat'

% Run of 10000 mut_rate at .15 and mut_factor at 10 to demonstrate low mutation rate behavior
load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-11-27_03.55.15__popSize10000_fit1.7026.mat'

% load 'C:\Users\Owner\Documents\GitHub\model2\GA_2025-11-30_13.41.19__popSize100_fit1.0392.mat'

%% Plot fitness versus generation
pops = gaData.populations;

fits = [];
gens = [];
for p = 1:length(pops)
    if p < length(pops)
        fits = [fits, pops{p}.fitnesses];
    else
        fits = [fits, pops{p}.genes{1}.fitness];
    end

    gens = [gens, ones(1, pops{p}.popSize) * pops{p}.generation];
end

figure(1)
scatter(gens, fits, '.')
grid on
xlabel("Generation")
ylabel("Fitness")
title("Fitness vs. Generation")

%% Plot disk margin vs. frequency
dmargins = [];
freqs = [];
for p = 1:length(pops)
    gene_list = pops{p}.genes;

    for g = 1:length(gene_list)
        if ~gene_list{g}.error
            DM = gene_list{g}.states{1};
    
            temp1 = zeros(length(DM), 1);
            temp2 = temp1;
            for d = 1:length(DM)
                temp1(d) = DM(d).DiskMargin;
                temp2(d) = DM(d).Frequency;
    
            end
    
            dmargins = [dmargins, temp1];
            freqs = [freqs, temp2];
        end
    end
end

figure(2)
scatter(dmargins(1, :), freqs(1, :), '.')
grid on
xlabel("Channel 1 Disk Margin")
ylabel("Channel 1 Crossover Frequency")
title("Pareto Horizon of Channel 1 Disk Margin and Crossover Frequency")

figure(3)
scatter(dmargins(2, :), freqs(2, :), '.')
grid on
xlabel("Channel 2 Disk Margin")
ylabel("Channel 2 Crossover Frequency")
title("Pareto Horizon of Channel 2 Disk Margin and Crossover Frequency")

figure(4)
scatter(dmargins(3, :), freqs(3, :), '.')
grid on
xlabel("Channel 3 Disk Margin")
ylabel("Channel 3 Crossover Frequency")
title("Pareto Horizon of Channel 3 Disk Margin and Crossover Frequency")

figure(5)
scatter(dmargins(4, :), freqs(4, :), '.')
grid on
xlabel("Channel 4 Disk Margin")
ylabel("Channel 4 Crossover Frequency")
title("Pareto Horizon of Channel 4 Disk Margin and Crossover Frequency")