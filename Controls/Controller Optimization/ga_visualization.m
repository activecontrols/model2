clear; close all;

%% Controller v1 Runs
% Promissing run of 10,000 from 11/26 evening
% load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-11-26_19.06.02__popSize10000_fit1.4327.mat'

% Promissing run of 10,000 from 11/26 morning (unfortunately done before i
% restructured the GA to store a cell array of each population
% load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-11-26_10.32.35__popSize10000_fit1.4534.mat'

% Run of 250 but set mut_rate to .25 and mut_factor to 100
% load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-11-26_21.52.58__popSize250_fit1.2239.mat'

% Run of 10000 mut_rate at .15 and mut_factor at 10 to demonstrate low mutation rate behavior
%load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\v1 Controller\GA_2025-11-30_13.54.55__popSize1000_fit1.5428.mat'

% load 'C:\Users\Owner\Documents\GitHub\model2\GA_2025-11-30_13.41.19__popSize100_fit1.0392.mat'

%% Controller v2 Runs
%load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-12-06_13.48.02__popSize1000_fit2.7077.mat'
%load 'C:\Users\Owner\Documents\GitHub\model2\Controls\Controller Optimization\GA Runs\GA_2025-12-06_15.07.34__popSize1000_fit6.2253.mat'
load 'Controls\'Controller Optimization'\'GA Runs'\GA_2025-12-06_12.38.15__popSize3000_fit1.3066.mat'

%% Plot fitness versus generation
pops = gaData.populations;

% Genetic algorithm results
popFinal = pops{end};
a = popFinal.genes{1}.alleles;
constants = popFinal.parameters{1};
Q = diag([ones(2,1) * a(1); a(2); ones(2,1) * a(3); a(4); ones(2,1) * a(5); a(6)]);
R = diag([ones(2,1) * a(7); a(8)]);
K = Controller2_Gen_GA(constants, Q, R);

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

clear temp1
clear temp2

for i = 1:size(dmargins, 1)
    figure(i+1)
    scatter(dmargins(i, :), freqs(i, :), '.')
    grid on
    xlabel("Channel " + string(i) + " Disk Margin")
    ylabel("Channel " + string(i) + " Crossover Frequency")
    title("Pareto Frontier of Channel " + string(i) + " Disk Margin and Crossover Frequency")
end

DM = popFinal.genes{1}.states{1};
fprintf("Final Disk Margins:\n")
for d = 1:length(DM)
    fprintf("    %f\n", DM(d).DiskMargin)
end

fprintf("\nFinal Crossover Frequencies:\n")
for f = 1:length(DM)
    fprintf("    %f\n", DM(f).Frequency)
end