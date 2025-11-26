function states = task_func(gene_instance)
% TASK_FUNC
%   
    arguments (Input)
        gene_instance gene
    end
    
    a = gene_instance.alleles;

    Q = diag([ones(2,1) * a(1); a(2); ones(3,1) * a(3); ones(2,1) * a(4); a(5); ones(2,1) * a(6); a(7)]);
    R = diag([ones(2,1) * a(8); a(9); a(10)]);
    linSys = gene_instance.parameters{1};
    constantsASTRA = gene_instance.parameters{2};
    thrustMax = gene_instance.parameters{3};
    evalDiskMarginReduced = gene_instance.parameters{4};

    [DM, MM] = evalDiskMarginReduced(Q, R, linSys, constantsASTRA, thrustMax);
    
    states = {DM, MM};
end