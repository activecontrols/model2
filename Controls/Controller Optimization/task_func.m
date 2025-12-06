function states = task_func(gene_instance)
% TASK_FUNC
%   
    arguments (Input)
        gene_instance gene
    end
    
    % Extract required info from the gene
    a = gene_instance.alleles;

    Q = diag([ones(2,1) * a(1); a(2); ones(2,1) * a(3); a(4); ones(2,1) * a(5); a(6)]);
    R = diag([ones(2,1) * a(7); a(8)]);

    linSys = gene_instance.parameters{2};
    ActuatorDelay = gene_instance.parameters{3};

    % Begin evaluating disk margin
    [K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);

    % First system linearization
    A = linSys.A;
    B = linSys.B;
    C = eye(size(linSys.A, 1));
    D = zeros(size(linSys.A, 1), size(linSys.B, 2));
    
    % Plant TF
    P = ss(A, B, C, D);
    
    % Controller TF (set only once)
    K_ss = ss(K);
    
    % Feedback TF
    L = K_ss * P;
    
    % Delayed Feedback TF
    Delay_MIMO_ss = ActuatorDelay;
    L = L * Delay_MIMO_ss;
    
    % Digital Filter TF (Using the transfer function of the worst-case digital
    % filter we'll have on board)
    thrust = 80;
    [Filter_TF, ~] = FilterTF_Gen(thrust);
    Filter_ss = ss(Filter_TF);
    L = L * Filter_ss;
    
    % Final Disk Margin structs.
    [DM, MM] = diskmargin(L);
    
    states = {DM, MM};
end