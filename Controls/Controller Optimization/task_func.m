function states = task_func(gene_instance)
% TASK_FUNC
%   
    arguments (Input)
        gene_instance gene
    end
    
    % Extract required values from gene
    a = gene_instance.alleles;
    constantsASTRA = gene_instance.parameters{1};
    lqri_gain = gene_instance.parameters{2};
    ActuatorDelay = gene_instance.parameters{3};

    % K_P1 = a(1:3);
    % K_P2 = a(4:6);
    % K_I = a(7:9);
    Q = diag(a(1:9));
    R = diag(a(10:12));

    [K_att, linSys] = lqri_gain(constantsASTRA, Q, R);
    

    % % First system linearization
    % x0 = zeros(15,1);
    % u0 = [0; 0; constantsASTRA.g * constantsASTRA.m; 0];
    % A = JacobianX(x0, u0);
    % A = A(1:12, 1:12);
    % B = JacobianU(x0, u0);
    % B = B(1:12, :);
    % C = eye(12);
    % D = zeros(12, 4);
    
    % Linearized system (NOTE: no longer linearizing about full set of
    % dynamics)
    A = linSys.A;
    B = linSys.B;
    C = eye(9);
    D = zeros(9, 3);

    % Plant TF
    P = ss(A, B, C, D);
    
    % Controller TFs
    K_att_ss = ss(K_att);
    

    % Feedback TF
    L = K_att_ss * P;
    
    % % Delayed Feedback TF (TODO: setup actuator delay for just attitude)
    % Delay_MIMO_ss = ActuatorDelay;
    % L = L * Delay_MIMO_ss;
    
    % % Digital Filter TF (Using the transfer function of the worst-case digital
    % % filter we'll have on board)
    % thrust = u0(3) / thrustMax;
    % [Filter_TF, ~] = FilterTF_Gen(thrust);
    % Filter_ss = ss(Filter_TF);
    % L = L * Filter_ss;
    
    % Final Disk Margin structs.
    [DM, MM] = diskmargin(L);

    states = {DM, MM};

end