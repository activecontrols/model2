% MPC Controller for ASTRAv2
%
% Operates on a linearized error-state formulation to return constrained
% optimal control inputs.
%
% INPUTS:
%   x         - current state (this will be the estimated state when running in
%               the loop)
%   t         - current time
%   x_ref     - reference state (must pass enough time steps to match
%               prediction horizon)
%   u_ref     - reference input (must pass enough time steps to match
%               prediction horizon)
%   constants - ASTRAv2 constants
%
% OUTPUTS:
%
% By: Alexander Kaufmann
% Last updated: 04/02/2026

function u = ASTRAv2_MPC(x, x_ref, u_ref, k, constants)
    % Compute error state
    e_q = quatmultiply(quatinv(x_ref(1:4, k+1).'), x(1:4).').';
    alpha = 2 * e_q(2:end);
    e_x = [alpha;
         x(5:13) - x_ref(5:13, k+1)];
    
    dim_ex = length(e_x);
    dim_eu = size(u_ref, 1);
    n = constants.n_mpc;

    % Define error trim conditions
    e_x_trim = zeros(dim_ex, 1);
    e_u_trim = zeros(dim_eu, 1);
    
    %% Dynamics prediction (TODO: MOVE BLOCK MATRIX BUILDING TO SEPARATE, 
    % NON-RUNTIME, FUNCTION) would require defining cell arrays as N x n 
    % where N is total time steps
    % Compute state jacobians and transition matrices
    A = cell(1, n);
    B = cell(1, n);
    phi = cell(1, n+1);


    phi{k+1} = eye(dim_ex); % state transition from step k to step k is identity

    for i = 1:n % all the indexing is different from my derivations because MATLAB indexing is lame
        A{k+i} = JacobianErrorX(e_x_trim, e_u_trim, x_ref(:, k+i), u_ref(:, k+i));
        B{k+i} = JacobianErrorU(e_x_trim, e_u_trim, x_ref(:, k+i), u_ref(:, k+i));
        phi{k+i+1} = A{k+i} * phi{k+i}; % we get phi ranging from k->k to k->k+n, 
        % H uses k->k+1:k->k+n, G uses k->k:k->k+n-1
    end

    % Create prediction matrices (THIS PART WOULD NEED TO INCORPORATE k TO
    % COMPUTE BEFORE SIMULATING)
    H = zeros(dim_ex * n, dim_ex);
    G = zeros(dim_ex * n, dim_eu * n);
    
    for i = 1:n
        row = (i - 1) * dim_ex; % block matrix row (base 0)
        row_s = row + 1; % block matrix row starting index (base 1)
        row_f = row + dim_ex; % block matrix row ending index (base 1)
        
        H(row_s:row_f, :) = phi{i+1}; % H blocks (transition matrices starting at k->k+1)

        for j = 1:n
            if i >= j
                col = (j - 1) * dim_eu; % block matrix column (base 0)
                col_s = col + 1; % block matrix column starting index (base 1)
                col_f = col + dim_eu; % block matrix column ending index (base 1)

                G(row_s:row_f, col_s:col_f) = phi{i} * inv(phi{j}) * B{j}; % G blocks
            end
        end
    end

    %% Cost Prediction
    Q = constants.Q_mpc;
    P = constants.P_mpc;
    R = constants.R_mpc;

    Q_bar = zeros(dim_ex * n, dim_ex * n);
    R_bar = zeros(dim_eu * n, dim_eu * n);

    for i = 1:n
        for j = 1:n
            idx_Q = (i - 1) * dim_ex;
            idx_Q_s = idx_Q + 1;
            idx_Q_f = idx_Q + dim_ex;
            
            if i < n
                Q_bar(idx_Q_s:idx_Q_f, idx_Q_s:idx_Q_f) = Q;
            else
                Q_bar(idx_Q_s:idx_Q_f, idx_Q_s:idx_Q_f) = P;
            end

            idx_R = (i - 1) * dim_eu;
            idx_R_s = idx_R + 1;
            idx_R_f = idx_R + dim_eu;
            
            R_bar(idx_R_s:idx_R_f, idx_R_s:idx_R_f) = R;
        end
    end
    
    M = Q + H.' * Q_bar * H;
    F = G.' * Q_bar * H;
    L = G.' * Q_bar * G + R_bar;

    %% Constraint Handling
    Iu = eye(dim_eu);
    I3 = eye(3); % this can be hard-coded because almost all states are vectors in R3
    Z3 = zeros(3);
    u_min = constants.u_min;
    u_max = constants.u_max;
    zeta_max = constants.zeta_max; % maximum angular deviation from inertial z-axis
    x_min = constants.x_min; % does not govern orientation
    x_max = constants.x_max; % does not govern orientation

    % Input constraints
    W_i = zeros(2 * n * dim_eu, 1);
    E_i = zeros(2 * n * dim_eu, n * dim_eu);

    for i = 1:n 
        row = (i - 1) * 2 * dim_eu;
        row_s = row + 1;
        row_f = row + 2 * dim_eu;
        
        W_i(row_s:row_f) = [-u_min + u_ref(:, k+i-1); 
                            u_max - u_ref(:, k+i-1)];

        for j = 1:n
            col = (j - 1) * dim_eu;
            col_s = col + 1;
            col_f = col + dim_eu;

            E_i(row_s:row_f, col_s:col_f) = [-Iu; 
                                             Iu];
        end
    end

    % State Constraints
    E_xi = zeros(2 * 9 + 1, 10); % see the documentation on the MPC controller for more details behind this structure
    W_xi = zeros(2 * 9 + 1, 1);
    E_X = zeros(n * size(E_xi));
    W_X = zeros(n * size(W_xi, 1), 1);
    z_hat_i = [0; 0; 1]; % unit vector for inertial-frame z-axis

    for i = 1:n
        row = (i - 1) * (2 * 9 + 1);
        row_s = row + 1;
        row_f = row + (2 * 9 + 1);

        z_hat_ref = x_ref(7, k+i-1); % unit vector for reference body-frame z-axis
        W_X(row_s:row_f) = [z_hat_ref.' * z_hat_i - cos(zeta_max);
                            -x_min(5:7) + x_ref(5:7, k+i-1);
                            x_max(5:7) - x_ref(5:7, k+i-1);
                            -x_min(5:7) + x_ref(8:10, k+i-1);
                            x_max(5:7) - x_ref(8:10, k+i-1);
                            -x_min(5:7) + x_ref(11:13, k+i-1);
                            x_max(5:7) - x_ref(11:13, k+i-1)]; % indices of x_min and x_max might have to be changed depending on how we decide to format the vectors at higher levels

        for j = 1:n
            col = (j - 1) * 10;
            col_s = col + 1;
            col_f = col + 10;

            E_X(row_s:row_f, col_s:col_f) = [-z_hat_i.' * zetaCross(z_hat_ref) * quatRot(x_ref(1:4, k+i-1)), Z3,  Z3,  Z3;
                                             zeros(3, 1),                                                   -I3,  Z3,  Z3;
                                             zeros(3, 1),                                                    I3,  Z3,  Z3;
                                             zeros(3, 1),                                                    Z3, -I3,  Z3;
                                             zeros(3, 1),                                                    Z3,  I3,  Z3;
                                             zeros(3, 1),                                                    Z3,  Z3, -I3;
                                             zeros(3, 1),                                                    Z3,  Z3,  I3];
        end
    end
    
    E_s = E_X * G;
    W_s = W_X - E_X * H * e_x;

    % Combined linear matrix inequality constraints
    E = [E_i; E_s];
    W = [W_i; W_s];

    %% Use solver to find required error input
    options = mpcActiveSetOptions; % default options
    iA = false(size(W)); % define all inequality constraints as active because of solver (I dont actually know what this does)

    U = mpcActiveSetSolver(L, F * e, E, W, [], zeros(0,1), iA, options);
    
    % Extract e_u(k) from U, then extract u from e_u = u - u_ref
    K = [eye(dim_eu) zeros(dim_eu, dim_eu * n - dim_eu)];
    e_u = K * U;
    u = e_u + u_ref(:, k);
end