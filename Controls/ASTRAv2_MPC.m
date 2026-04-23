% MPC Controller for ASTRAv2
%
% Operates on a linearized error-state formulation to return constrained
% optimal control inputs.
%
% INPUTS:
%   x         - current state (this will be the estimated state when running in
%               the loop)
%   x_ref     - reference state (must pass enough time steps to match
%               prediction horizon)
%   u_ref     - reference input (must pass enough time steps to match
%               prediction horizon)
%   
%   constants - ASTRAv2 constants
%
% OUTPUTS:
%   u   - control input
%   e_x - error-state (for diagnostic purposes)
%
% By: Alexander Kaufmann
% Last updated: 04/02/2026

function [u, e_x] = ASTRAv2_MPC(x, x_ref, u_ref, k, n_mpc, constants)
    % Extract values from the constants struct
    m = constants.m;
    g = constants.g;
    Q = constants.Q_mpc;
    P = constants.P_mpc;
    R = constants.R_mpc;
    u_min = constants.u_min;
    u_max = constants.u_max;
    zeta_max = constants.zeta_max; % maximum angular deviation from inertial z-axis
    r_min = constants.r_min;
    v_min = constants.v_min;
    omega_min = constants.omega_min;
    r_max = constants.r_max;
    v_max = constants.v_max;
    omega_max = constants.omega_max;
    
    % Compute error state
    e_q = quatmultiply(quatinv(x_ref(1:4, k+1).'), x(1:4).').';
    alpha = 2 * e_q(2:end);
    e_x = [alpha;
         x(5:13) - x_ref(5:13, k+1)];
    
    dim_ex = length(e_x);
    dim_u = size(u_ref, 1);

    % Define error trim conditions
    e_x_trim = zeros(dim_ex, 1);
    u_trim = [0; 0; m*g; 0];
    
    %% Dynamics prediction (TODO: MOVE BLOCK MATRIX BUILDING TO SEPARATE, 
    % NON-RUNTIME, FUNCTION) would require defining cell arrays as N x n 
    % where N is total time steps
    % Compute state jacobians and transition matrices
    A = zeros(dim_ex, dim_ex, n_mpc);
    B = zeros(dim_ex, dim_u, n_mpc);
    Phi = zeros(dim_ex, dim_ex, n_mpc+1);


    Phi(:,:,1) = eye(dim_ex); % state transition from step k to step k is identity

    for i = 1:n_mpc % all the indexing is different from my derivations because MATLAB indexing is lame
        A(:,:,i) = JacobianErrorX(e_x, u_trim, x_ref(:, k+i), u_ref(:, k+i));
        B(:,:,i) = JacobianErrorU(e_x, u_trim, x_ref(:, k+i), u_ref(:, k+i));
        Phi(:,:,i+1) = A(:,:,i) * Phi(:,:,i); % we get Phi ranging from k->k to k->k+n, 
        % H uses k->k+1:k->k+n, G uses k->k:k->k+n-1
    end

    % Create prediction matrices (THIS PART WOULD NEED TO INCORPORATE k TO
    % COMPUTE BEFORE SIMULATING)
    H = zeros(dim_ex * n_mpc, dim_ex);
    G = zeros(dim_ex * n_mpc, dim_u * n_mpc);
    
    for i = 1:n_mpc
        row_s = (i-1)*dim_ex + 1;
        row_f = i*dim_ex;
        
        H(row_s:row_f, :) = Phi(:,:,i+1);
        
        % Diagonal block: G(i,i) = B(i), no transition needed
        col_s = (i-1)*dim_u + 1;
        col_f = i*dim_u;
        G(row_s:row_f, col_s:col_f) = B(:,:,i);
        
        % Propagate all previously-set column-j blocks down one row
        % by left-multiplying with A(:,:,i)
        for j = 1:i-1
            prev_row_s = (i-2)*dim_ex + 1;
            prev_row_f = (i-1)*dim_ex;
            col_s_j = (j-1)*dim_u + 1;
            col_f_j = j*dim_u;
            G(row_s:row_f, col_s_j:col_f_j) = A(:,:,i) * G(prev_row_s:prev_row_f, col_s_j:col_f_j);
        end
    end

    %% Cost Prediction
    Q_bar = zeros(dim_ex * n_mpc, dim_ex * n_mpc);
    R_bar = zeros(dim_u * n_mpc, dim_u * n_mpc);

    for i = 1:n_mpc
        for j = 1:n_mpc
            idx_Q = (i - 1) * dim_ex;
            idx_Q_s = idx_Q + 1;
            idx_Q_f = idx_Q + dim_ex;
            
            if i < n_mpc
                Q_bar(idx_Q_s:idx_Q_f, idx_Q_s:idx_Q_f) = Q;
            else
                Q_bar(idx_Q_s:idx_Q_f, idx_Q_s:idx_Q_f) = P;
            end

            idx_R = (i - 1) * dim_u;
            idx_R_s = idx_R + 1;
            idx_R_f = idx_R + dim_u;
            
            R_bar(idx_R_s:idx_R_f, idx_R_s:idx_R_f) = R;
        end
    end
    
    M = Q + H.' * Q_bar * H;
    F = G.' * Q_bar * H;
    L = G.' * Q_bar * G + R_bar;

    %% Constraint Handling
    Iu = eye(dim_u);
    I3 = eye(3); % this can be hard-coded because almost all states are vectors in R3
    Z3 = zeros(3);

    % Input constraints
    W_i = zeros(2 * n_mpc * dim_u, 1);
    E_i = zeros(2 * n_mpc * dim_u, n_mpc * dim_u);

    for i = 1:n_mpc 
        row = (i - 1) * 2 * dim_u;
        row_s = row + 1;
        row_f = row + 2 * dim_u;
        col = (i - 1) * dim_u;
        col_s = col + 1;
        col_f = col + dim_u;

        W_i(row_s:row_f) = [-u_min + u_ref(:, k+i); 
                             u_max - u_ref(:, k+i)];

            
        E_i(row_s:row_f, col_s:col_f) = [-Iu; 
                                          Iu];
    end

    % State Constraints
    E_xi = zeros(2 * 9 + 1, 12); % see the documentation on the MPC controller for more details behind this structure
    W_xi = zeros(2 * 9 + 1, 1);
    E_X = zeros(n_mpc * size(E_xi));
    W_X = zeros(n_mpc * size(W_xi, 1), 1);
    z_hat_i = [0; 0; 1]; % unit vector for inertial-frame z-axis

    for i = 1:n_mpc
        row = (i - 1) * 19;
        row_s = row + 1;
        row_f = row + 19;
        col = (i - 1) * dim_ex;
        col_s = col + 1;
        col_f = col + dim_ex;

        z_hat_ref = quatRot(x_ref(1:4, k+i)) * [0; 0; 1]; % unit vector for reference body-frame z-axis
        W_X(row_s:row_f) = [z_hat_ref.' * z_hat_i - cos(zeta_max);
                            -r_min + x_ref(5:7, k+i);
                             r_max - x_ref(5:7, k+i);
                            -v_min + x_ref(8:10, k+i);
                             v_max - x_ref(8:10, k+i);
                            -omega_min + x_ref(11:13, k+i);
                             omega_max - x_ref(11:13, k+i)];
        
        E_X(row_s:row_f, col_s:col_f) = [-z_hat_i.' * zetaCross(z_hat_ref) * quatRot(x_ref(1:4, k+i)), zeros(1,3), zeros(1,3), zeros(1,3);
                                         Z3, -I3,  Z3,  Z3;
                                         Z3,  I3,  Z3,  Z3;
                                         Z3,  Z3, -I3,  Z3;
                                         Z3,  Z3,  I3,  Z3;
                                         Z3,  Z3,  Z3, -I3;
                                         Z3,  Z3,  Z3,  I3];
    end
    
    E_s = E_X * G;
    W_s = W_X - E_X * H * e_x;

    % Combined linear matrix inequality constraints
    % E = [E_i; E_s];
    % W = [W_i; W_s];
    E = E_i;
    W = W_i;

    %% Use solver to find required error input
    options = mpcActiveSetOptions; % default options
    iA = false(size(W)); % define all inequality constraints as active because of solver (I dont actually know what this does)

    U = mpcActiveSetSolver(L, F * e_x, E, W, zeros(0,dim_u*n_mpc), zeros(0,1), iA, options);
    
    % Extract e_u(k) from U, then extract u from e_u = u - u_ref
    K = [eye(dim_u) zeros(dim_u, dim_u * n_mpc - dim_u)];
    u = K * U + u_trim;
end