dt = 0.1;
T = 10; % horizon
N_seg = T/dt;
l = @(x, u) x(1:3)'*x(1:3) + u'*u; % running cost
G = @(x) x(1:3)'*x(1:3); % terminal cost
% V_N = 0;
q_max = eul2quat([pi, pi, pi]);
q_min = eul2quat([-pi, -pi, -pi]);
x_min = [-5; -5; -5; -5; -5; -5; q_min'; -5; -5; -5];
x_max = [5; 5; 5; 5; 5; 5; q_max'; 5; 5; 5];
u_min = [0; 0; 0; 0];
u_max = [10; 10; 10; 10];
x0 = x_max;
N_mpc = 2; % MPC window size

% Dynamics
constants = constructConstants;
[x, u, x_dot] = EoMGenerator(constants);
f = matlabFunction(x_dot, 'Vars', [x; u]);

% Linearize dynamics, then  discretize (this should probably be taken care
% of with the dynamics.m function, but right now that function returns a
% linearization about vertical hovering, not the generic state x.
lin.A = jacobian(x_dot, x); % Jacobian of f with respect to x
lin.B = jacobian(x_dot, u); % Jacobian of f with respect to u
lin.C = eye(size(lin.A)); % Until Output is added we have modeled it as being able to directly observe the states
lin.D = zeros(size(lin.B));

% % Discretize using matrix exponential
% Ad = expm(lin.A*dt); %% Might have to discretize at each time step too because syms is super comp intensive for expm
% 
% % Symbolic integral for Bd
% tau = sym('tau', 'real');
% Bd_integrand = expm(lin.A*tau) * lin.B;
% Bd = int(Bd_integrand, tau, 0, dt);

% Create MATLAB function handles
% Ad_func = matlabFunction(Ad, 'Vars', {x, u, dt});
% Bd_func = matlabFunction(Bd, 'Vars', {x, u, dt});
A_func = matlabFunction(lin.A, 'Vars', [x; u]);
B_func = matlabFunction(lin.B, 'Vars', [x; u]);
C_func = matlabFunction(lin.C, 'Vars', [x; u]);
D_func = matlabFunction(lin.D, 'Vars', [x; u]);

% MPC Loop
for k = 1:N_seg
    % Linearize about current state
    A = A_func(x, u);
    B = B_func(x, u);
    C = C_func(x, u);
    D = D_func(x, u);

    % Discretize using matrix exponential
    Ad = expm(A*dt); % Might have to discretize at each time step too because syms is super comp intensive for expm
    
    % Symbolic integral for Bd
    tau = sym('tau', 'real');
    Bd_integrand = expm(A*tau) * B;
    Bd = int(Bd_integrand, tau, 0, dt);

    % Ad = Ad_func(x(:, k), u(:, k), dt);
    % Bd = Bd_func(x(:, k), u(:, k), dt);

    % Dynamics and Cost Prediction Step
    n = size(Ad, 1);
    m = size(Bd, 2);
    H = zeros(N_mpc * n, n);
    G = zeros(N_mpc * n, N_mpc * m);
    
    row = 1;
    for i = 1:N_mpc
        H(row:row+n-1, 1:n) = Ad^i;
        col = 1;
        for j = i:-1:1
            G(row:row+n-1, col:col+m-1) = Ad^(j-1) * Bd;
            col = col + m;
        end
        row = row + n;
    end
    
    % Cost Prediction Step
    Q_bar = zeros(n*N_mpc, n*N_mpc);
    for row = 1:n:n*N_mpc
        for col = 1:n:n*N_mpc
            if row == col
                Q_bar(row:row+n-1, col:col+n-1) = Q;
            end
        end
    end

    R_bar = zeros(m*N_mpc, m*N_mpc);
    for row = 1:m:m*N_mpc
        for col = 1:m:m*N_mpc
            if row == col
                R_bar(row:row+m-1, col:col+m-1) = R;
            end
        end
    end

    F = G' * Q_bar * H;
    L = G' * Q_bar * G + R_bar;

    % Linear Matrix Inequality
    mins = zeros(m*N_mpc, 1);
    maxes = zeros(m*N_mpc, 1);
    for i = 1:m:m*N_mpc
        mins(i:i+m, 1:m) = eye(m) * u_min;
        maxes(i:i+m, 1:m) = eye(m) * u_max;
    end
    W = [mins; maxes];
    E = [-eye(m*N_mpc); eye(m*N_mpc)]; % negatives for minimums, positives for maximums
    
    % Simulate

end