%function u_star = generate_trajectory()
dt = 0.1;
T = 10; % horizon
N = T/dt;
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

% Dynamics
constants = struct('m', 10, 'l', 1, 'g', 9.81, 'rTB', 1, 'J', diag([100 100 100]));
[x, u, xdot] = EoMGenerator(constants);
f = matlabFunction(xdot, 'Vars', [x; u]);

% Build sample of reachable states using Monte Carlo Rollout Method
num_rollouts = 1000;
X = cell(1, num_rollouts);
U = cell(1, num_rollouts);

dim_state = length(x_min);
dim_ctrl = length(u_min);
for i = 1:num_rollouts
    % x0 = rand(dim_state, 1) .* (x_max - x_min) + x_min; % Random sample of initial states within bounds
    U{i} = rand(dim_ctrl, N) .* (u_max - u_min + u_min);
    X{i} = rk4_method(f, 1:N, dt, x0, U{i});
end


% Dynamic Programming Loop
V_star = inf * ones(1, N);
for i = 1:length(X) % Initialize terminal costs for each monte carlo sample
    x_N = X{i}(:, N);
    V_star(i, N) = G(x_N);
end


u_star = zeros(length(u_max), N);
for k = N-1:(-1):1
    V_star(k) = inf;  % initialize cost-to-go at step k

    
    for i = 1:numel(X) % all admissable states
        x = X{i}(:, k);

        for j = 1:numel(U)
            u = U{j}(:, k); % u test val
            % x_next = x + dt * f(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12), x(13), u(1), u(2), u(3), u(4)); % Might not be best discretization approach
            
            V_k = l(x, u) + V_star(k+1);

            if V_k < V_star(k)
                V_star(k) = V_k;
                u_star(:, k) = u;
                x0 = x(:, 1);
            end
        end
    end
end

% Generate states for trajectory
x = rk4_method(f, 1:N, dt, x0, u_star);

plot(1:N+1, x(1:3, :))
legend('x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','x11','x12','x13')
%end