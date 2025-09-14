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

% MPC Loop
for k = 1:N
    % Linearize about current state
    
end