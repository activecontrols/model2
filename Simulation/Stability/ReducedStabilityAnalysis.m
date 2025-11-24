
function StateSpace = ActuatorDelay
    %Creates a first order actuator model
    ActuatorModel = cell(4, 1);
    tau = [0.08; 0.08; 0.15; 0.15];

    for i =1:size(tau, 1)
        tau_i = tau(i);
        ActuatorModel{i} = tf(1, [tau_i, 1]);
    end

    % Assemble actuator models
    Delay_MIMO = blkdiag(ActuatorModel{:});
    StateSpace = ss(Delay_MIMO);
end

addpath('.\Filtering');
addpath('.\Filtering\ANF');
addpath('.\Simulation');
addpath('.\Simulation\Disturbances\');
addpath('.\Simulation\Helper\');
addpath('.\Simulation\Stability\');
addpath('.\Simulation\Vehicle Motion\');

%% Initial State
% Build constants array
constantsASTRA = constructConstants;
[x, u2, x_dot] = EoMGenerator(constantsASTRA, 2);
[linSys, disLinSys] = dynamics(x, u2, x_dot, constantsASTRA);
linSys.A = linSys.A(1:12,1:12);
linSys.B = linSys.B(1:12,:);

% Define the bounds for the actuators
thrustMax = 1.5 * 9.8;   
gimbalMax = pi/18;
InputBounds = [-gimbalMax       gimbalMax;
               -gimbalMax       gimbalMax;
               .4 * thrustMax   thrustMax;
               -pi/6            pi/6];

% Load LQR tuning matrices for recomputing
% Brysons Rule for Q and R.
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [3, 3, 0.5, 1000, 1000, 1000, 1, 1, 0.4, pi/8, pi/8, 2];
max_u = [pi/18, pi/18, 6, 0.4];

Q = eye(size(linSys.A,1)) .* a_weights ./ max_x.^2;
R = diag([260, 260, 4, 10]);

[K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);

% First system linearization
x0 = zeros(15,1);
u0 = [0; 0; constantsASTRA.g * constantsASTRA.m; 0];
A = JacobianX(x0, u0);
A = A(1:12, 1:12);
B = JacobianU(x0, u0);
B = B(1:12, :);
C = eye(12);
D = zeros(12, 4);

% Plant TF
P = ss(A, B, C, D);

% Controller TF (set only once)
K_ss = ss(K);

% Feedback TF
L = K_ss * P;

% Delayed Feedback TF
Delay_MIMO_ss = ActuatorDelay;
L = L * Delay_MIMO_ss;

% Digital Filter TF
thrust = u0(3) / thrustMax;
[Filter_TF, ~] = FilterTF_Gen(thrust);
Filter_ss = ss(Filter_TF);
L = L * Filter_ss;

% Disk Margins
[DM, MM] = diskmargin(L);