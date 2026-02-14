%% Second iteration of an Autotuner for ASTRAv2. Only tunes the inner loop.
%% Build constants and system
clear;
clear CrossoverMin;
addpath('.\Filtering');
addpath('.\Filtering\ANF');
addpath('.\Simulation');
addpath('.\Simulation\Disturbances\');
addpath('.\Simulation\Helper\');
addpath('.\Simulation\Stability\');
addpath('.\Simulation\Vehicle Motion\');
addpath('.\Parameters\');
constantsASTRA = constructConstants;
[K_Att, linSys] = Controller2_Gen(constantsASTRA);
Delay = ActuatorDelay;

% Digital Filter TF (Using the transfer function of the worst-case digital
% filter we'll have on board)
thrust = 80;
[Filter_TF, ~] = FilterTF_Gen(thrust);
Filter = ss(Filter_TF) * eye(3);

% Initial parameter guess
P = zeros(12,1);
P(1) = 0.7; P(3) = 0.1; P(6) = 0.2;
P(7) = 0.7; P(9) = 0.1;   P(12) = 0.2;
R = eye(3) * 5;

% Lower and upper bounds
LB = -inf(12,1);
UB = inf(12,1);
diagIDX = [1 3 6 7 9 12];
LB(diagIDX) = 0.01;

% Solver function (fmincon) and options
options = optimoptions('fmincon', ...
                       'Algorithm','sqp', ...
                       'Display','iter', ...
                       'DiffMinChange',1e-5, ...
                       'MaxFunctionEvaluations', 1000, ...
                       'TolCon', 1e-2, ...
                       'TolFun', 1e-4, ...
                       'StepTolerance', 1e-3);

% Objective function handle
fun = @(x) CrossoverMin(linSys.A, linSys.B, R, Delay, Filter, x);
constraint = @(x) DiskMarginConstraint(linSys.A, linSys.B, R, Delay, Filter, x);

% Set up solver
[P_Opt, FVal] = fmincon(fun, P, [], [], [], [], LB, UB, constraint, options);

%% Displays
% Display optimized parameters and function value
disp('Optimized Q Matrix:');
Q_Opt = BuildQ(P_Opt);
disp(Q_Opt);
disp('Critical Frequency Value:');
disp(-FVal);
disp('Attitude Gain Matrix:')
[K, ~, ~] = lqr(linSys.A, linSys.B, Q_Opt, R);
disp(K);
[DM, MM, ~] = Margins(linSys.A, linSys.B, K, Delay, Filter);
[DM2, MM2, ~] = Margins(linSys.A, linSys.B, K_Att, Delay, Filter);

% ---- Q MATRIX (copy-paste ready) ----
fprintf('%% ============== COPY FROM HERE DOWN ==============\n');
fprintf('Q = [ ');
for row = 1:9
    fprintf('');                                        % nice indent
    fprintf(' %18.12f', Q_Opt(row,:));                  % 12 decimal places = full double precision
    fprintf(' ;\n');
end
fprintf('];\n\n');

% ---- R MATRIX (usually fixed, but print anyway) ----
fprintf('R = diag([ %g, %g, %g ]);\n\n', diag(R));

%% Functions
function Q = BuildQ(P)
    % Roll Lower Triangular
    L_r = [ P(1)  0     0;
        P(2)  P(3)  0;
        P(4)  P(5)  P(6) ];
    
    % Gimbal Lower Triangular
    L_g = [ P(7)  0     0;
            P(8)  P(9)  0;
            P(10) P(11) P(12) ];
    
    % Compute 3x3 weight matrices which assure Positive Semi-Definite Q
    Q_roll   = L_r * L_r';
    Q_gimbal = L_g * L_g';

    % Roll Elements
    r11 = Q_roll(1,1); r12 = Q_roll(1,2); r13 = Q_roll(1,3);
                       r22 = Q_roll(2,2); r23 = Q_roll(2,3);
                                          r33 = Q_roll(3,3);
                                          
    % Gimbal Elements
    g11 = Q_gimbal(1,1); g12 = Q_gimbal(1,2); g13 = Q_gimbal(1,3);
                         g22 = Q_gimbal(2,2); g23 = Q_gimbal(2,3);
                                              g33 = Q_gimbal(3,3);

    % Diagonal Weights
    Q_qq = diag([g11, g11, r11]);  % Position weights
    Q_ww = diag([g22, g22, r22]);  % Rate weights
    Q_ii = diag([g33, g33, r33]);  % Integral weights
    
    % Off-Diagonal Coupling Weights
    Q_qw = diag([g12, g12, r12]);  % Pos-Rate coupling
    Q_qi = diag([g13, g13, r13]);  % Pos-Integral coupling
    Q_wi = diag([g23, g23, r23]);  % Rate-Integral coupling
    
    % Full Q matrix
    Q = [ Q_qq      Q_qw      Q_qi ;
          Q_qw'     Q_ww      Q_wi ;
          Q_qi'     Q_wi'     Q_ii ];
end
function [DM, MM, L] = Margins(A, B, K, Delay, Filter)
    % Full SS System
    C = eye(size(A, 1));
    D = zeros(size(A, 1), size(B, 2));
    
    % Plant TF
    P = ss(A, B, C, D);
    
    % Controller TF (set only once)
    K_ss = ss(K);
    
    % Feedback TF
    L = K_ss * P;
    
    % Delayed Feedback TF
    Delay_MIMO_ss = Delay;
    L = L * Delay_MIMO_ss;
    L = L * Filter;
    
    % Final Disk Margin structs.
    [DM, MM] = diskmargin(L);
end
function Cross = CrossoverMin(A, B, R, Delay, Filter, P)
    % Penalized Objective: Speed Cost + Robustness Penalty
    Penalty_Factor = 1e4; % Lambda (Adjust this if needed)
    TargetMargin = 0.65;
    
    try
        Q = BuildQ(P);
        [K, ~, ~] = lqr(A, B, Q, R);
        
        % Speed Cost
        Loop = A - B * K;
        Poles_Real = real(eig(Loop));
        sigma_target = -8; 
        Slower_Poles = Poles_Real(Poles_Real > sigma_target);
        
        if isempty(Slower_Poles)
            Speed_Cost = 0.01 * max(Poles_Real); 
        else
            Speed_Cost = sum((Slower_Poles - sigma_target).^2);
        end
        
        % Robustness Penalty
        [~, MM, ~] = Margins(A, B, K, Delay, Filter);
        Disk = MM.DiskMargin;

        % Violation is max(0, TargetMargin - Disk)
        Violation = max(0, TargetMargin - Disk); 
        Robustness_Penalty = Penalty_Factor * Violation;
        Cross = Speed_Cost + Robustness_Penalty;

    catch
        % Massive penalty if LQR fails, driving the solver away
        Cross = 1e10;
    end
end
function [c, ceq] = DiskMarginConstraint(A, B, R, Delay, Filter, P)
    ceq = [];
    try
        Q = BuildQ(P);
        [K, ~, ~] = lqr(A, B, Q, R);
        
        % Final Margin structs.
        [~, MM, ~] = Margins(A, B, K, Delay, Filter);
        Disk = MM.DiskMargin;
    
        % Build constraint
        TargetMargin = 0.65;
        c = TargetMargin - Disk;
    catch
        c = 100;
    end
end
function StateSpace = ActuatorDelay
    %Creates a first order actuator model
    ActuatorModel = cell(3, 1);
    tau = [0.08; 0.08; 0.15];

    for i =1:size(tau, 1)
        tau_i = tau(i);
        ActuatorModel{i} = tf(1, [tau_i, 1]);
    end

    % Assemble actuator models
    Delay_MIMO = blkdiag(ActuatorModel{:});
    StateSpace = ss(Delay_MIMO);
end