function K = Controller2_Gen(constants)
%% Generation script for v2 Controls (LQRi Step, Third Loop)
% Symbolic Variables
syms q0 q1 q2 q3            % earth-body quaternion
syms omega1 omega2 omega3   % earth-body angular velocity
syms theta phi;             % thrust angles
syms thrust                 % thrust magnitude
syms roll                   % roll input (rad/sec^2)
syms m l g rTB              % system constants
syms J [3 3]                % Intertia Matrix

%% Equations of Motion
%Full quaternion vector
q = [q0; q1; q2; q3];

%Angular velocity (taken wrt E-frame, represented in B-frame) 
omegaB = [omega1; omega2; omega3];

%Body frame thrust vector
T_vec = [cos(theta)*sin(phi); -sin(theta); cos(theta)*cos(phi)];
TB = thrust * T_vec;

% Torque vector
MB = zetaCross([0; 0; rTB])*TB;
M = [q(1) -q(2) -q(3) -q(4);
     q(2)  q(1) -q(4)  q(3);
     q(3)  q(4)  q(1) -q(2);
     q(4) -q(3)  q(2)  q(1)];

% Dynamics
qdot = 0.5 * M * [0; omegaB];
omegaBdot = J^(-1) * (MB - zetaCross(omegaB) * J * omegaB) + (T_vec * roll);

% State vectors
x = [q; omegaB];
xdot = [qdot; omegaBdot];
u = [theta; phi; roll];

% Substitude constants in
constVal = [constants.m; constants.l; constants.g; constants.rTB; constants.J(:);
            constants.m * constants.g];
constVec = [m; l; g; rTB; J(:); thrust];

% Subsitute numeric values of constants into xdot
xdot = subs(xdot, constVec, constVal);

%% Linearization and Controller Generaiton
lin.A = jacobian(xdot, x);
lin.B = jacobian(xdot, u);

% Map to 6 states.
T = [zeros(1,6); eye(6)];
T(1:4,1:3) = 0.5 * [zeros(1,3); eye(3)];
lin.A = pinv(T) * lin.A * T;
lin.B = pinv(T) * lin.B;

% Linearization point
delx = [1; 0; 0; 0; 0; 0; 0];
delu = [0; 0; 0];
lin.A = double(subs(lin.A, [x; u], [delx; delu]));
lin.B = double(subs(lin.B, [x; u], [delx; delu]));

% Augment system with Error-Integral
lin.A = [lin.A zeros(6,3);
         eye(3) zeros(3,6)];
lin.B = [lin.B; zeros(3)];

% Hand tuning for Q for now
a_weights = ones(6,1);
a_weights = a_weights / norm(a_weights);
max_x = [0.25, 0.25, 0.25, 3, 3, 1.0];
Q = eye(6) .* a_weights ./ max_x.^2;
R = diag([10, 10, 0.2]);

% Augment Q with integral states
Qi = diag([0.01, 0.01, 0.0000002]);
Q = [Q zeros(6,3);
     zeros(3,6) Qi];

% Initial Controller Solution
K = lqr(lin.A, lin.B, Q, R);
end

