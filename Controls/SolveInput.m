%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function computes the optimal input at the current timestep by
% re-linearizing LQR at the current estimated state x0. We then find an input
% u0 such that Ax0 + Bu0 = 0 and solve for our optimal Gain Matrix K. We
% then add this u0 input as a feedforward signal. 
%
% By: Pablo Plata   -   10/22/25
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [K, u] = SolveInput(x0, x_ref, u0)

% Calculate the J_u Jacobian [B matrices] at thrust up
B = JacobianU([x0; zeros(3,1)], u0);
B = B(1:12,:);

% Solve for u0 such that Ax0 + Bu0 = 0
%   invp(B) is the Moore-Penrose pseudoinverse of B
%   check Eigen availability of this function, prob avaiable under SVD.
xdot = nominalDynamics([x0; zeros(3,1)], zeros(4,1));
u0 = -pinv(B) * xdot(1:12);

% Calculate the J_x Jacobian [A matrices] at current state
A = JacobianX([x0; zeros(3,1)], u0);
A = A(1:12,1:12);

% Define Q and R matrices for LQR using Bryson's Rule
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [2, 2, 0.08, 1000, 1000, 1000, 0.8, 0.8, 2, 2, 2, 10];
max_u = [pi/30, pi/30, 6, 0.5];

Q = eye(size(A,1)) .* a_weights ./ max_x.^2;
R = eye(size(B,2)) .* b_weights ./ max_u.^2;

% Solve the LQR problem and compute gain matrix. 
K = SolveLQR(A, B, Q, R);

% Compute optimal input
u = -K * (x0 - x_ref) + u0;

% Input saturation
thrust_max = 1.5 * 9.8;
maxU = [pi/24; pi/24; thrust_max; thrust_max * 10];
minU = [-pi/24; -pi/24; thrust_max * 0.4; -thrust_max * 10];
u = max(minU, u);
u = min(maxU, u);
