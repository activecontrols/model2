%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function computes the optimal input at the current timestep by
% re-linearizing LQR at the current estimated state x0. We then find an input
% u0 such that Ax0 + Bu0 = 0 and solve for our optimal Gain Matrix K. We
% then add this u0 input as a feedforward signal. 
%
% By: Pablo Plata   -   10/22/25
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [K, U] = SolveInput(x0, x_ref, u0)
% Input bounds
thrustMax = 1.7 * 9.8;   %N
gimbalMax = pi/18;
InputBounds = [-gimbalMax       gimbalMax;
               -gimbalMax       gimbalMax;
               .4 * thrustMax   thrustMax;
               -pi/6            pi/6];

% Relinearize System
A = JacobianX(x_ref, u0);
B = JacobianU(x_ref, u0);

% Compute input trim for steady state
DeltaU = -pinv(B) * A * (x_ref - x0);
U = u0 + DeltaU;
uMax = InputBounds(:, 2);
uMin = InputBounds(:, 1);
U = min(max(U, uMin), uMax);

% Save Input and relinearize
A = JacobianX(x_ref, U);
B = JacobianU(x_ref, U);
A = A(1:12, 1:12);
B = B(1:12, :);

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
U = -K * (x0(1:12) - x_ref(1:12)) + u0;

% Input saturation
U = min(max(U, uMin), uMax);