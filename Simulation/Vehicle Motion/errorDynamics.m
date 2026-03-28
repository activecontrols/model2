% Computes the linearized error state dynamics about a given reference
% trajectory and trim point.
%
% Used in the ASTRAv2_MPC controller function. Returns a linearized system
% representing the error-state dynamics
%
% INPUTS:
%   e_x       - error-state
%   e_u       - input error
%   x_ref     - reference state
%   u_ref     - reference input
%   e_x_dot   - error-state dynamics
%   constants - system constants
%
% OUTPUTS:
%   lin_err     - SS matrices for continuous time linearized error-state 
%                 dynamics

function lin_err = errorDynamics(e_x, e_u, x_ref, u_ref, e_x_dot, constants)
    % Linearize about zero error condition
    e_x_trim = zeros(size(e_x));
    e_u_trim = zeros(size(e_u));

    % Take jacobians to generate A and B matrices
    lin_err.A = jacobian(e_x_dot, e_x);
    lin_err.B = jacobian(e_x_dot, e_u);

    % Numerical functions for Jacobians for Controls.
    matlabFunction(lin_err.A, 'File', './Controls/JacobianErrorX.m', 'Vars', [{e_x}, {e_u}, {x_ref}, {u_ref}]);
    matlabFunction(lin_err.B, 'File', './Controls/JacobianErrorU.m', 'Vars', [{e_x}, {e_u}, {x_ref}, {u_ref}]);
    
    % Substitute trim conditions and create C and D matrices
    % This returns a struct with SS matrices as functions of reference
    % trajectory (THIS MIGHT NOT BE NEEDED AT ALL)
    lin_err.A = subs(lin_err.A, [e_x; e_u], [e_x_trim; e_u_trim]);
    lin_err.B = subs(lin_err.B, [e_x; e_u], [e_x_trim; e_u_trim]);
    lin_err.C = eye(size(lin_err.A, 1));
    lin_err.D = zeros(size(lin_err.C, 1), size(lin_err.B, 2));
    % sys_lin_err = ss(lin_err.A, lin_err.B, lin_err.C, lin_err.D);

    % Discretize (Never mind: see derivations about zero order hold assumption)
    % sys_lin_err_dis = c2d(sys_lin_err, constants.T);
    % lin_err_dis.Ad = sys_lin_err_dis.A;
    % lin_err_dis.Bd = sys_lin_err_dis.B;
    % lin_err_dis.Cd = sys_lin_err_dis.C;
    % lin_err_dis.Dd = sys_lin_err_dis.D;
end