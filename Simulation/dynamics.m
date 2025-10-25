% Converts the EoMs into the Dynamics
% The dynamics function converts the manually derived EoM into the linear
% and discretized versions for other functions to use. This function also
% generates the nonlinear plant functions for x_dot & y that can be used by
% other functions and as simulink blocks.
%
% Outputs:
%   lin         - structure containing state space matrices (A,B,C,D)
%   linDis      - structure containing discrete state space matrices
%   plantState  - nonlinear plant state function
%   plantOutput - nonlinear plant output function
%
% [NOT IMPLEMENTED YET! Need more info on ASTRA] 
%   w - not included in linearization
%   y - currently uses y = eye(n)*x

function [lin, linDis] = dynamics(x, u, x_dot, constants)
    
    % Linearized around static vertical position
    delx = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    delu = [0; 0; constants.m * constants.g; 0];
    
    % Linear relations
    % Takes the jacobian of the state dyanmics and output and subsitutes in
    % the values at our desired point of linearization.
    % NOTE: replace "[x; u], [delx; delu]" with "[x; u; w], [delx; delu;
    % delw]" when disturbance is added
    lin.A = double(subs(jacobian(x_dot, x), [x; u], [delx; delu])); % Jacobian of f with respect to x
    lin.B = double(subs(jacobian(x_dot, u), [x; u], [delx; delu])); % Jacobian of f with respect to u

    % Numerical functions for Jacobians for Controls.
    matlabFunction(jacobian(x_dot, x), 'File', './Controls/JacobianX.m', 'Vars', [{x}, {u}]);
    matlabFunction(jacobian(x_dot, u), 'File', './Controls/JacobianU.m', 'Vars', [{x}, {u}]);

    % Assumes direct measurement of positions via GPS and angular velocity
    % via gyroscope (In the future, could expand to measure quaterion
    % directly via accelerometer data and DCM).
    % Adds 6 due to bias augmentation
    lin.C = eye(size(lin.A, 1));
    lin.C = lin.C([1:7 10:12], :);
    lin.D = zeros(size(lin.C, 1), size(lin.B, 2));
    y = x;    
    
    % Discrete Linear
    % Creates a system object using continuous matrices and converts them to
    % discrete form using the c2d function with time step T
    sysLin = ss(lin.A, lin.B, lin.C, lin.D);
    sysDis = c2d(sysLin, constants.T);
    
    linDis.Ad = sysDis.A;
    linDis.Bd = sysDis.B;
    linDis.Cd = sysDis.C;
    linDis.Dd = sysDis.D;

    % Update the C matrix to augmented form
    lin.C = eye(size(lin.A, 1) + 3);
    lin.C = lin.C([1:7 10:12], :);
    
    % Output linear and discrete functions for matlab. Use matlabFunciton to
    % get nonlinear plant model
    % plantState = matlabFunction(x_dot, 'File', './sim/lib/plantfcn.m', "Vars",[{x}, {u}]);
    % plantOutput = matlabFunction(y, "Vars", [{x}, {u}]);
    
    % Repeats function generation process for augmented system for EKF
    % [xAUG, u, xAUG_dot] = EoMGenerator(constants, 2);
    % matlabFunction(jacobian(xAUG_dot, xAUG), 'File', './sim/lib/AUG_JacobianX.m', 'Vars', [{xAUG}, {u}]);
    % matlabFunction(jacobian(xAUG_dot, u), 'File', './sim/lib/AUG_JacobianU.m', 'Vars', [{xAUG}, {u}]);
    % plantState = matlabFunction(xAUG_dot, 'File', './sim/lib/AUG_plantfcn.m', "Vars",[{xAUG}, {u}]);


end