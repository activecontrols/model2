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

function [lin, linDis, plantState, plantOutput] = dynamics(x, u, x_dot, constants)
    % Linearized around static vertical position
    delx = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
    delu = [0; 0; constants.m * constants.g; 0];
    
    % Linear relations
    % Takes the jacobian of the state dyanmics and output and subsitutes in
    % the values at our desired point of linearization.
    % NOTE: replace "[x; u], [delx; delu]" with "[x; u; w], [delx; delu;
    % delw]" when disturbance is added
    lin.A = eval(subs(jacobian(x_dot, x), [x; u], [delx; delu])); % Jacobian of f with respect to x
    lin.B = eval(subs(jacobian(x_dot, u), [x; u], [delx; delu])); % Jacobian of f with respect to u
    % lin.C = eval(subs(jacobian(y, x), [x; u], [delx; delu]));   % Jacobian of g with respect to x
    % lin.D = eval(subs(jacobian(y, u), [x; u], [delx; delu]));   % Jacobian of g with respect to u
    

    % Until Output is added we have modeled it as being able to directly
    % observe the states
    lin.C = eye(size(lin.A));
    lin.D = zeros(size(lin.B));
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
    
    % Output linear and discrete functions for matlab. Use matlabFunciton to
    % get nonlinear plant model
    plantState = matlabFunction(x_dot, "Vars",[{x}, {u}]);
    plantOutput = matlabFunction(y, "Vars", [{x}, {u}]);
    
    % Createas a MATLAB 
    matlabFunctionBlock("genSym/Plant/Plant State", x_dot, "Vars", [{x}, {u}], "Outputs", {'x_dot'});
    matlabFunctionBlock("genSym/Plant/Plant Output", y, "Vars", [{x}, {u}], "Outputs", {'y'});
end