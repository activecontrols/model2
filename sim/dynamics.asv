% Generates the EoMs
% The EoMGenerator function is an input free function that builds the
% Equations of Motion (EoMs) which are derived within this function. The
% function is designed to output the following values
%
% Outputs:
%   x    - symbolic vector of states
%   u    - symbolic vector of inputs
%   xdot - symbolic vector of the system dynamics
%
% [NOT IMPLEMENTED YET! Need more info on ASTRA] 
%   w    - symbolic vector of disturbances          
%           Blocker: Modes of Disturbance Analysis
%   y    - symbolic vector of outputs               
%           Blocker: ASTRA sensor design

function [lin, linDis, plant, output] = dynamics(x, u, x_dot, constants)
    % Debateable 
    % linearization points
    delx = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
    delu = [0; 0; 0; 0];
    
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
    plant = matlabFunction(x_dot, "Vars",[{x}, {u}]);
    output = matlabFunction(y, "Vars", [{x}, {u}]);
    
    % Createas a MATLAB 
    matlabFunctionBlock("genSym/Plant/Plant State", x_dot, "Vars", [{x}, {u}], "Outputs", {'x_dot'});
    matlabFunctionBlock("genSym/Plant/Plant Output", y, "Vars", [{x}, {u}], "Outputs", {'y'});
end