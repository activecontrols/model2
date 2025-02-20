function [lin, linDis, plant, output] = dynamics(x, u, x_dot)
    % initals
    % consts = [m,l,g,rTB,J].';
    % Jval = eye(3)
    % constsVal = [20;2;10;1;Jval(:)]
    % xdot = subs(xdot, consts, constsVal); % Pluts in constants
    
    % function outs = nonlinearDynamics(x_dot, y, x, u)
    % Given from data
    % T = Timestep
    % Given from EoMs.m
    % x_dot = f(x, u, w)
    % y = g(x, u, w)
    % Debateable 
    % linearization points
    % delx, delu, delw 
    delx = ones(size(x));
    delu = ones(size(u));
    % delw = zeros(size(w));
    
    
    % Notation
    % x_dot = f(x, u)
    % y = g(x, u)
    
    % Linear relations
    % Takes the jacobian of the state dyanmics and output and subsitutes in
    % the values at our desired point of linearization.
    % NOTE: replace "[x; u], [delx; delu]" with "[x; u; w], [delx; delu;
    % delw]" when disturbance is added
    lin.A = subs(jacobian(x_dot, x), [x; u], [delx; delu]); % Jacobian of f with respect to x
    lin.B = subs(jacobian(x_dot, u), [x; u], [delx; delu]); % Jacobian of f with respect to u
    % lin.C = subs(jacobian(y, x), [x; u], [delx; delu]);   % Jacobian of g with respect to x
    % lin.D = subs(jacobian(y, u), [x; u], [delx; delu]);   % Jacobian of g with respect to u
    
    % Until Output is added we have modeled it as being able to directly
    % observe the states
    lin.C = eye(size(lin.A));
    lin.D = zeros(size(lin.B));
    
    
    % Discrete Linear
    % Creates a system object using continuous matrices and converts them to
    % discrete form using the c2d function with time step T
    sysLin = ss(lin.A, lin.B, lin.C, lin.D);
    sysDis = c2d(sysLin, T);
    
    linDis.Ad = sysDis.A;
    linDis.Bd = sysDis.B;
    % linDis.Cd = sysDis.C;
    % linDis.Dd = sysDis.D;
    
    % Output linear and discrete functions for matlab. Use matlabFunciton to
    % get nonlinear plant model
    plant = matlabFunction(x_dot, "Vars",[{x}, {u}]);
    output = matlabFunction(y, "Vars", [{x}, {u}]);
    
    matlabFunctionBlock("genSym/Plant/Plant State", x_dot_eq, "Vars", [{x}, {u}], "Outputs", {'x_dot'});
    matlabFunctionBlock("genSym/Plant/Plant Output", y, "Vars", [{x}, {u}], "Outputs", {'y'});
end