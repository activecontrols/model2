% Reference Trajectory Generator for use with MPC controller
%
% Returns a reference state given the current time in the simulation, 
% current position, target position, and hold time requirements. Also 
% returns reference input

function [x_ref, u_ref] = ref_generator_mpc(t, x, checkpoints, holdTimeReqs, constantsASTRA)
    persistent i
    persistent t_start_hold
    persistent hold_flag


    % Initialize persistent variables
    if isempty(hold_flag)
        hold_flag = false;
    end

    if isempty(i)
        i = 1;
    end

    % Grab values from constants
    m = constantsASTRA.m;
    g = constantsASTRA.g;
    dt = constantsASTRA.dt;

    % Get current checkpoint information
    p_target = checkpoints(:, i);
    dt_hold = holdTimeReqs(i);

    % If done holding position, update target and reset hold flag
    if hold_flag == true && (t - t_start_hold >= dt_hold)
        i = i + 1;
        
        p_hat = 4 * (p_target - x(4:6)) / vecnorm(p_target - x(4:6));
        
        C_IB = quatRot(x(1:4));
        TB = 5 * p_hat;
        FI = C_IB * TB + [0; 0; -m*g];
        v_target = FI / m * dt;

        T_ref = TB;
        
    % If just reached target position, update hold flag and start hold
    elseif hold_flag == false && (p_target - x(4:6) <= 1e-3) % CHANGE HARDCODED TOLERANCE LATER
        hold_flag = true;
        t_start_hold = t;
        v_target = [0; 0; 0;];
        T_ref = m * g;
    end

    % HARDCODED REFERENCE, CHANGE LATER
    x_ref = [1; 0; 0; 0; p_target; v_target; 0; 0; 0];
    u_ref = [T_ref; 0; 0; 0]; % GOING TO START BY SETTING R=0 SO INPUT SHOULDN'T MATTER YET

end