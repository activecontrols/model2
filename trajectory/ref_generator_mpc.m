function [t_traj, x_traj, u_traj] = ref_generator_mpc(Checkpoints, HoldTimeReqs, constantsASTRA, dt)
% ref_generator_mpc
%
% Generate a reference trajectory for ASTRA v2 MPC.
%
% CHECKPOINT TYPES (auto-detected):
%   ground  z == 0 : vehicle is on the ground.
%                    Reference thrust = 0, attitude vertical, all rates zero.
%   hold    z >  0 : vehicle hovers in mid-air.
%                    Reference thrust = mg, attitude vertical, all rates zero.
%
% TRAVEL SEGMENT TYPES:
%   normal  : quintic spline between two airborne points. Attitude tracks
%             the required thrust direction (bank angle). Gimbal carries
%             small residual. Attitude is slerp-blended at boundaries.
%   landing : travel segment whose destination is a ground checkpoint.
%             Cruise phase uses the normal quintic spline up to the
%             waypoint directly above the landing site. A separate
%             descent phase then smoothly ramps thrust from mg down to 0
%             while blending attitude back to vertical over t_land seconds.
%             The descent endpoint satisfies: z=0, v=0, omega=0, T=0.
%
% INPUTS
%   Checkpoints    (3 x M)  Each column is [x; y; z] in inertial frame.
%   HoldTimeReqs   (1 x M)  Hold duration (s) at each checkpoint.
%   constantsASTRA          Struct from constructConstants / init script.
%                           Required fields: .m, .g
%   dt             (scalar) Timestep (s).
%
% OUTPUTS
%   t_traj   (1 x N)    Time vector (s).
%   x_traj   (13 x N)   State  [q(4); r(3); v(3); omega(3)].
%   u_traj   (4  x N)   Input  [theta; phi; T; gamma].

%  Constants
m = constantsASTRA.m;
g = constantsASTRA.g;
[~, M] = size(Checkpoints);

% Cruise speed used to set travel segment duration.
% Larger v_avg => larger lateral acceleration => larger bank angle.
v_avg = 1.0;   % m/s
t_min = 1.0;   % s

% Attitude blend width at start/end of each travel segment (s).
% Prevents step-changes in omega at hover<->travel transitions.
t_blend = 0.5; % s

% Duration of the powered descent phase appended to landing segments (s).
% During this phase thrust ramps from mg -> 0 and attitude returns to
% vertical. Should be long enough for the vehicle to actually decelerate;
% the quintic spline already brings v to zero at the landing site so this
% phase mainly handles the thrust and attitude unwinding.
t_land = 2.0;  % s

% Identify ground checkpoints
is_ground = (Checkpoints(3,:) == 0);   % 1 x M logical

% Build segment list
seg_types   = {};   % 'ground' | 'hold' | 'travel' | 'landing'
seg_cp      = [];   % checkpoint index for hold/ground segments
seg_cp_from = [];
seg_cp_to   = [];
seg_dur     = [];

for i = 1:M
    if is_ground(i)
        seg_types{end+1}   = 'ground';                  %#ok<AGROW>
    else
        seg_types{end+1}   = 'hold';                    %#ok<AGROW>
    end
    seg_cp(end+1)          = i;                         %#ok<AGROW>
    seg_cp_from(end+1)     = NaN;                       %#ok<AGROW>
    seg_cp_to(end+1)       = NaN;                       %#ok<AGROW>
    seg_dur(end+1)         = HoldTimeReqs(i);           %#ok<AGROW>

    if i < M
        d = norm(Checkpoints(:,i+1) - Checkpoints(:,i));
        if is_ground(i+1)
            seg_types{end+1} = 'landing';               %#ok<AGROW>
        else
            seg_types{end+1} = 'travel';                %#ok<AGROW>
        end
        seg_cp(end+1)      = NaN;                       %#ok<AGROW>
        seg_cp_from(end+1) = i;                         %#ok<AGROW>
        seg_cp_to(end+1)   = i+1;                       %#ok<AGROW>
        seg_dur(end+1)     = max(d / v_avg, t_min);    %#ok<AGROW>
    end
end

% -------------------------------------------------------------------------
%  Pre-compute step counts
%  For landing segments the descent phase is appended AFTER the quintic
%  spline, so we track the extra steps separately.
% -------------------------------------------------------------------------
n_segs      = numel(seg_types);
seg_steps   = max(1, round(seg_dur / dt));   % quintic / hold steps
land_steps  = zeros(1, n_segs);             % extra descent steps

for s = 1:n_segs
    if strcmp(seg_types{s}, 'landing')
        land_steps(s) = max(1, round(t_land / dt));
    end
end

N      = sum(seg_steps) + sum(land_steps);
t_traj = (0:N-1) * dt;
x_traj = zeros(13, N);
u_traj = zeros(4,  N);

q_vert  = [1; 0; 0; 0];
u_hover = [0; 0; m*g; 0];

% Fill segments
col = 1;

for s = 1:n_segs
    ns   = seg_steps(s);
    ns_l = land_steps(s);

    if strcmp(seg_types{s}, 'ground')
        r_h = Checkpoints(:, seg_cp(s));
        x_traj(:, col:col+ns-1) = repmat([q_vert; r_h; zeros(6,1)], 1, ns);
        u_traj(:, col:col+ns-1) = zeros(4, ns);
        col = col + ns;

    elseif strcmp(seg_types{s}, 'hold')
        r_h = Checkpoints(:, seg_cp(s));
        x_traj(:, col:col+ns-1) = repmat([q_vert; r_h; zeros(6,1)], 1, ns);
        u_traj(:, col:col+ns-1) = repmat(u_hover, 1, ns);
        col = col + ns;

    elseif strcmp(seg_types{s}, 'travel')
        [x_seg, u_seg] = travel_segment( ...
            Checkpoints(:, seg_cp_from(s)), ...
            Checkpoints(:, seg_cp_to(s)),   ...
            ns, dt, t_blend, m, g, q_vert);

        x_traj(:, col:col+ns-1) = x_seg;
        u_traj(:, col:col+ns-1) = u_seg;
        col = col + ns;

    elseif strcmp(seg_types{s}, 'landing')
        r0 = Checkpoints(:, seg_cp_from(s));
        r1 = Checkpoints(:, seg_cp_to(s));    % ground point, z=0

        % --- Phase 1: Quintic cruise from r0 to r1 -----------------------
        % The spline enforces zero velocity and acceleration at r1, so the
        % vehicle arrives with v=0 and the thrust needed equals mg*z_hat
        % (pure hover). The descent phase then unwinds from there to T=0.
        [x_cruise, u_cruise] = travel_segment( ...
            r0, r1, ns, dt, t_blend, m, g, q_vert);

        x_traj(:, col:col+ns-1) = x_cruise;
        u_traj(:, col:col+ns-1) = u_cruise;
        col = col + ns;

        % --- Phase 2: Powered descent  (thrust mg -> 0, q -> vertical) --
        %
        % State at the start of descent: position r1, v=0, omega=0, q~vertical.
        % We simply ramp thrust linearly from mg to 0 and hold attitude
        % vertical. No position change (velocity is already zero at r1 and
        % zero thrust means zero net acceleration once we remove the thrust,
        % but we are ON the ground so forces don't matter — this phase is
        % purely about giving the MPC a smooth reference to track as the
        % motors spin down).
        %
        % Thrust ramp:  T(k) = mg * (1 - k/ns_l),  k = 0..ns_l-1
        k_vec   = 0:ns_l-1;
        T_ramp  = m*g * (1 - k_vec / ns_l);       % mg down to ~0

        x_desc  = repmat([q_vert; r1; zeros(6,1)], 1, ns_l);
        u_desc  = [zeros(1,ns_l); zeros(1,ns_l); T_ramp; zeros(1,ns_l)];

        x_traj(:, col:col+ns_l-1) = x_desc;
        u_traj(:, col:col+ns_l-1) = u_desc;
        col = col + ns_l;

    end
end

end % ref_generator_mpc



function [x_seg, u_seg] = travel_segment(r0, r1, ns, dt, t_blend, m, g, q_vert)
    %  travel_segment shared by 'travel' and the cruise phase of 'landing'
    %
    % Generates one travel segment using a quintic position spline. Attitude
    % is derived from the required thrust direction and slerp-blended to
    % vertical at segment boundaries.
    
    T_seg = ns * dt;
    
    % Quintic polynomial in normalised time tau in [0,1)
    tau = ((0:ns-1) * dt) / T_seg;
    p   =   10*tau.^3 - 15*tau.^4 +  6*tau.^5;
    dp  =   30*tau.^2 - 60*tau.^3 + 30*tau.^4;
    ddp =   60*tau    - 180*tau.^2 + 120*tau.^3;
    
    dr    = r1 - r0;
    r_seg = r0 + dr .* p;
    v_seg = dr .* dp   / T_seg;
    a_seg = dr .* ddp  / T_seg^2;
    
    % Required inertial thrust  F_i = m*(a + g*z_hat)
    F_i    = m * (a_seg + repmat([0;0;g], 1, ns));
    T_arr  = sqrt(sum(F_i.^2, 1));
    T_safe = max(T_arr, 1e-6);
    
    % Desired body-z direction = normalised F_i
    z_b_des = F_i ./ T_safe;
    
    % Slerp blend factor: 0 at edges, 1 in the middle
    n_blend = min(round(t_blend / dt), floor(ns/2));
    blend   = ones(1, ns);
    if n_blend > 0
        ramp                      = (0:n_blend-1) / n_blend;
        blend(1:n_blend)          = ramp;
        blend(ns-n_blend+1:ns)    = fliplr(ramp);
    end
    
    % Per-step quaternion: sweep from vertical to banked pose
    q_seg = zeros(4, ns);
    for k = 1:ns
        q_banked   = zaxis_align_quat(z_b_des(:,k));
        q_seg(:,k) = quat_sweep(q_vert, q_banked, blend(k));
    end
    
    % Angular velocity via finite differences  omega_b = 2*(q^-1 ⊗ dq/dt)
    omega_seg = zeros(3, ns);
    for k = 1:ns
        if k < ns
            dq = (q_seg(:,k+1) - q_seg(:,k)) / dt;
        else
            dq = (q_seg(:,k) - q_seg(:,k-1)) / dt;
        end
        omega_full    = 2 * quatmultiply(quatconj(q_seg(:,k).'), dq.').';
        omega_seg(:,k) = omega_full(2:4);
    end
    
    % Gimbal angles: residual between slerp-blended attitude and exact thrust
    theta_seg = zeros(1, ns);
    phi_seg   = zeros(1, ns);
    for k = 1:ns
        Cib          = quatRot(q_seg(:,k));          % C_i^b
        f_b          = Cib * (F_i(:,k) / T_safe(k));
        theta_seg(k) = asin(clamp(-f_b(2), -1, 1));
        cos_th       = cos(theta_seg(k));
        phi_seg(k)   = asin(clamp(f_b(1) / max(abs(cos_th), 1e-6), -1, 1));
    end
    
    x_seg = [q_seg; r_seg; v_seg; omega_seg];
    u_seg = [theta_seg; phi_seg; T_arr; zeros(1,ns)];
end


% Local helpers
function q = zaxis_align_quat(z_hat_b)
    % Shortest-arc quaternion rotating [0;0;1] to z_hat_b.
    z_hat_b = z_hat_b / norm(z_hat_b);
    z_i     = [0; 0; 1];
    d       = clamp(dot(z_i, z_hat_b), -1, 1);
    if abs(d - 1) < 1e-10,  q = [1;0;0;0];  return;  end
    if abs(d + 1) < 1e-10,  q = [0;1;0;0];  return;  end
    angle = acos(d);
    ax    = cross(z_i, z_hat_b);  ax = ax / norm(ax);
    q     = [cos(angle/2); sin(angle/2)*ax];
end

function q_out = quat_sweep(q0, q1, t)
    % Spherical linear interpolation.  t=0 -> q0, t=1 -> q1.
    if dot(q0, q1) < 0,  q1 = -q1;  end
    c = clamp(dot(q0, q1), -1, 1);
    if c > 0.9995
        q_out = (q0 + t*(q1-q0));  q_out = q_out / norm(q_out);  return
    end
    half  = acos(c);
    q_out = (sin((1-t)*half)*q0 + sin(t*half)*q1) / sin(half);
end

function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end