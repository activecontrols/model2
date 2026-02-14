% Load the Simulink model
LoadSimulation;
load_system("SimulationLoop.slx");

% Run the simulation
simOut = sim("SimulationLoop.slx");

% Extract state timeseries
stateTimeSeries = simOut.state_log;
MEKFsTimeSeries = simOut.MEKF_state;
MEKFpTimeSeries = simOut.MEKF_P;
inputTimeSeries = simOut.inputCMD;
QerrTimeSeries = simOut.AttErrorControl;

%% Post-Processing & Plotting Script for ASTRAv2 Report
% Fixes:
% 1. Draws Hover Boxes at ALL non-zero checkpoints.
% 2. Fixes "weird" attitude plots by enforcing shortest-rotation.
% 3. UPDATED: Adds 3-Sigma Covariance bounds to Gyro Bias plot.

% --- 1. Data Extraction & Formatting ---
% Helper to ensure (Time x Data) format regardless of input shape
formatData = @(ts) correctDims(ts.Data, length(ts.Time));

% Extract Time
t = stateTimeSeries.Time;

% Extract Data
x_true  = formatData(stateTimeSeries);  % Truth
x_est   = formatData(MEKFsTimeSeries);  % Estimate
P_diag  = formatData(MEKFpTimeSeries);  % Covariance
u_cmd   = formatData(inputTimeSeries);  % Inputs
q_err_raw = formatData(QerrTimeSeries); % Quat Error

% --- 2. Variable Mapping ---
% Truth: Indices 1:4=Quat, 5:7=Pos
q_true   = x_true(:, 1:4);
pos_true = x_true(:, 5:7);

% Estimate: Indices 1:4=Quat, 5:7=Pos, 11:13=GyroBias
q_est    = x_est(:, 1:4);
pos_est  = x_est(:, 5:7);
bias_est = x_est(:, 11:13);

% Covariance: 
% 1:3=Attitude, 4:6=Position, 7:9=Velocity, 10:12=GyroBias
sig3_att  = 3 * sqrt(P_diag(:, 1:3));
sig3_pos  = 3 * sqrt(P_diag(:, 4:6));
sig3_bias = 3 * sqrt(P_diag(:, 10:12)); % New: Gyro Bias Covariance

% Controls: 1=Theta, 2=Phi, 3=Thrust
gimbal_theta = u_cmd(:, 1);
gimbal_phi   = u_cmd(:, 2);
thrust_cmd   = u_cmd(:, 3);

% --- 3. Calculations ---

% 3.1 Reconstruct Target Attitude
q_target = zeros(size(q_true));
for k = 1:length(t)
    q_T = q_true(k, :)';
    q_E = q_err_raw(k, :)';
    
    H = HamiltonianProd(q_T); 
    q_target(k, :) = (H * q_E)';
end

% 3.2 Calculate Attitude Error (for Plot 2)
att_err_angle = zeros(length(t), 3);
for k = 1:length(t)
    % Conjugate of estimate (scalar first)
    q_E_conj = [q_est(k,1); -q_est(k,2:4)']; 
    q_T = q_true(k, :)';
    
    H_inv = HamiltonianProd(q_E_conj);
    q_diff = H_inv * q_T;
    
    % Enforce shortest rotation (Sign Flip)
    if q_diff(1) < 0
        q_diff = -q_diff;
    end
    
    % Normalize
    q_diff = q_diff / norm(q_diff);
    
    % Small angle approximation: alpha = 2 * vector_part
    att_err_angle(k, :) = 2 * q_diff(2:4)';
end

% 3.3 Euler Angles (for Plot 4)
eul_true = quat2eul(q_true, 'ZYX');
eul_targ = quat2eul(q_target, 'ZYX');

% --- Plot 1: Mission Trajectory ---
figure('Name', 'Figure 5.1: Mission Trajectory', 'Color', 'w', 'Position', [100 100 900 600]);
hold on; grid on; axis equal; view(3);

N = pos_true(:,1); W = pos_true(:,2); U = pos_true(:,3);

% 1. Checkpoints
plot3(Checkpoints(1,:), Checkpoints(2,:), Checkpoints(3,:), 'r--s', ...
    'LineWidth', 1.5, 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');

% 2. Hover Boxes
for i = 1:size(Checkpoints, 2)
    cp_curr = Checkpoints(:, i);
    if cp_curr(3) > 0.1
        v_cube = 1/2 * [0.5 -0.5 -0.5; 0.5 0.5 -0.5; -0.5 0.5 -0.5; -0.5 -0.5 -0.5; ...
                        0.5 -0.5 0.5; 0.5 0.5 0.5; -0.5 0.5 0.5; -0.5 -0.5 0.5];
        v_cube = v_cube + cp_curr'; 
        [f_cube, ~] = convhull(v_cube);
        patch('Vertices', v_cube, 'Faces', f_cube, 'FaceColor', 'g', ...
              'FaceAlpha', 0.1, 'EdgeColor', 'g');
    end
end

% 3. Trajectory Surface
surface([N, N], [W, W], [U, U], [t, t], ...
        'FaceColor', 'no', 'EdgeColor', 'interp', 'LineWidth', 2);
colormap('turbo'); cb = colorbar; cb.Label.String = 'Time [s]';

xlabel('North [m]'); ylabel('West [m]'); zlabel('Altitude [m]');
title('3D Mission Trajectory'); legend('Checkpoints', 'Hover Zones');

% --- Plot 2: Estimator Performance (Updated) ---
figure('Name', 'Figure 5.2: Estimator Performance', 'Color', 'w', 'Position', [150 150 1000 800]);

% Position Error
subplot(3,1,1); hold on; grid on;
plot(t, pos_est - pos_true, 'LineWidth', 1.5);
% plot(t, sig3_pos, 'k--', 'LineWidth', 1);
% plot(t, -sig3_pos, 'k--', 'LineWidth', 1);
ylabel('Pos Error [m]'); legend('N', 'W', 'U'); title('Position Error');

% Attitude Error
subplot(3,1,2); hold on; grid on;
plot(t, rad2deg(att_err_angle), 'LineWidth', 1.5);
% plot(t, rad2deg(sig3_att), 'k--', 'LineWidth', 1);
% plot(t, -rad2deg(sig3_att), 'k--', 'LineWidth', 1);
ylabel('Att Error [deg]'); legend('\alpha_x', '\alpha_y', '\alpha_z'); title('Attitude Error MEKF');

% Bias Error & Convergence (UPDATED)
subplot(3,1,3); hold on; grid on;
% Calculate Bias Error (Estimate - Truth)
% gyroBias is 1x3, so we subtract it from every row of bias_est
bias_error = bias_est - gyroBias; 

% Plot Error
cols = ['b', 'r', 'y']; % Standardizing colors
plot(t, bias_error, 'LineWidth', 1.5);

% Plot 3-Sigma Bounds (Black Dashed)
% plot(t, sig3_bias, 'k--', 'LineWidth', 1);
% plot(t, -sig3_bias, 'k--', 'LineWidth', 1);

ylabel('Bias Error [rad/s]'); xlabel('Time [s]'); 
legend('\beta_x Error', '\beta_y Error', '\beta_z Error');
title('Gyro Bias Estimation Error vs Covariance');

% --- Plot 4: Controller Performance ---
figure('Name', 'Figure 5.4: Controller Performance', 'Color', 'w', 'Position', [200 200 1000 700]);

% Attitude Tracking
subplot(2,1,1); hold on; grid on;
plot(t, rad2deg(eul_targ(:,3)), 'r--', 'LineWidth', 1.5);
plot(t, rad2deg(eul_true(:,3)), 'r', 'LineWidth', 1);
plot(t, rad2deg(eul_targ(:,2)), 'g--', 'LineWidth', 1.5);
plot(t, rad2deg(eul_true(:,2)), 'g', 'LineWidth', 1);
ylabel('Angle [deg]'); legend('Roll Cmd', 'Roll True', 'Pitch Cmd', 'Pitch True');
title('Attitude Tracking');

% Actuators
subplot(2,1,2); hold on; grid on;
yyaxis left; plot(t, rad2deg(gimbal_theta), 'b'); plot(t, rad2deg(gimbal_phi), 'c'); ylabel('Gimbal [deg]');
yyaxis right; plot(t, thrust_cmd, 'k'); ylabel('Thrust [N]');
xlabel('Time [s]'); legend('\theta', '\phi', 'Thrust');
title('Actuator Effort');

%% --- Helper Function ---
function out = correctDims(data, timeLen)
    % 1. Squeeze out singleton dimensions (handles 1xNxM)
    sqData = squeeze(data);
    
    % 2. Orient so Time is the first dimension
    sz = size(sqData);
    if sz(1) == timeLen
        out = sqData;         % Already N x M
    elseif sz(2) == timeLen
        out = sqData';        % Transpose to N x M
    else
        error('Data dimension mismatch: No dimension matches time vector length.');
    end
end