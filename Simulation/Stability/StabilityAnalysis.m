%% This script calculates disk margins for ASTRAv2 linearized around operating conditions
% We define a set of bounds for our max expected operating conditions, and
% we then relinearize the system by sampling random points from this set of
% operations and calculating the gain and disk margins for them. We then
% analyze the data looking for the worst disk margin value, any resonance,
% or other
%% RUN LOAD SIM FIRST

function Sample = SampleBounds(Min, Max)
   N = size(Max, 1);
   Sample = Min + (Max - Min) .* rand(N, 1); 
end

function StateSpace = ActuatorDelay
    %Creates a first order actuator model
    ActuatorModel = cell(4, 1);
    tau = [0.05; 0.05; 0.15; 0.15];

    for i =1:size(tau, 1)
        tau_i = tau(i);
        ActuatorModel{i} = tf(1, [tau_i, 1]);
    end

    % Assemble actuator models
    Delay_MIMO = blkdiag(ActuatorModel{:});
    StateSpace = ss(Delay_MIMO);
end

function waitbar_update(hWait, total, doReset)
   persistent count
    % Reset Logic
    if doReset
        count = 0;
        return; % Exit immediately
    end 

    % Update Logic
    if isempty(count)
        count = 0;
    end
    count = count + 1;
    progress = count / total;
    waitbar(progress, hWait, sprintf('Simulation Progress: %.0f%%', progress * 100));
end

% Define the bounds for the operating conditions
thrustMax = 1.5 * 9.8;   %N
gimbalMax = pi/18;
InputBounds = [-gimbalMax       gimbalMax;
               -gimbalMax       gimbalMax;
               .4 * thrustMax   thrustMax;
               -pi/6            pi/6];

% Euler Angle Limits
MaxTilt = pi/60;
YawBounds = [-MaxTilt MaxTilt];
PitchBounds = [-MaxTilt MaxTilt];
RollBounds = [-pi/12 pi/12];

% Other State Limits (Position and Velocity don't affect linearization)
PosBounds = zeros(3,2);
VelBounds = zeros(3,2);
MaxRate = pi/6;
RateBounds = [-MaxRate MaxRate;
              -MaxRate MaxRate;
              -MaxRate MaxRate];

% Final State Bounds
StateBounds = [YawBounds; PitchBounds; RollBounds; PosBounds; VelBounds; RateBounds];

%% Initial State
% Load LQR tuning matrices for recomputing
% Brysons Rule for Q and R.
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [5, 5, 0.06, 1000, 1000, 1000, 0.55, 0.55, 1.5, 2, 2, 3];
max_u = [pi/24, pi/24, 6, 0.4];

Q = eye(size(linSys.A,1)) .* a_weights ./ max_x.^2;
R = eye(size(linSys.B,2)) .* b_weights ./ max_u.^2;

% First system linearization
x0 = zeros(15,1);
u0 = [0; 0; 9.8; 0];
A = JacobianX(x0, u0);
A = A(1:12, 1:12);
B = JacobianU(x0, u0);
B = B(1:12, :);
C = eye(12);
D = zeros(12, 4);

% Plant TF
P = ss(A, B, C, D);

% Controller TF (set only once)
K_ss = ss(K);

% Feedback TF
L = K_ss * P;

% Delayed Feedback TF
Delay_MIMO_ss = ActuatorDelay;
L = L * Delay_MIMO_ss;

% Digital Filter TF
thrust = u0(3) / thrustMax;
[Filter_TF, ~] = FilterTF_Gen(thrust);
Filter_ss = ss(Filter_TF);
L = L * Filter_ss;

% Disk Margins
[DM, MM] = diskmargin(L);

%% Sample random operating states
numSamples = 500;
StateVec = zeros(15, numSamples);
InputVec = zeros(4,  numSamples);
EulerVec = zeros(3,  numSamples);

% Pre-allocate space for results using initial run
DM_MonteCarlo = cell(1, numSamples);
MM_MonteCarlo = cell(1, numSamples); 

% Start simulation
clear waitbar_update
hWait = waitbar(0, 'Running Monte Carlo Simulation...');
dq = parallel.pool.DataQueue;
waitbar_update(hWait, 0, true);
afterEach(dq, @(~) waitbar_update(hWait, numSamples,  false));
parfor i = 1:numSamples

    % Sample a Random State Vector
    tempState = [SampleBounds(StateBounds(:, 1), StateBounds(:, 2)); zeros(3,1)];

    % Transform Euler Angles to Q_Vec
    Quat = eul2quat(tempState(1:3)', 'XYZ');
    EulerVec(:, i) = tempState(1:3);
    tempState(1:3) = Quat(2:4)';

    % Save state
    StateVec(:, i) = tempState;

    % Relinearize System
    A = JacobianX(tempState, u0);
    B = JacobianU(tempState, u0);

    % Compute input trim for steady state
    DeltaU = -pinv(B) * A * (tempState - x0);
    U = u0 + DeltaU;
    uMax = InputBounds(:, 2);
    uMin = InputBounds(:, 1);
    U = min(max(U, uMin), uMax);

    % Save Input and relinearize
    InputVec(:,i) = U;
    A = JacobianX(StateVec(:,i), U);
    B = JacobianU(StateVec(:,i), U);
    A = A(1:12, 1:12);
    B = B(1:12, :);
    C = eye(12);
    D = zeros(12, 4);

    % Perform Stability Analysis using Disk Margins
    % Plant TF
    P = ss(A, B, C, D);

    % Feedback TF
    % K = SolveLQR(A, B, Q, R);
    % K_ss = ss(K);
    L = K_ss * P;

    % Delayed Feedback TF
    Delay_MIMO_ss = ActuatorDelay;
    L = L * Delay_MIMO_ss;

    % Digital Filter TF
    thrust = U(3) / thrustMax;
    [Filter_TF, ~] = FilterTF_Gen(thrust);
    Filter_ss = ss(Filter_TF);
    L = L * Filter_ss;

    % Disk Margins
    [DM, MM] = diskmargin(L);
    DM_MonteCarlo{i} = DM;
    MM_MonteCarlo{i} = MM;
    
    % Update progress
    send(dq, []);
end
fprintf('\nSimulation Complete.\n');
delete(hWait);

% Cell to struct conversion
validIdx = ~cellfun(@isempty, MM_MonteCarlo);

% Flatten the Cell Array into a regular Struct Array
MM_MonteCarlo = [MM_MonteCarlo{validIdx}]; 
DM_MonteCarlo = [DM_MonteCarlo{validIdx}];

%% Plot a histogram of the distribution of MM values
diskMarginArray = zeros(numSamples, 1);
phaseMarginArray = diskMarginArray;
gainMarginArray = diskMarginArray;
freqArray = diskMarginArray;
for i = 1:numSamples
    diskMarginArray(i) = MM_MonteCarlo(i).DiskMargin;
    phaseMarginArray(i) = MM_MonteCarlo(i).PhaseMargin(1);
    freqArray(i) = MM_MonteCarlo(i).Frequency(1);
    gainMarginArray(i) = MM_MonteCarlo(i).GainMargin(2);
end
figure;
subplot(1,2,1);
histogram(gainMarginArray, round(sqrt(numSamples))); grid on;
xlabel('Gain Margin Value');
ylabel('Frequency');
title('Gain Margin Distribution');

subplot(1,2,2);
histogram(phaseMarginArray, round(sqrt(numSamples))); grid on;
xlabel('Low End Phase Margin Value');
ylabel('Frequency');
title('Phase Margin Distribution [deg]');

figure;
subplot(1,2,1);
histogram(diskMarginArray, round(sqrt(numSamples))); grid on;
xlabel('Disk Margin Value');
ylabel('Frequency');
title('Disk Margin Distribution');

subplot(1,2,2);
histogram(freqArray / (2*pi), round(sqrt(numSamples))); grid on;
xlabel('Frequency Value [Hz]');
title('Worst Case Frequency Distribution');

% Total number of data points (Sample Size)
N = numel(diskMarginArray);
threshold = 0.5;

% Sort the data (Crucial step for ECDF)
x_sorted = sort(diskMarginArray);

% Calculate the cumulative probability (F) for each point
F = (1:N)' / N;

% Adjust for the stairstep plot:
x_plot = [x_sorted(1) - 0.001; x_sorted]; % Start slightly before the min value
F_plot = [0; F];

% Plot
figure;
plot(x_plot, F_plot, 'LineWidth', 2, 'Color', [0 0.447 0.741]);
title('Empirical Cumulative Distribution Function (ECDF)');
xlabel('Disk Margin Value (x)');
ylabel('Cumulative Probability F(x)');
xlim([0, max(diskMarginArray)]);
grid on;

% The ECDF value for x=0.4 is the count of points <= 0.4 divided by N.
countThreshold = sum(diskMarginArray <= threshold);
probThreshold = countThreshold / N;

% Plot the horizontal and vertical lines for visualization
hold on;
xline(threshold, 'r--', 'LineWidth',1);
yline(probThreshold, 'r--', 'LineWidth',1);
scatter(threshold, probThreshold, 50, 'r', 'filled');
hold off;
fprintf('Cumulative Probability of Disk Margin being below %.2f is: %.2f%%\n', threshold, probThreshold * 100);

% 3. Angular Rates (States 10-12)
RollRate = StateVec(10, :);
PitchRate = StateVec(11, :);
YawRate = StateVec(12, :);

% Calculate Total Tilt Magnitude (Approximation via RSS of sampled angles in radians)
EulerVec = rad2deg(EulerVec);
Tilt_Total = sqrt(EulerVec(1,:).^2 + ...
                  EulerVec(2,:).^2 + ...
                  EulerVec(3,:).^2);

% Calculate Total Angular Rate Magnitude (RSS of p, q, r in deg/sec)
RollRate = rad2deg(RollRate);
PitchRate = rad2deg(PitchRate);
YawRate = rad2deg(YawRate);
Omega_Total = sqrt(RollRate.^2 + PitchRate.^2 + YawRate.^2);

% 1. Create a regular 2D grid for interpolation
num_grid_points = 50;
Tilt_Grid = linspace(min(Tilt_Total), max(Tilt_Total), num_grid_points);
Omega_Grid = linspace(min(Omega_Total), max(Omega_Total), num_grid_points);
[XX, YY] = meshgrid(Tilt_Grid, Omega_Grid);

% 2. Interpolate the scattered data (Disk Margin) onto the grid
ZZ = griddata(Tilt_Total, Omega_Total, diskMarginArray, XX, YY);

% 3. Plot the surface
figure;
surf(XX, YY, ZZ);

% Make it readable
xlabel('Total Tilt [deg]');
ylabel('Total Angular Rate [deg/s]');
zlabel('Disk Margin Value (\alpha)');
title('Robustness Surface: Disk Margin vs. Flight Condition');
colorbar;
colormap jet
clim([0, 0.7])
view(2); % View from 3D perspective
grid on;