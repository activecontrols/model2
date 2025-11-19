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

% Define the bounds for the operating conditions
thrustMax = 1.5 * 9.8;   %N
gimbalMax = pi/36;
InputBounds = [-gimbalMax       gimbalMax;
               -gimbalMax       gimbalMax;
               .4 * thrustMax   thrustMax;
               -pi/8            pi/8];

% Euler Angle Limits
MaxTilt = pi/12;
YawBounds = [-MaxTilt MaxTilt];
PitchBounds = [-MaxTilt MaxTilt];
RollBounds = [-pi/4 pi/4];

% Other State Limits (Position and Velocity don't affect linearization)
PosBounds = zeros(3,2);
VelBounds = zeros(3,2);
MaxRate = pi/6;
RateBounds = [-MaxRate MaxRate;
              -MaxRate MaxRate;
              -MaxRate MaxRate];

% Final State Bounds
StateBounds = [RollBounds; PitchBounds; YawBounds; PosBounds; VelBounds; RateBounds];

%% Initial State 
x0 = zeros(15,1);
u0 = [0; 0; 1.5*9.8; 0];
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
numSamples = 100;
StateVec = zeros(15, numSamples);
InputVec = zeros(4,  numSamples);

% Pre-allocate space for results using initial run
DM_MonteCarlo = repmat(DM, 1, numSamples);
MM_MonteCarlo = repmat(MM, 1, numSamples);
TimePerSample = 0.0073;       %min    
fprintf(['Started a %i sample Monte Carlo Sim!\n' ...
         'Expected completion time: %.2f min\n'], numSamples, TimePerSample * numSamples);
for i = 1:numSamples
    StateVec(:, i) = [SampleBounds(StateBounds(:, 1), StateBounds(:, 2)); zeros(3,1)];
    InputVec(:, i) = SampleBounds(InputBounds(:, 1), InputBounds(:, 2));

    % Transform Euler Angles to Q_Vec
    Q = eul2quat(StateVec(1:3, i)', 'XYZ');
    StateVec(1:3, i) = Q(2:4)';

    % Relinearize System
    A = JacobianX(StateVec(:,i), InputVec(:,i));
    B = JacobianU(StateVec(:,i), InputVec(:,i));
    A = A(1:12, 1:12);
    B = B(1:12, :);
    C = eye(12);
    D = zeros(12, 4);

    % Perform Stability Analysis using Disk Margins
    % Plant TF
    P = ss(A, B, C, D);

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
    DM_MonteCarlo(:,i) = DM;
    MM_MonteCarlo(i) = MM;
    if mod(i, 25) == 0
        fprintf('%.2f %% done with Monte Carlo Stability Simulation\n', i / numSamples * 100);
    end
end

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
title('Gain Margin Distribution [dB]');

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
threshold = 0.4;

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
xlim([min(diskMarginArray), max(diskMarginArray)]);
grid on;

% The ECDF value for x=0.4 is the count of points <= 0.4 divided by N.
countThreshold = sum(diskMarginArray <= threshold);
probThreshold = countThreshold / N;

% Plot the horizontal and vertical lines for visualization
hold on;
xline(threshold, 'r--', 'LineWidth',1);
yline(probThreshold, 'r--', 'LineWidth',1);
scatter(0.4, probThreshold, 50, 'r', 'filled');
hold off;
fprintf('Cumulative Probability of Disk Margin being below %.2f is: %.2f%%\n', threshold, probThreshold * 100);