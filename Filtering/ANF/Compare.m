clear;
clear DigitalNF;
addpath('Filtering\ANF\Roll Data\');

% File list
list = dir('Filtering\ANF\Roll Data\');

% Get the number of files found
numFiles = length(list);

% Initialize an empty struct array to store the results
allData = struct('filename', {}, 'raw_data', {}, 'filtered_data', {}, 'thrust', {});

% --- NOTE: Your manual assignments (k=1, k=2) are used as-is ---
allData(1).raw_data = readmatrix('7.5_12.5.csv');
allData(1).thrust = 10;
allData(2).raw_data = readmatrix('8.70_11.20.csv');
allData(2).thrust = 9.95;
startFile = 3; % Start the loop from k=3

% Loop through each file to read data
for k = startFile:1:numFiles
   
    % Get the filename from the list
    filename = list(k).name;

    % Extract thrust data
    nameValues = sscanf(filename, '%f_%f.csv');
    
    if length(nameValues) == 2
        allData(k).thrust = mean(nameValues);
    else
        warning('Could not parse thrust value from: %s', filename);
        allData(k).thrust = NaN;
    end
    
    allData(k).filename = filename;
    allData(k).raw_data = readmatrix(filename);
end


%% 1. Filter Data and Perform Fourier Analysis (NEW STEP)
fourier_raw = struct('data', {}, 'freq', {});
fourier_filtered = struct('data', {}, 'freq', {});
plotChannel = 1; % The signal channel to analyze (e.g., Roll Rate)

for k = 1:numFiles
    % Timestep and Fs
    raw_data = allData(k).raw_data;
    dt = mean(diff(raw_data(:,1)), 'omitnan');
    fs = 1 / dt;
    
    % Data to be filtered (assuming Time is Col 1, Signals Col 2 onwards)
    signals = raw_data(:, 2:end);
    
    % Initialize Filtered Data matrix
    filtered_signals = zeros(size(signals));
    
    % Run Filter Loop for the current file
    THRUST = allData(k).thrust;
    GND = false; % Filter is active
    clear DigitalNF;

    for i = 1:size(signals, 1) % Iterate through time steps
        
        % Pass all channels (the entire row) at once
        IN_row = signals(i, :);
        
        % The persistent variables are reset once per file by the 'clear DigitalNF' command.
        
        OUT_row = DigitalNF(IN_row', GND, THRUST, dt); % Note: Transpose to make it N_channels x 1
        
        filtered_signals(i, :) = OUT_row'; % Transpose back for storage
    end
    
    % Store the filtered data
    allData(k).filtered_data = [raw_data(:,1), filtered_signals]; % Time, then filtered signals

    L = size(raw_data, 1);
    
    % Fourier Transform on RAW Data
    fourier_raw_k = fft(signals);
    fourier_raw(k).data = abs(fourier_raw_k(1:floor(L/2) + 1,:)) / L;
    fourier_raw(k).data(2:end-1,:) = 2 * fourier_raw(k).data(2:end-1,:);
    fourier_raw(k).freq = fs * (0:floor(L/2)) / L;

    % Fourier Transform on FILTERED Data
    fourier_filtered_k = fft(filtered_signals);
    fourier_filtered(k).data = abs(fourier_filtered_k(1:floor(L/2) + 1,:)) / L;
    fourier_filtered(k).data(2:end-1,:) = 2 * fourier_filtered(k).data(2:end-1,:);
    fourier_filtered(k).freq = fs * (0:floor(L/2)) / L;
end

%% 2. 3D Surface Plot Comparison (Binned)

% --- Configuration ---
numInterpPoints = 1000; % Resolution for the Y-axis (frequency)
% ---------------------

fprintf('Generating COMPARISON 3D surface plots for signal %d...\n', plotChannel);

% Re-use existing binning logic for the RAW and FILTERED data
% Since both share the same frequency vector, we can bin both simultaneously

% 1. Create the common frequency axis (Y-axis) and Thrust axis (X-axis)
maxFreq = max(fourier_raw(1).freq); % Assuming all have same max freq
freq_interp = linspace(0, maxFreq, numInterpPoints)';
roundedThrusts = round([allData.thrust]);
minThrust = min(roundedThrusts);
maxThrust = max(roundedThrusts);
integerThrustAxis = minThrust:maxThrust;
numThrustPoints = length(integerThrustAxis);

% 2. Create the new Amplitude Grids (Z-axis)
binnedAmpRaw = zeros(numInterpPoints, numThrustPoints);
binnedAmpFiltered = zeros(numInterpPoints, numThrustPoints);
binCount = zeros(1, numThrustPoints); % Only need one counter

% 3. Loop through all files and place them into the correct bin
for k = 1:numFiles
    % Get this file's data
    original_freq = fourier_raw(k).freq;
    
    original_amp = fourier_raw(k).data(:, plotChannel);
    filtered_amp = fourier_filtered(k).data(:, plotChannel);
    
    % Interpolate this file's FFT onto the common frequency axis
    interp_raw = interp1(original_freq, original_amp, freq_interp, 'linear', 0);
    interp_filtered = interp1(original_freq, filtered_amp, freq_interp, 'linear', 0);
    
    % Map the thrust value to a column index
    currentRoundedThrust = roundedThrusts(k);
    idx = currentRoundedThrust - minThrust + 1;
    
    % Add this file's data to the sum for that bin
    binnedAmpRaw(:, idx) = binnedAmpRaw(:, idx) + interp_raw;
    binnedAmpFiltered(:, idx) = binnedAmpFiltered(:, idx) + interp_filtered;
    
    % Increment the counter for that bin
    binCount(idx) = binCount(idx) + 1;
end

% 4. Average the bins
for j = 1:numThrustPoints
    if binCount(j) > 1
        binnedAmpRaw(:, j) = binnedAmpRaw(:, j) / binCount(j);
        binnedAmpFiltered(:, j) = binnedAmpFiltered(:, j) / binCount(j);
    end
end

% 5. Create the COMPARISON 3D SURFACE plots
idx_10Hz = find(freq_interp > 10, 1);
max_ampl_raw = max(max(binnedAmpRaw(idx_10Hz:end, :)));
max_ampl_filt = max(max(binnedAmpFiltered(idx_10Hz:end, :)));

figure;
% --- Top Plot: Unfiltered Data ---
colormap(turbo);
surf(integerThrustAxis, freq_interp, binnedAmpRaw, 'EdgeColor', 'none');
shading interp;
title(sprintf('RAW (UNFILTERED) FFT Surface (Channel %d)', plotChannel));
xlabel('Thrust (Rounded)');
ylabel('Frequency (Hz)');
zlabel('Amplitude');
clim([0, 0.4 * max_ampl_raw]);
ylim([10 max(freq_interp)]);
zlim([0 1.1 * max_ampl_raw]);
grid on;
view(-100, 40);

% --- Bottom Plot: Filtered Data ---
figure;
colormap(turbo);
surf(integerThrustAxis, freq_interp, binnedAmpFiltered, 'EdgeColor', 'none');
shading interp;
title(sprintf('FILTERED FFT Surface (Channel %d)', plotChannel));
xlabel('Thrust (Rounded)');
ylabel('Frequency (Hz)');
zlabel('Amplitude');
clim([0, 0.4 * max_ampl_raw]); % Use the same scale for direct comparison
ylim([10 max(freq_interp)]);
zlim([0 1.1 * max_ampl_raw]);
grid on;
view(-100, 40);

disp('Comparison 3D surface plots generated.');
fprintf(['Max Noise Amplitude Pre-Filter: %.3f \n' ...
         'Max Noise Amplitude Post-Filter: %.3f \n'], max_ampl_raw, max_ampl_filt);

figure;
TestNumber = 36;
Channel = plotChannel + 2;
plot(allData(TestNumber).raw_data(:, 1), allData(TestNumber).raw_data(:, Channel)); hold on;
plot(allData(TestNumber).raw_data(:, 1), allData(TestNumber).filtered_data(:, Channel))
legend('Raw Data', 'ANF-S Filtered')
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')
title('Test Sensor Data Filtered vs. ANF-S Filtered');
fprintf('Plotted Test File: %s \n', allData(TestNumber).filename);