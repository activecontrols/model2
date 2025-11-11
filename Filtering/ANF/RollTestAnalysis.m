clear;
addpath('Filtering\ANF\Roll Data\');

% File list
list = dir('Filtering\ANF\Roll Data\');

% Get the number of files found
numFiles = length(list);

% Initialize an empty struct array to store the results
allData = struct('filename', {}, 'data', {}, 'thrust', {});

% Loop through each file in the list
allData(1).data = readmatrix('7.5_12.5.csv');
allData(1).thrust = 10;
allData(2).data = readmatrix('8.70_11.20.csv');
allData(2).thrust = 9.95;
for k = 3:1:numFiles
   
    % Get the filename from the list
    filename = list(k).name;

    % Extract thrust data
    nameValues = sscanf(filename, '%f_%f.csv');
    
    % Check if sscanf successfully found two numbers
    if length(nameValues) == 2
        % Calculate the average and store it
        allData(k).thrust = mean(nameValues);
    else
        % If the name doesn't match, store NaN as a placeholder
        warning('Could not parse thrust value from: %s', filename);
        allData(k).thrust = NaN;
    end
    
    % Store the filename in our struct
    allData(k).filename = filename;
    
    % Read the data using readmatrix and store it in the struct
    allData(k).data = readmatrix(filename);
end

%% Fourier Analysis
fourier = struct('filename', {}, 'data', {}, 'freq', {});
for k = 1:numFiles
    % Timestep
    dt = mean(diff(allData(k).data(:,1)), 'omitnan');
    fs = 1 / dt;

    % Perform Fourier Transform on the data
        %PSD Plots
        % fourier(k).filename = allData(k).filename;
        % fourier_k = pwelch(allData(k).data(:,2:end));
        % fourier(k).data = abs(fourier_k);
        % fourier(k).freq = 1:1:size(fourier_k,1);

        %FFT Plots
        fourier_k = fft(allData(k).data(:,2:end));
        L = size(allData(k).data, 1);

        % Post-processing
        fourier_k = abs(fourier_k / L);
        fourier(k).data = fourier_k(1:floor(L/2) + 1,:);
        fourier(k).data(2:end-1,:) = 2 * fourier(k).data(2:end-1,:);
        fourier(k).freq = fs * (0:floor(L/2)) / L;
end

%% 3D Surface Plot (Binned by Integer Thrust, Gaps as Zeros)

% --- Configuration ---
plotChannel = 2; 
numInterpPoints = 1000; % Resolution for the Y-axis (frequency)
% ---------------------

fprintf('Generating binned 3D surface plot for signal %d...\n', plotChannel);

% 1. Create the common frequency axis (Y-axis)
maxFreq = 0;
for k = 1:numFiles
    maxFreq = max(maxFreq, max(fourier(k).freq));
end
freq_interp = linspace(0, maxFreq, numInterpPoints)';

% 2. Create the new integer-based thrust axis (X-axis)
roundedThrusts = round([allData.thrust]);
minThrust = min(roundedThrusts);
maxThrust = max(roundedThrusts);

% This is our new, clean X-axis: [10, 11, 12, 13, ...]
integerThrustAxis = minThrust:maxThrust;
numThrustPoints = length(integerThrustAxis);

% 3. Create the new Amplitude Grid (Z-axis)
% Gaps will remain as zeros by default
binnedAmplitudeMatrix = zeros(numInterpPoints, numThrustPoints);
binCount = zeros(1, numThrustPoints);

% 4. Loop through all files and place them into the correct bin
for k = 1:numFiles
    % Get this file's data
    original_freq = fourier(k).freq;
    if size(fourier(k).data, 2) >= plotChannel
        original_amp = fourier(k).data(:, plotChannel);
    else
        original_amp = zeros(size(original_freq)); % No data
    end
    
    % Interpolate this file's FFT onto the common frequency axis
    interpolated_amp = interp1(original_freq, original_amp, freq_interp, 'linear', 0);
    
    % Find which integer bin this file belongs to
    currentRoundedThrust = roundedThrusts(k);
    
    % Map the thrust value to a column index
    idx = currentRoundedThrust - minThrust + 1;
    
    % Add this file's data to the sum for that bin
    binnedAmplitudeMatrix(:, idx) = binnedAmplitudeMatrix(:, idx) + interpolated_amp;
    
    % Increment the counter for that bin
    binCount(idx) = binCount(idx) + 1;
end

% 5. Average the bins that have multiple files
for j = 1:numThrustPoints
    if binCount(j) > 1
        % If we added 2+ files to this bin, divide by the count
        binnedAmplitudeMatrix(:, j) = binnedAmplitudeMatrix(:, j) / binCount(j);
    end
    % If binCount(j) == 0, the column remains zeros, creating the gap.
end

% 6. Create the 3D SURFACE plot
figure;
% Use surf() for a surface
colormap(turbo);
surf(integerThrustAxis, freq_interp, binnedAmplitudeMatrix, 'EdgeColor', 'none');
idx_10Hz = find(freq_interp > 10,1);
max_ampl = max(max(binnedAmplitudeMatrix(idx_10Hz:end, :)));
clim([0, 0.4 * max_ampl]);

% Use shading interp for smooth coloring
shading interp;

title(sprintf('Binned Surface vs. Frequency and Thrust (Gaps as Zeros, Signal %d)', plotChannel));
xlabel('Thrust (Rounded)');
ylabel('Frequency (Hz)');
zlabel('Amplitude');

colorbar;
view(3);
grid on;

% Optional: Limit the Y-axis (frequency) to zoom in
% ylim([0, 50]);
ylim([10 max(freq_interp)]);
zlim([0 1.2 * max_ampl])

disp('Binned 3D surface plot with gaps generated.');

%% 3D Waterfall Plot (Line Traces)
fprintf('Generating 3D waterfall plot for signal %d...\n', plotChannel);

figure;  % Create a new figure window
hold on; % Hold the plot open so we can add all lines

% 1. Get thrust values and sort them
% (Sorting makes the plot much cleaner to look at)
thrustValues = [allData.thrust];
[sortedThrust, sortIdx] = sort(thrustValues);

% 2. Loop through the *sorted* indices
for k_idx = 1:numFiles
    
    % Get the original, unsorted index 'k'
    k = sortIdx(k_idx);
    
    % --- Create the X, Y, and Z vectors for this single line ---
    
    % X-Axis: A constant thrust value
    % We need a vector of this value, one point for each frequency
    currentThrust = sortedThrust(k_idx);
    x_line = repmat(currentThrust, length(fourier(k).freq), 1);
    
    % Y-Axis: The frequency vector for this specific test
    y_line = fourier(k).freq;
    
    % Z-Axis: The amplitude vector for this specific test
    if size(fourier(k).data, 2) >= plotChannel
        z_line = fourier(k).data(:, plotChannel);
    else
        % File didn't have this signal, plot a flat line at zero
        z_line = zeros(size(y_line));
        warning('File %s did not have signal column %d', allData(k).filename, plotChannel);
    end

    % --- Plot the single 3D line ---
    plot3(x_line, y_line, z_line);
    
end

% 3. Add labels and formatting
hold off; % Release the plot
title(sprintf('Waterfall FFT vs. Frequency and Thrust (Signal %d)', plotChannel));
xlabel('Thrust');
ylabel('Frequency (Hz)');
zlabel('Amplitude');
ylim([10 max(freq_interp)]);

grid on;
view(3); % Set the default 3D viewing angle
% view(70, 45); % This is often a good angle for waterfall plots

% Optional: Limit the Y-axis (frequency) to zoom in
% ylim([0, 50]); 

disp('Waterfall plot generated.');