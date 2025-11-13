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
plotChannel = 1; 
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

title(sprintf('Averaged FFT vs. Frequency and Thrust (Gaps as Zeros, Channel %d)', plotChannel));
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

disp('Binned 3D surface plot with gaps generated.\n');

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
title(sprintf('Waterfall FFT vs. Frequency and Thrust (EVERY TEST) (Channel %d)', plotChannel));
xlabel('Thrust');
ylabel('Frequency (Hz)');
zlabel('Amplitude');
ylim([10 max(freq_interp)]);

grid on;
view(3); % Set the default 3D viewing angle
% view(70, 45); % This is often a good angle for waterfall plots

% Optional: Limit the Y-axis (frequency) to zoom in
% ylim([0, 50]); 
disp('Waterfall plot generated. \n');

%% Line Fits for S-ANF
% nonZeroThrust = find(binnedAmplitudeMatrix(1,:) > 0);
% FFT_Matrix = binnedAmplitudeMatrix(:, nonZeroThrust);
% thrust_levels = integerThrustAxis(nonZeroThrust);
% 
% % Frequency vector
% frequencies = freq_interp; % Full frequency vector for interpolation
% 
% % Tuned parameters for better tracking
% prominence_factor = 0.01;   % Lower to detect more peaks
% min_peak_distance = 20;     % In frequency bins; kept from original
% freq_window = 60;           % Base window for adjacent steps
% initial_freqs = [50, 140];  % Updated seeds: 50 Hz and 200 Hz at low thrust
% init_tolerance = 50;        % Hz tolerance for initial peak matching (increased slightly)
% min_freq = 15;              % New: Minimum frequency to consider for peaks (ignore 0-15 Hz)
% 
% % Compute typical step for handling gaps/valleys
% if length(thrust_levels) > 1
%     typical_step = median(diff(thrust_levels));
% else
%     typical_step = 10; % Fallback assumption
% end
% 
% % Step 1: Find peaks per thrust level (all prominent ones, sorted by amp descending)
% peaks_per_level = cell(length(thrust_levels), 1);
% for i = 1:length(thrust_levels)
%     FFT = FFT_Matrix(:, i);
%     max_amp = max(FFT);
% if max_amp <= 0, continue; end % Skip empty slices
%     [pks, locs] = findpeaks(FFT, 'MinPeakProminence', max_amp * prominence_factor, ...
%     'MinPeakDistance', min_peak_distance, ...
%     'SortStr', 'descend');
% if ~isempty(locs)
%         peak_freqs = frequencies(locs); % Interpolate to Hz
%         peak_amps = pks;
%         % Filter out peaks below min_freq
%         valid_idx = peak_freqs >= min_freq;
%         peak_freqs = peak_freqs(valid_idx);
%         peak_amps = peak_amps(valid_idx);
%         if ~isempty(peak_freqs)
%             peaks_per_level{i} = [peak_freqs(:), peak_amps(:)]; % [freq, amp] pairs (column vectors)
%         end
% end
% end
% % Step 2: Track peaks across levels with prediction for continuity
% tracks = {}; % Cell array of tracks, each: N x 3 matrix [thrust, freq, amp]
% 
% % Initialize from first thrust level (lowest), closest to initial_freqs
% first_peaks = peaks_per_level{1};
% if ~isempty(first_peaks)
%     for j = 1:length(initial_freqs)
%         dists = abs(first_peaks(:,1) - initial_freqs(j));
%         [min_dist, idx] = min(dists);
%         if min_dist < init_tolerance
%             tracks{end+1} = [thrust_levels(1), first_peaks(idx,1), first_peaks(idx,2)];
%         end
%     end
% end
% 
% % Track sequentially through remaining levels
% for i = 2:length(thrust_levels)
%     current_peaks = peaks_per_level{i};
%     if isempty(current_peaks), continue; end
%     % Adjust window for gaps (valleys with no data)
%     delta_thrust = thrust_levels(i) - thrust_levels(i-1);
%     freq_window_eff = freq_window * (delta_thrust / typical_step);
%     assignments = zeros(length(tracks), 2); % Temp [freq, amp] for each track
%     used_idx = []; % To avoid double-assigning peaks
%     for k = 1:length(tracks)
%         track_data = tracks{k};
%         if size(track_data, 1) >= 2
%             % Extrapolate from last two points
%             last_two_thrust = track_data(end-1:end, 1);
%             last_two_freq = track_data(end-1:end, 2);
%             slope = (last_two_freq(2) - last_two_freq(1)) / (last_two_thrust(2) - last_two_thrust(1));
%             predicted_freq = last_two_freq(2) + slope * (thrust_levels(i) - last_two_thrust(2));
%         else
%             predicted_freq = track_data(end, 2);
%             slope = 0; % No trend yet
%         end
%         dist = abs(current_peaks(:,1) - predicted_freq);
%         candidates_idx = find(dist < freq_window_eff & ~ismember((1:size(current_peaks,1))', used_idx));
%         if ~isempty(candidates_idx)
%             % Pick the candidate with highest amp
%             [~, best] = max(current_peaks(candidates_idx, 2));
%             best_idx = candidates_idx(best);
%             assignments(k, :) = current_peaks(best_idx, :);
%             tracks{k} = [tracks{k}; thrust_levels(i), assignments(k,1), assignments(k,2)];
%             used_idx = [used_idx; best_idx];
%         end
%     end
% end
% % Step 3: Fit curves to each track (example: linear fit with polyfit)
% fits = cell(length(tracks), 1); % Cell of fit params per track (e.g., for linear: [slope, intercept])
% for k = 1:length(tracks)
%     track_data = tracks{k};
%     if size(track_data, 1) < 3, continue; end % Need at least 3 points for reliable fit
%     x = track_data(:, 1); % Thrust
%     y = track_data(:, 2); % Freq
%     p = polyfit(x, y, 1); % Linear fit; change to 2 for quadratic, or use fit() for other models
%     fits{k} = p;
% end
% % Display results
% disp('Tracked Lines (thrust, freq, amp):');
% for k = 1:length(tracks)
%     disp(['Track ' num2str(k) ':']);
%     disp(tracks{k});
% end
% disp('Linear Fits (slope, intercept):');
% for k = 1:length(fits)
%     if ~isempty(fits{k})
%         disp(['Track ' num2str(k) ': ' num2str(fits{k})]);
%     end
% end
% 
% % 6. RE-Create the 3D SURFACE plot
% figure;
% colormap(turbo);
% surf(integerThrustAxis, freq_interp, binnedAmplitudeMatrix, 'EdgeColor', 'none');
% idx_10Hz = find(freq_interp > 10,1);
% max_ampl = max(max(binnedAmplitudeMatrix(idx_10Hz:end, :)));
% clim([0, 0.4 * max_ampl]);
% 
% % Use shading interp for smooth coloring
% shading interp;
% 
% title(sprintf('Binned Surface vs. Frequency and Thrust (Gaps as Zeros, Signal %d)', plotChannel));
% xlabel('Thrust (Rounded)');
% ylabel('Frequency (Hz)');
% zlabel('Amplitude');
% 
% colorbar;
% view(3);
% grid on;
% ylim([10 max(freq_interp)]);
% zlim([0 1.2 * max_ampl])
% 
% hold on;
% colors = lines(length(tracks));
% for k = 1:length(tracks)
%     track_data = tracks{k};
%     plot3(track_data(:,1), track_data(:,2), track_data(:,3)*1.1, 'o-', 'Color', [0.6 0.6 0.6], 'LineWidth', 2); % Offset amp slightly for visibility
% end
% hold off;
% xlabel('Thrust (Rounded)');
% ylabel('Frequency (Hz)');
% zlabel('Amplitude');
% title('Tracked Peaks Over FFT Surface');