%% ASTRAv2 Data Redux script
% Run LoadSimulation.m first to load in constants and gain matrices for
% analysis
%
% Pablo Plata   -   11/22/25
function matrix = zeroPad(stream, numCols)
    rawVec = sscanf(stream, '%f');
    residual = mod(length(rawVec), numCols);
    if residual > 0
        paddingCount = numCols - residual;
        rawVec = [rawVec; zeros(paddingCount, 1)];
        fprintf('Warning: Padded data with %d zeros to complete the last row.\n', paddingCount);
    end
    matrix = reshape(rawVec, numCols, [])';
end

%% Begin Redux
dataFolder = 'Flight Tests\Nov 22 Tests';

% File list
path = fullfile(pwd, dataFolder);
addpath(path);
list = dir(fullfile(path, '*.txt'));

% Get the number of files found
numFiles = length(list);

% Initialize an empty struct array to store the results
testData = struct('filename', {}, 'Time', {}, 'z_vec', {}, 'x_vec', {}, 'u_vec', {}, 'trg', {}, 'MET', {}, ...
              'GND', {}, 'ARMED', {}, 'THRUST', {}, 'DIFFY', {}, ...
              'events', struct('Arm', [], 'Kill', [], 'Finished', []));

% Manually set the two first files
list(1).name = 'log_2025-12-06_14-18-57.txt';
list(2).name = 'log_2025-12-06_14-21-21.txt';
totalTests = 0;
for k = 1:1:numFiles
    
    % Get the filename
    filename = list(k).name;

    % Read the file
    rawString = fileread(filename);
    lengthStr = size(rawString, 2);
    cleanText = regexprep(rawString, '\', '');

    %% Find Arm, Kill, Finished events.
    [~, packetLocs] = regexp(rawString, '>a');
    
    % Initialize index arrays
    idxArm = [];    idxKill = [];   idxFinished = [];
    
    if ~isempty(packetLocs)
        % Find Arm Events
            [~, locsArm] = regexp(rawString, '[\r\n]y[\r\n]');
            for m = 1:length(locsArm)
                idxArm(end+1) = sum(packetLocs < locsArm(m)) + 1;
            end
        
        % Find Kill Events
            [~, locsKill] = regexp(rawString, '[\n\s>]k[\n\s]');
            for m = 1:length(locsKill)
                idxKill(end+1) = max(1, sum(packetLocs < locsKill(m)));
            end
        
        % Find Finished events
            [~, locsFin] = regexp(rawString, 'Finished following trajectory!'); 
            for m = 1:length(locsFin)
                idxFinished(end+1) = max(1, sum(packetLocs < locsFin(m)));
            end
    end

    %% Measurement vector stream
        zVecTokens = regexp(cleanText, '>a([\d\.\-\s]+)', 'tokens');
        % Convert cells of text to a numeric matrix
        if ~isempty(zVecTokens)
            z_vec = char(join([zVecTokens{:}]));
            z_vec = zeroPad(z_vec, 15);
        else
            z_vec = [];
        end

    %% Estimated state stream
        xVecTokens = regexp(cleanText, '>b([\d\.\-\s]+)', 'tokens');
        % Convert cells of text to a numeric matrix
        if ~isempty(xVecTokens)
            x_vec = char(join([xVecTokens{:}]));
            x_vec = zeroPad(x_vec, 16);
        else
            x_vec = [];
        end

    %% Input vector stream
        uVecTokens = regexp(cleanText, '>c([\d\.\-\s]+)', 'tokens');
        % Convert cells of text to a numeric matrix
        if ~isempty(uVecTokens)
            rawU = char(join([uVecTokens{:}]));
            rawU = zeroPad(rawU, 7);
            u_vec = rawU(:, 1:4);
            trg = rawU(:,5:end);
        else
            u_vec = []; trg = [];
        end

    %% Processed data stream
        pVecTokens = regexp(cleanText, '>d([\d\.\-\s]+)', 'tokens');
        % Convert cells of text to a numeric matrix
        if ~isempty(pVecTokens)
            finalStream = char(join([pVecTokens{:}]));
            finalStream = zeroPad(finalStream, 5);
            MET = finalStream(:, 1);
            GND = finalStream(:, 2);
            ARMED = finalStream(:, 3);
            THRUST = finalStream(:, 4);
            DIFFY = finalStream(:, 5);
        else
            MET = []; GND = []; ARMED = []; THRUST = []; DIFFY = [];
        end
    
    %% Split files into single test cycles
        dataLen = size(z_vec, 1);

        % Define Split Boundaries: 0, [Kill Indices], [Finish Indices], End
        stopPoints = sort([idxKill, idxFinished]);
        boundaries = unique([0, stopPoints, dataLen]);
        
        for b = 1:length(boundaries)-1
            startIdx = boundaries(b) + 1;
            endIdx   = boundaries(b+1);
            
            % Basic sanity check: Segment must be long enough (e.g., 10 samples)
            if (endIdx - startIdx) < 10, continue; end
            
            % Only keep this segment if it contains an ARM event
            % Find arm events that occurred strictly within this segment
            segmentArms = idxArm(idxArm >= startIdx & idxArm <= endIdx);
            
            if isempty(segmentArms)
                continue; % Skip this segment, no arming during test cycle
            end
            totalTests = totalTests + 1;
            
            % Helper for safe slicing (handles different stream lengths if serial was buggy)
            safeSlice = @(mat) mat(min(startIdx, size(mat,1)) : min(endIdx, size(mat,1)), :);
            
            testData(totalTests).filename = filename;
            testData(totalTests).OriginalFileIdx = k;
            
            testData(totalTests).z_vec  = safeSlice(z_vec);
            testData(totalTests).x_vec  = safeSlice(x_vec);
            testData(totalTests).u_vec  = safeSlice(u_vec);
            testData(totalTests).trg    = safeSlice(trg);
            testData(totalTests).MET    = safeSlice(MET);
            testData(totalTests).GND    = safeSlice(GND);
            testData(totalTests).ARMED  = safeSlice(ARMED);
            testData(totalTests).THRUST = safeSlice(THRUST);
            testData(totalTests).DIFFY  = safeSlice(DIFFY);
            
            % Re-index Events relative to this specific test segment
            testData(totalTests).events.Arm      = segmentArms - startIdx + 1;
            testData(totalTests).events.Kill     = idxKill(idxKill >= startIdx & idxKill <= endIdx) - startIdx + 1;
            testData(totalTests).events.Finished = idxFinished(idxFinished >= startIdx & idxFinished <= endIdx) - startIdx + 1;
            
            % Use Mission Elapsed Time [MET] to form a T+ time vector
            idxArm = testData(totalTests).events.Arm(1);
            flightMET = testData(totalTests).MET(idxArm:end);
            dt = median(diff(flightMET));
            totalSamples = length(testData(totalTests).MET);
            indices = (1:totalSamples)';
            testData(totalTests).Time = (indices - idxArm) * dt;
        end
end

%% DATA ANALYSIS
%% GPS Velocity Scrolling Plot
figure;
for i = 2:totalTests-1
    % GPS Vel Corrector
    GPS_Vel = zeros(size(testData(i).Time, 1), 3);
    for j = 1:size(testData(i).Time, 1)
        q = testData(i).x_vec(j, 1:4);
        Gyros = testData(i).z_vec(j, 4:6);
        GPS_Vel(j, :) = testData(i).z_vec(j, 13:15);
        rGPS = [0 0 0.31];
        R_b2i = quatRot(q)';
        GPS_Vel(j, :) = GPS_Vel(j, :) - (R_b2i * cross(Gyros, rGPS)')';
    end

    plot(testData(i).Time, testData(i).z_vec(:, 11), 'r-', 'LineWidth', 1);  hold on; grid on;
    plot(testData(i).Time, testData(i).x_vec(:, 6),  'b-', 'LineWidth', 1); hold off;
    legend('GPS Pos', 'EKF Pos');
    str = sprintf('GPS.POS vs. Time  ||  Test: %i', i);
    title(str);
    xlabel('Test Cycle Timer [s]');
    ylabel('GPS Pos [m]');
    xlim([-5 15]);
    pause(3);
end

%% Input Scrolling Plot
% Store original gain matrix
K1 = K;   

% Define the bounds for the operating conditions
thrustMax = 1.5 * 9.8;   %N
gimbalMax = pi/24;
InputBounds = [-gimbalMax       gimbalMax;
               -gimbalMax       gimbalMax;
               .4 * thrustMax   thrustMax;
               -pi/6            pi/6];
uMax = InputBounds(:, 2);
uMin = InputBounds(:, 1);

figure;
for i = 2:totalTests-2

    % Local input reconstruction
    arrayLen = size(testData(i).x_vec(:, 2:13), 1);
    uLocalv1 = zeros(4, arrayLen);
    uLocalv2 = uLocalv1;  
    for j = 1:arrayLen
        x_vec = testData(i).x_vec(j, :)';
        x_trg = [zeros(6,1); testData(i).trg(j, :)'; zeros(3,1)];
        uLocalv1(:, j) = -K1 * (x_vec(2:13) - x_trg);
        uLocalv1(:, j) = min(max(uLocalv1(:,j), uMin), uMax);
    end
    eulerAngles = quat2eul(testData(i).x_vec(:,1:4), 'XYZ');
    % plot(testData(i).Time, eulerAngles(:,3) * 180 / pi, 'y', 'LineWidth', 1); hold on; grid on;
    yyaxis left
    accelTot = sqrt(testData(i).z_vec(:,1).^2 + testData(i).z_vec(:,2).^2 + testData(i).z_vec(:,3).^2) / constantsASTRA.g;
    plot(testData(i).Time, accelTot, 'g', 'LineWidth', 1); hold on; grid on;
    ylabel('Accel Reading Norm [G]');
    hold off;
    yyaxis right
    % plot(testData(i).Time, testData(i).u_vec(:,1), 'b', 'LineWidth', 1);
    % plot(testData(i).Time, uLocalv1(4, :) * 180 / pi, 'r', 'LineWidth', 1);
    hold off;
    str = sprintf('Ang. Rate and Gimbal vs. Time  ||  Test: %i', i);
    title(str);
    xlabel('Test Cycle Timer [s]');
    ylabel('Thrust CMD [N]');
    yline(constantsASTRA.m * constantsASTRA.g, 'r--');
    legend('Accel Reading','Thrust', '1 TWR');
    xlim([-2 15]);
    pause(5);
end

%% Gimbal Time Constant Estimation
% Raw Data Ingestion (No Filtering)
master_cmd_segments = {};     
master_accel_vec = [];        
master_dt_list = [];        

fprintf('Loading all raw data...\n');

for i = 2:length(testData)-1
    t = testData(i).Time;
    dt = mean(diff(t));
    
    cmd_cols = [1, 2];
    gyro_cols = [4, 5];
    
    for axis_idx = 1:2
        % Extract Raw Inputs
        raw_cmd = testData(i).u_vec(:, cmd_cols(axis_idx)) * 180/pi;
        raw_gyro = testData(i).z_vec(:, gyro_cols(axis_idx));
        
        % Calculate Acceleration (Central Difference)
        accel = gradient(raw_gyro) ./ dt;
        
        % Store Everything
        master_cmd_segments{end+1} = raw_cmd;
        master_accel_vec = [master_accel_vec; accel];
        master_dt_list(end+1) = dt;
    end
end

fprintf('Loaded %d data points.\n', length(master_accel_vec));

% Optimization Loop (two diffrent metrics)
tau_range = 0.0:0.001:0.3; 
residuals = zeros(size(tau_range));

for k = 1:length(tau_range)
    curr_tau = tau_range(k);

    % Simulate Response for ALL data points
    simulated_angles = [];
    for seg = 1:length(master_cmd_segments)
        cmd_seg = master_cmd_segments{seg};
        dt_seg = master_dt_list(seg);
        alpha = dt_seg / (curr_tau + dt_seg);

        % Filter command to estimate physical angle
        angle_est = filter(alpha, [1 -(1-alpha)], cmd_seg);
        simulated_angles = [simulated_angles; angle_est];
    end

    % Fit Line to ALL data
    p = polyfit(simulated_angles, master_accel_vec, 1);
    y_fit = polyval(p, simulated_angles);

    % Calculate RMSE
    residuals(k) = sqrt(mean((master_accel_vec - y_fit).^2));
end

% Results
[min_err, idx] = min(residuals);
best_tau_RMSE = tau_range(idx);

% Optimization Loop (PCA / Cloud Thinness)
PCA_metric = zeros(size(tau_range));
fprintf('Optimizing for Hysteresis Collapse (Cloud Thinness)...\n');

for k = 1:length(tau_range)
    curr_tau = tau_range(k);
    
    % Simulate Response
    simulated_angles = [];
    for seg = 1:length(master_cmd_segments)
        cmd_seg = master_cmd_segments{seg};
        dt_seg = master_dt_list(seg);
        alpha = dt_seg / (curr_tau + dt_seg);
        angle_est = filter(alpha, [1 -(1-alpha)], cmd_seg);
        simulated_angles = [simulated_angles; angle_est];
    end

    % Normalize data (Z-Score) so Angle (0.15) and Accel (40) define shape equally
    % If we don't do this, PCA just sees a vertical line because 40 >> 0.15
    x_norm = (simulated_angles - mean(simulated_angles)) / std(simulated_angles);
    y_norm = (master_accel_vec - mean(master_accel_vec)) / std(master_accel_vec);
    data_matrix = [x_norm, y_norm];
    
    % Calculate Singular Value Decomposition (SVD)
    s = svd(data_matrix);
    
    % s(1) is the length of the diagonal (Signal)
    % s(2) is the width of the oval (Hysteresis + Noise)
    PCA_metric(k) = s(2) / s(1); 
end

% Results
[min_PCA, idx] = min(PCA_metric);
best_tau = tau_range(idx);

% Reconstruct best fit
final_angles = [];
for seg = 1:length(master_cmd_segments)
    dt_seg = master_dt_list(seg);
    alpha = dt_seg / (best_tau + dt_seg);
    final_angles = [final_angles; filter(alpha, [1 -(1-alpha)], master_cmd_segments{seg})];
end

% Final Slope Calculation
p_final = polyfit(final_angles, master_accel_vec, 1);

fprintf('---------------------------------\n');
fprintf('Lag (Tau):           %.3f s\n', best_tau);
fprintf('Control Authority:   %.3f rad/s^2 per deg\n', p_final(1));

% Plot
figure; hold on; grid on;
scatter(final_angles, master_accel_vec, 10, 'g', 'filled', 'MarkerFaceAlpha', 0.25);

% Plot Fit
x_range = linspace(min(final_angles), max(final_angles), 100);
plot(x_range, polyval(p_final, x_range), 'r', 'LineWidth', 2);
xlabel('Estimated Gimbal Angle [rad]'); 
ylabel('Angular Acceleration [rad/sec^2]'); 
title(['Raw Characterization (Tau = ' num2str(best_tau) 's)']);
legend('Raw Flight Data', ['Fit Slope: ' num2str(p_final(1))]);

figure; 
plot(tau_range, PCA_metric / max(PCA_metric), 'LineWidth', 2); hold on;
plot(tau_range, residuals / max(residuals), 'LineWidth', 2);
xlabel('Tau (s)'); ylabel('Normalized Metrics');
title('Gimbal Path Delay Identification');
grid on;
Colors = colororder('glow');
xline(best_tau,'--', ['Best Fit PCA: ' num2str(best_tau)], 'Color', Colors(1, :));
legend('Minor / Major Axis Ratio for Data Cloud', 'Squared Residuals from linear Fit', 'Best Fit');

