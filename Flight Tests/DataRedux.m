%% ASTRAv2 Data Redux script
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
clear;
dataFolder = 'Nov 22 Tests';

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
list(1).name = 'log_2025-11-22_13-08-09.txt';
list(2).name = 'log_2025-11-22_13-12-36.txt';
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
            trg = u_vec(:,5:end);
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
for i = 6:totalTests-2
    plot(testData(i).Time, testData(i).z_vec(:, 11), 'r-', 'LineWidth', 1);  hold on; grid on;
    plot(testData(i).Time, testData(i).x_vec(:, 6),  'b-', 'LineWidth', 1); hold off;
    legend('GPS Velocity', 'M-EKF Velocity');
    str = sprintf('GPS.VEL vs. Time  ||  Test: %i', i);
    title(str);
    xlabel('Test Cycle Timer [s]');
    ylabel('GPS Velocity [m/sec]');
    xlim([-5 15]);
    pause(3);
end

%% Input Scrolling Plot
for i = 6:totalTests-2
    subplot(2,1,1);
    plot(testData(i).Time, testData(i).u_vec(:, 1) * 180/pi, 'y-', 'LineWidth',1); hold on; grid on;
    plot(testData(i).Time, testData(i).u_vec(:, 2) * 180/pi, 'b-', 'LineWidth',1);
    hold off;
    legend('Gimbal 1 [Yaw]', 'Gimbal 2 [Pitch]');
    str = sprintf('Gimbal Command vs. Time  ||  Test: %i', i);
    title(str);
    xlabel('Test Cycle Timer [s]');
    ylabel('Gimbal Angle [deg]');
    xlim([-5 15]);

    subplot(2,1,2)
    eulerAngles = quat2eul(testData(i).x_vec(:,1:4), 'XYZ');
    plot(testData(i).Time, eulerAngles(:,1) * 180 / pi, 'y', 'LineWidth', 1); hold on; grid on;
    plot(testData(i).Time, eulerAngles(:,2) * 180 / pi, 'b', 'LineWidth', 1); grid on;
    hold off;
    legend('Yaw', 'Pitch');
    str = sprintf('Attitude Angle vs. Time  ||  Test: %i', i);
    title(str);
    xlabel('Test Cycle Timer [s]');
    ylabel('Angle [deg]');
    xlim([-5 15]);
    pause(20);
end
