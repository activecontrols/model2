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
dataFolder = 'Nov 11 Tests';

% File list
path = fullfile(pwd, dataFolder);
list = dir(fullfile(path, '*.txt'));

% Get the number of files found
numFiles = length(list);

% Initialize an empty struct array to store the results
allData = struct('filename', {}, 'z_vec', {}, 'x_vec', {}, 'u_vec', {}, 'trg', {}, 'MET', {}, ...
              'GND', {}, 'ARMED', {}, 'THRUST', {}, 'DIFFY', {}, ...
              'events', struct('Arm', [], 'Kill', [], 'Finished', []));
validData = allData;

% Manually set the two first files
list(1).name = 'log_2025-11-22_13-08-09.txt';
list(2).name = 'log_2025-11-22_13-08-39.txt';
countARM = 0;
for k = 1:1:numFiles
    
    % Get the filename
    filename = list(k).name;
    allData(k).filename = filename;

    % Read the file
    rawString = fileread(filename);
    lengthStr = size(rawString, 2);
    cleanText = regexprep(rawString, '\', '');

    %% Find Arm, Kill, Finished events.
    [~, packetLocs] = regexp(rawString, '>a');
    
    % Initialize index arrays
    idxArm = [];
    idxKill = [];
    idxFinished = [];
    
    if ~isempty(packetLocs)
        % Find Arm Events
            [~, locsArm] = regexp(rawString, '[\n\s>]y[\n\s]');
            for m = 1:length(locsArm)
                % Map to nearest previous data packet
                idxArm(end+1) = sum(packetLocs < locsArm(m));
            end
        
        % Find Kill Events
            [~, locsKill] = regexp(rawString, '[\n\s>]k[\n\s]');
            for m = 1:length(locsKill)
                idxKill(end+1) = sum(packetLocs < locsKill(m));
            end
        
        % Find Finished events
            [~, locsFin] = regexp(rawString, 'Finished following trajectory!'); 
            for m = 1:length(locsFin)
                idxFinished(end+1) = sum(packetLocs < locsFin(m));
            end
    end
    
    % Assign arrays to the specific struct fields
    allData(k).events.Arm = idxArm;
    allData(k).events.Kill = idxKill;
    allData(k).events.Finished = idxFinished;

    %% Measurement vector stream
        zVecTokens = regexp(cleanText, '>a([\d\.\-\s]+)', 'tokens');
        % Convert cells of text to a numeric matrix
        if ~isempty(zVecTokens)
            z_vec = char(join([zVecTokens{:}]));
            z_vec = zeroPad(z_vec, 15);
            allData(k).z_vec = z_vec;
        else
            allData(k).z_vec = [];
        end

    %% Estimated state stream
        xVecTokens = regexp(cleanText, '>b([\d\.\-\s]+)', 'tokens');
        % Convert cells of text to a numeric matrix
        if ~isempty(xVecTokens)
            x_vec = char(join([xVecTokens{:}]));
            x_vec = zeroPad(x_vec, 16);
            allData(k).x_vec = x_vec;
        else
            allData(k).x_vec = [];
        end

    %% Input vector stream
        uVecTokens = regexp(cleanText, '>c([\d\.\-\s]+)', 'tokens');
        % Convert cells of text to a numeric matrix
        if ~isempty(uVecTokens)
            u_vec = char(join([uVecTokens{:}]));
            u_vec = zeroPad(u_vec, 7);
            allData(k).u_vec = u_vec(:, 1:4);
            allData(k).trg = u_vec(:,5:end);
        else
            allData(k).u_vec = [];
            allData(k).trg = [];
        end

    %% Processed data stream
        pVecTokens = regexp(cleanText, '>d([\d\.\-\s]+)', 'tokens');
        % Convert cells of text to a numeric matrix
        if ~isempty(pVecTokens)
            finalStream = char(join([pVecTokens{:}]));
            finalStream = zeroPad(finalStream, 5);
            allData(k).MET = finalStream(:, 1);
            allData(k).GND = finalStream(:, 2);
            allData(k).ARMED = finalStream(:, 3);
            allData(k).THRUST = finalStream(:, 4);
            allData(k).DIFFY = finalStream(:, 5);
        else
            allData(k).MET = [];
            allData(k).GND = [];
            allData(k).ARMED = [];
            allData(k).THRUST = [];
            allData(k).DIFFY = [];
        end
    
    %% Keep only tests with a flight arming
        if ~isempty(find(allData(k).ARMED, 1))
            countARM = countARM + 1;
            validData(countARM) = allData(k);
        end
end

%% DATA ANALYSIS
% figure;
% %title('Roll Angle vs. Time after armed');
% %xlabel('Samples after arming');
% %ylabel('Roll Angle [deg]');
% for i = 5:1:countARM
%     % Euler Angle extraction
%     startIDX = find(validData(i).ARMED, 1);
%     quatVec = validData(i).x_vec(startIDX:end,1:4);
%     eulerAngles = quat2eul(quatVec, 'XYZ');
% 
%     % First Plot (Euler Angle vs. Sample)
%     L = size(validData(i).z_vec(startIDX:end, 1), 1);
%     t = linspace(0, 1, L) * 15;
%     % plot(eulerAngles(:, 2) * 180 / pi); hold on; grid on;
%     plot(t, validData(i).z_vec(startIDX:end, 1));  hold on; grid on;
%     % plot(validData(i).u_vec(startIDX:end, 3)); hold on; grid on;
%     xlim([0 15]);
%     pause(1);
% end




