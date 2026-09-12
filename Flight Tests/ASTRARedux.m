% Data redux script for ASTRAv2
% Put the folder with the logs here
addpath('Flight Tests\May 2 Tests\');
fileList = dir('Flight Tests\May 2 Tests\*.csv');

for i = 1:length(fileList)
    currentFile = fullfile(fileList(i).folder, fileList(i).name);
    flightData = readtable(currentFile);
    
    Data(i).Time = flightData.elapsed_time;
    
    Data(i).State.Pos = [flightData.state_pos_north, flightData.state_pos_west, flightData.state_pos_up];
    Data(i).State.Vel = [flightData.state_vel_north, flightData.state_vel_west, flightData.state_vel_up];
    Data(i).State.Quat = [flightData.state_q_vec_w, flightData.state_q_vec_x, flightData.state_q_vec_y, flightData.state_q_vec_z];
    
    Data(i).Sensors.Pre.Accel = [flightData.accel_x, flightData.accel_y, flightData.accel_z];
    Data(i).Sensors.Pre.Gyro = [flightData.gyro_roll, flightData.gyro_pitch, flightData.gyro_yaw]; 
    Data(i).Sensors.Pre.Mag = [flightData.mag_x, flightData.mag_y, flightData.mag_z];
    
    Data(i).Sensors.Pre.GPS_Pos = [flightData.gps_pos_north, flightData.gps_pos_west, flightData.gps_pos_up];
    Data(i).Sensors.Pre.GPS_Vel = [flightData.gps_vel_north, flightData.gps_vel_west, flightData.gps_vel_up];
    Data(i).Sensors.Pre.GPS_Stats = [flightData.gps_hor_prec, flightData.gps_ver_prec, flightData.gps_sat_count, flightData.rtk_status];
    
    Data(i).Sensors.Post.Accel = [flightData.filtered_accel_x, flightData.filtered_accel_y, flightData.filtered_accel_z];
    Data(i).Sensors.Post.Gyro = [flightData.filtered_gyro_x, flightData.filtered_gyro_y, flightData.filtered_gyro_z];
    Data(i).Sensors.Post.Mag = [flightData.filtered_mag_x, flightData.filtered_mag_y, flightData.filtered_mag_z];
    
    Data(i).Sensors.Biases.Accel = [flightData.accel_bias_x, flightData.accel_bias_y, flightData.accel_bias_z];
    Data(i).Sensors.Biases.Gyro = [flightData.gyro_bias_roll, flightData.gyro_bias_pitch, flightData.gyro_bias_yaw];
    Data(i).Sensors.Biases.Mag = [flightData.mag_bias_x, flightData.mag_bias_y, flightData.mag_bias_z];
    
    Data(i).Control.Outputs.Thrust_N = flightData.thrust_N;
    Data(i).Control.Outputs.Thrust_Perc = flightData.thrust_perc;
    Data(i).Control.Outputs.Diffy_Perc = flightData.diffy_perc;
    Data(i).Control.Outputs.Roll_RadSec2 = flightData.roll_rad_sec_squared;
    Data(i).Control.Outputs.Gimbal = [flightData.gimbal_pitch_raw, flightData.gimbal_yaw_raw];
    
    Data(i).Control.TRG.Pos = [flightData.target_pos_north, flightData.target_pos_west, flightData.target_pos_up];
    Data(i).Control.TRG.Vel = [flightData.velocity_target_x, flightData.velocity_target_y, flightData.velocity_target_z];
    Data(i).Control.TRG.Accel = [flightData.accel_target_x, flightData.accel_target_y, flightData.accel_target_z];
    Data(i).Control.TRG.Att = [flightData.attitude_target_0_, flightData.attitude_target_1_, flightData.attitude_target_2_, flightData.attitude_target_3_];
    
    Data(i).Control.Integ.Att = [flightData.attitude_integrator_x, flightData.attitude_integrator_y, flightData.attitude_integrator_z];
    Data(i).Control.Integ.Vel = [flightData.velocity_integrator_x, flightData.velocity_integrator_y, flightData.velocity_integrator_z];
    
    Data(i).Flags.GND_flag = flightData.GND_flag;
    Data(i).Flags.Armed = flightData.flight_armed;
    
    Data(i).Covariance.Pos = [flightData.posCovNN, flightData.posCovNE, flightData.posCovND, flightData.posCovEE, flightData.posCovED, flightData.posCovDD];
    Data(i).Covariance.Vel = [flightData.velCovNN, flightData.velCovNE, flightData.velCovND, flightData.velCovEE, flightData.velCovED, flightData.velCovDD];
end
%% 
set(0, 'DefaultFigureWindowStyle', 'docked');

Index = 6;
dt = mean(diff(Data(Index).Time)); 
if dt > 0
    fs = 1 / dt;
else
    fs = 400; 
end

numFlights = length(Data);
windowSize = 256;                  
overlap = floor(windowSize * 0.9); 
nfft = 2048;                       
axisLabels = {'X-Axis', 'Y-Axis', 'Z-Axis'};

figure('Name', sprintf('Spectrograms: Flight %d - Pre-filtered Accel', Index));
for ax = 1:3
    subplot(3, 1, ax);
    sensorData = Data(Index).Sensors.Pre.Accel(:, ax);
    spectrogram(sensorData, kaiser(windowSize, 5), overlap, nfft, fs, 'yaxis');
    title(sprintf('Pre-filtered Accel Spectrogram: %s', axisLabels{ax}));
    colorbar;
end

figure('Name', 'Peak Freq (>10Hz) vs Thrust Command - All Flights');
freqThreshold = 10; 

for ax = 1:3
    subplot(3, 1, ax);
    hold on;
    
    all_thrust = [];
    all_peakFreqs = [];
    
    for f_idx = 1:numFlights
        sensorData = Data(f_idx).Sensors.Pre.Accel(:, ax);
        
        if length(sensorData) > windowSize
            [s, f, t_spec] = spectrogram(sensorData, kaiser(windowSize, 5), overlap, nfft, fs);
            
            validFreqIdx = find(f > freqThreshold);
            validF = f(validFreqIdx);
            validS = abs(s(validFreqIdx, :));
            
            [~, maxIdx] = max(validS, [], 1);
            peakFreqs = validF(maxIdx);
            
            t_abs = t_spec + Data(f_idx).Time(1);
            
            thrustData = Data(f_idx).Control.Outputs.Thrust_Perc;
            thrustInterp = interp1(Data(f_idx).Time, thrustData, t_abs, 'linear', 'extrap');
            
            scatter(thrustInterp, peakFreqs, 15, 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', sprintf('Flight %d', f_idx));
            
            all_thrust = [all_thrust, thrustInterp(:)'];
            all_peakFreqs = [all_peakFreqs, peakFreqs(:)'];
        end
    end
    
    if ~isempty(all_thrust) && ~isempty(all_peakFreqs)
        outliers = isoutlier(all_peakFreqs, 'median');
        clean_thrust = all_thrust(~outliers);
        clean_freqs = all_peakFreqs(~outliers);
        
        if length(clean_thrust) > 1
            p = polyfit(clean_thrust, clean_freqs, 1);
            x_fit = linspace(min(all_thrust), max(all_thrust), 100);
            y_fit = polyval(p, x_fit);
            
            y_pred = polyval(p, clean_thrust);
            sigma = std(clean_freqs - y_pred);
            y_upper = y_fit + 2*sigma;
            y_lower = y_fit - 2*sigma;
            
            R_mat = corrcoef(clean_thrust, clean_freqs);
            rsq = R_mat(1,2)^2;
            
            plot(x_fit, y_fit, 'k-', 'LineWidth', 2, 'DisplayName', sprintf('Fit: y=%.2fx+%.2f (R^2=%.2f)', p(1), p(2), rsq));
            plot(x_fit, y_upper, 'k--', 'LineWidth', 1, 'DisplayName', '+2 Sigma');
            plot(x_fit, y_lower, 'k--', 'LineWidth', 1, 'DisplayName', '-2 Sigma');
        end
    end
    
    title(sprintf('Peak Freq (>10Hz) vs Thrust Command: %s', axisLabels{ax}));
    xlabel('Thrust Command (%)');
    ylabel('Peak Frequency (Hz)');
    ylim([100, 170]);
    grid on;
    hold off;
    
    legend('Location', 'best', 'NumColumns', min(numFlights+3, 5));
end

figure('Name', sprintf('Time Domain Comparison: Flight %d - Accel & Gyro', Index));

% Define time vector for plotting
timeVec = Data(Index).Time; 
for ax = 1:3
    subplot(2, 3, ax); 
    hold on;
    
    preData = Data(Index).Sensors.Pre.Accel(:, ax);
    postData = Data(Index).Sensors.Post.Accel(:, ax);
    
    % Plot Pre-filter (Raw) first so Post-filter overlays on top
    plot(timeVec, preData, 'r', 'DisplayName', 'Pre-filter (Raw)');
    plot(timeVec, postData, 'b', 'LineWidth', 1.5, 'DisplayName', 'Post-filter');
    
    hold off; grid on;
    title(sprintf('Accel %s', axisLabels{ax}));
    xlabel('Time (s)');
    ylabel('Acceleration');
    xlim([timeVec(1), timeVec(end)]);
    
    % Add legend to the last plot in the row
    if ax == 3; legend('Location', 'best'); end
end
for ax = 1:3
    subplot(2, 3, ax + 3); 
    hold on;
    
    preData = Data(Index).Sensors.Pre.Gyro(:, ax);
    postData = Data(Index).Sensors.Post.Gyro(:, ax);
    
    % Plot Pre-filter (Raw) first so Post-filter overlays on top
    plot(timeVec, preData, 'r', 'DisplayName', 'Pre-filter (Raw)');
    plot(timeVec, postData, 'b', 'LineWidth', 1.5, 'DisplayName', 'Post-filter');
    
    hold off; grid on;
    title(sprintf('Gyro %s', axisLabels{ax}));
    xlabel('Time (s)');
    ylabel('Angular Rate');
    xlim([timeVec(1), timeVec(end)]);
    
    % Add legend to the last plot in the row
    if ax == 3; legend('Location', 'best'); end
end

figure('Name', sprintf('PSD Comparison: Flight %d - Accel & Gyro', Index));

for ax = 1:3
    subplot(2, 3, ax); 
    hold on;
    
    preData = Data(Index).Sensors.Pre.Accel(:, ax);
    postData = Data(Index).Sensors.Post.Accel(:, ax);
    
    [pxx_pre, f_pre]   = pwelch(preData, hamming(windowSize), overlap, nfft, fs);
    [pxx_post, f_post] = pwelch(postData, hamming(windowSize), overlap, nfft, fs);
    
    plot(f_pre, 10*log10(pxx_pre), 'r', 'DisplayName', 'Pre-filter (Raw)');
    plot(f_post, 10*log10(pxx_post), 'b', 'LineWidth', 1.5, 'DisplayName', 'Post-filter');
    
    hold off; grid on;
    title(sprintf('Accel %s', axisLabels{ax}));
    xlabel('Frequency (Hz)');
    ylabel('Power/Freq (dB/Hz)');
    xlim([0, fs/2]); 
    if ax == 3; legend('Location', 'best'); end
end

for ax = 1:3
    subplot(2, 3, ax + 3); 
    hold on;
    
    preData = Data(Index).Sensors.Pre.Gyro(:, ax);
    postData = Data(Index).Sensors.Post.Gyro(:, ax);
    
    [pxx_pre, f_pre]   = pwelch(preData, hamming(windowSize), overlap, nfft, fs);
    [pxx_post, f_post] = pwelch(postData, hamming(windowSize), overlap, nfft, fs);
    
    plot(f_pre, 10*log10(pxx_pre), 'r', 'DisplayName', 'Pre-filter (Raw)');
    plot(f_post, 10*log10(pxx_post), 'b', 'LineWidth', 1.5, 'DisplayName', 'Post-filter');
    
    hold off; grid on;
    title(sprintf('Gyro %s', axisLabels{ax}));
    xlabel('Frequency (Hz)');
    ylabel('Power/Freq (dB/Hz)');
    xlim([0, fs/2]);
end

timeVec = Data(Index).Time; 

figure('Name', sprintf('Position Target vs State: Flight %d', Index));
posLabels = {'North (X)', 'West (Y)', 'Up (Z)'};

for ax = 1:3
    subplot(3, 1, ax);
    hold on;
    plot(timeVec, Data(Index).Control.TRG.Pos(:, ax), 'g', 'LineWidth', 1.5, 'DisplayName', 'Target');
    plot(timeVec, Data(Index).State.Pos(:, ax), 'b', 'LineWidth', 1, 'DisplayName', 'State Estimate');
    hold off;
    
    grid on;
    title(sprintf('Position: %s', posLabels{ax}));
    ylabel('Position (m)');
    if ax == 3; xlabel('Time (s)'); end
    legend('Location', 'best');
end

figure('Name', sprintf('Velocity Target vs State: Flight %d', Index));
velLabels = {'North (X)', 'West (Y)', 'Up (Z)'};

for ax = 1:3
    subplot(3, 1, ax);
    hold on;
    plot(timeVec, Data(Index).Control.TRG.Vel(:, ax), 'g', 'LineWidth', 1.5, 'DisplayName', 'Target');
    plot(timeVec, Data(Index).State.Vel(:, ax), 'b', 'LineWidth', 1, 'DisplayName', 'State Estimate');
    hold off;
    
    grid on;
    title(sprintf('Velocity: %s', velLabels{ax}));
    ylabel('Velocity (m/s)');
    if ax == 3; xlabel('Time (s)'); end
    legend('Location', 'best');
end

figure('Name', sprintf('Attitude Target vs State: Flight %d', Index));

stateEuler_rad = quat2eul(Data(Index).State.Quat);
targetEuler_rad = quat2eul(Data(Index).Control.TRG.Att);

stateEuler_deg = rad2deg(stateEuler_rad);
targetEuler_deg = rad2deg(targetEuler_rad);
eulIndices = [3, 2, 1]; 
eulLabels = {'Yaw', 'Pitch', 'Roll'};

for ax = 1:3
    subplot(3, 1, ax);
    hold on;
    
    colIdx = eulIndices(ax);
    
    plot(timeVec, targetEuler_deg(:, colIdx), 'g', 'LineWidth', 1.5, 'DisplayName', 'Target');
    plot(timeVec, stateEuler_deg(:, colIdx), 'b', 'LineWidth', 1, 'DisplayName', 'State Estimate');
    hold off;
    
    grid on;
    title(sprintf('Attitude: %s', eulLabels{ax}));
    ylabel('Angle (deg)');
    if ax == 3; xlabel('Time (s)'); end
    legend('Location', 'best');
end

figure('Name', sprintf('GPS Delay - Velocity: Flight %d', Index));

for ax = 1:3
    subplot(3, 1, ax);
    hold on;
    
    stateVel = Data(Index).State.Vel(:, ax);
    gpsVel = Data(Index).Sensors.Pre.GPS_Vel(:, ax);
    
    plot(timeVec, stateVel, 'b', 'LineWidth', 1.5, 'DisplayName', 'State Estimate (IMU+EKF)');
    plot(timeVec, gpsVel, 'r.-', 'MarkerSize', 6, 'LineWidth', 1, 'DisplayName', 'Raw GPS (Delayed)');
    
    hold off;
    grid on;
    title(sprintf('Velocity Tracking & Delay: %s', velLabels{ax}));
    ylabel('Velocity (m/s)');
    if ax == 3; xlabel('Time (s)'); end
    legend('Location', 'best');
end

figure('Name', sprintf('GPS Delay - Position: Flight %d', Index));

for ax = 1:3
    subplot(3, 1, ax);
    hold on;
    
    statePos = Data(Index).State.Pos(:, ax);
    gpsPos = Data(Index).Sensors.Pre.GPS_Pos(:, ax) - Data(Index).Sensors.Pre.GPS_Pos(1, ax);
    
    plot(timeVec, statePos, 'b', 'LineWidth', 1.5, 'DisplayName', 'State Estimate (IMU+EKF)');
    plot(timeVec, gpsPos, 'r.-', 'MarkerSize', 6, 'LineWidth', 1, 'DisplayName', 'Raw GPS (Delayed)');
    
    hold off;
    grid on;
    title(sprintf('Position Tracking & Delay: %s', posLabels{ax}));
    ylabel('Position (m)');
    if ax == 3; xlabel('Time (s)'); end
    legend('Location', 'best');
end

figure('Name', sprintf('Pure Hardware Delay (Accel DR vs GPS) - Flight %d', Index));

q = Data(Index).State.Quat; 
accel_body = Data(Index).Sensors.Post.Accel;

try
    accel_nav = quatrotate(q, accel_body);
catch
    accel_nav = accel_body; 
end

accel_nav(:, 3) = accel_nav(:, 3) - 9.80145;

cutoff_hp = 0.5; 
[b_hp, a_hp] = butter(2, cutoff_hp / (fs/2), 'high');

maxLag_sec = 2.0; 
maxLag_samples = round(maxLag_sec * fs);

for ax = 1:3
    subplot(3, 1, ax);
    hold on;

    dr_vel_raw = cumtrapz(timeVec, accel_nav(:, ax));
    dr_vel_clean = filtfilt(b_hp, a_hp, dr_vel_raw);
    
    gps_vel_raw = Data(Index).Sensors.Pre.GPS_Vel(:, ax);
    gps_vel_clean = filtfilt(b_hp, a_hp, gps_vel_raw);

    [R, lags] = xcorr(dr_vel_clean, gps_vel_clean, maxLag_samples, 'coeff');
    lag_time = lags * dt;

    [max_R, max_idx] = max(R);
    delay_sec = lag_time(max_idx);
    delay_ms = delay_sec * 1000;

    plot(lag_time, R, 'b', 'LineWidth', 1.5);
    plot(delay_sec, max_R, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Peak Correlation');
    xline(0, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');

    hold off;
    grid on;
    xlim([-2.0, 2.0]); 
    ylim([0, 1.1]);
    
    title(sprintf('Pure Transport Delay %s: %.1f ms', velLabels{ax}, delay_ms));
    ylabel('Correlation (Norm)');
    if ax == 3; xlabel('Lag Time (seconds)'); end
end

figure('Name', 'Velocity Integrators - All Flights');
velIntegLabels = {'X', 'Y', 'Z'};

for ax = 1:3
    subplot(3, 1, ax);
    hold on;
    
    for f_idx = 1:numFlights
        plot(Data(f_idx).Time, Data(f_idx).Control.Integ.Vel(:, ax), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Flight %d', f_idx));
    end
    
    hold off;
    grid on;
    title(sprintf('Velocity Integrator Gain over Time: %s', velIntegLabels{ax}));
    ylabel('Integrator Value');
    if ax == 3; xlabel('Time (s)'); end
    
    if ax == 1 && numFlights <= 15
        legend('Location', 'best', 'NumColumns', min(numFlights, 5));
    end
end

figure('Name', 'Attitude Integrators - All Flights');
attIntegLabels = {'X (Roll)', 'Y (Pitch)', 'Z (Yaw)'}; 

for ax = 1:3
    subplot(3, 1, ax);
    hold on;
    
    for f_idx = 1:numFlights
        plot(Data(f_idx).Time, Data(f_idx).Control.Integ.Att(:, ax), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Flight %d', f_idx));
    end
    
    hold off;
    grid on;
    title(sprintf('Attitude Integrator Gain over Time: %s', attIntegLabels{ax}));
    ylabel('Integrator Value');
    if ax == 3; xlabel('Time (s)'); end
    
    if ax == 1 && numFlights <= 15
        legend('Location', 'best', 'NumColumns', min(numFlights, 5));
    end
end

figure('Name', sprintf('Gimbal Angles: Flight %d', Index));
gimbalLabels = {'Pitch', 'Yaw'}; 

for ax = 1:2
    subplot(2, 1, ax);
    hold on;
    plot(timeVec, Data(Index).Control.Outputs.Gimbal(:, ax), 'b', 'LineWidth', 1.5, 'DisplayName', 'Commanded Angle');
    hold off;
    
    grid on;
    title(sprintf('Gimbal Output: %s', gimbalLabels{ax}));
    ylabel('Angle (deg)');
    if ax == 2; xlabel('Time (s)'); end
    legend('Location', 'best');
end
