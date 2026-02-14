function Plot6DoF(Checkpoints)
    % Load the Simulink model
    LoadSimulation;
    load_system("SimulationLoop.slx");
    
    % Run the simulation
    simOut = sim("SimulationLoop.slx");
    
    % Extract state timeseries
    stateTimeSeries = simOut.state_log;
    t = stateTimeSeries.Time;  % Time vector
    x = stateTimeSeries.Data;  % State matrix (15 x time)
    
    % Extract states (q_vec is 3x1 vector part)
    q = x(1:4, :)';      % [qw qx qy qz]
    r = x(5:7, :)';      % Position [x y z]
    % v = x(7:9, :)';    % Not plotted
    % ang_rate = x(10:12, :)';  % Not plotted
    % gyro_bias = x(13:15, :)'; % Not plotted
    
    % Compute scalar quaternion q0 (assuming unit norm and q0 >= 0)
    % q0 = sqrt(1 - sum(q_vec.^2, 2));
    % q_full = [q0, q_vec(:,1), q_vec(:,2), q_vec(:,3)];  % [w x y z]
    
    % Plot 3D trajectory with color gradient by time (blue to red)
    figure(1);
    colormap('jet');  % Blue at start, red at end; adjust colormap if desired
    surf([r(:,1) r(:,1)], [r(:,2) r(:,2)], [r(:,3) r(:,3)], [t t], ...
         'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', 2);
    cb = colorbar;
    cb.Label.String = 'Time (s)';
    hold on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Trajectory');
    grid on;
    axis equal;

    % Plot checkpoints
    for i = 1:1:size(Checkpoints, 2)
        plot3(Checkpoints(1,i), Checkpoints(2,i), Checkpoints(3,i), "ro", 'MarkerSize',20);
    end
    zlim([0 6]);
    hold off;

    % New separate figure with 3 2D plots for traces (using tiledlayout for reduced spacing)
    figure(2);
    colormap('jet');  % Same colormap
    
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');  % Tighter spacing with 'none'
    
    % XY trace
    nexttile;
    surf([r(:,1) r(:,1)], [r(:,2) r(:,2)], [zeros(size(t)) zeros(size(t))], [t t], ...
         'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', 2);
    view(2);  % 2D view
    xlabel('X');
    ylabel('Y');
    title('XY Trace');
    grid on;
    axis equal;
    
    % XZ trace
    nexttile;
    surf([r(:,1) r(:,1)], [r(:,3) r(:,3)], [zeros(size(t)) zeros(size(t))], [t t], ...
         'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', 2);
    view(2);
    xlabel('X');
    ylabel('Z');
    title('XZ Trace');
    grid on;
    axis equal;
    
    % YZ trace
    nexttile;
    surf([r(:,2) r(:,2)], [r(:,3) r(:,3)], [zeros(size(t)) zeros(size(t))], [t t], ...
         'FaceColor', 'none', 'EdgeColor', 'interp', 'LineWidth', 2);
    view(2);
    xlabel('Y');
    ylabel('Z');
    title('YZ Trace');
    grid on;
    axis equal;
    
    % Add shared colorbar for the figure (slimmer to save space)
    cb = colorbar('Position', [0.93 0.15 0.015 0.7]);  % Slimmer width (0.015 instead of 0.02)
    cb.Label.String = 'Time (s)';
    
    sgtitle('2D Trajectory Traces');  % Overall title

    measurementLog = simOut.meas_log;
    %% FFT
    figure;
    fs = 1 / mean(diff(t));
    windowSize = 256;       % ~0.25 seconds of data
    overlap = floor(windowSize * 0.9); % 90% overlap
    nfft = 2048;            % High NFFT for smooth Y-axis
    spectrogram(measurementLog.Data(4,:), kaiser(windowSize, 5), overlap, nfft, fs, 'yaxis');

    %% Flight Animation
    AnimateFlight(t, r, q, Checkpoints)
end
function AnimateFlight(t, r, q, Checkpoints)
    % Create a new figure for animation with Dark Mode
    fAnim = figure('Name', '6DoF Flight Animation', 'Color', [0.1 0.1 0.1]);
    ax = axes(fAnim);
    set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.3);
    
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
    view(ax, 3);
    
    % 1. Plot the static Trajectory Path
    plot3(ax, r(:,1), r(:,2), r(:,3), 'Color', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1);
    
    % 2. Plot Checkpoints
    if ~isempty(Checkpoints)
        for i = 1:size(Checkpoints, 2)
            plot3(ax, Checkpoints(1,i), Checkpoints(2,i), Checkpoints(3,i), ...
                'ro', 'MarkerSize', 10, 'LineWidth', 1.5);
        end
    end

    % 3. Define Vehicle Geometry (Aligned with Z-Axis / UP)
    % This creates a "Stick" drone with legs at the bottom.
    scale = 0.25; 
    
    % Vertices: [X, Y, Z]
    % Nose is at +Z. Tail/Legs are at -Z.
    bodyVerts = [ 0.0,  0.0,  2.5;   % 1. Nose Tip (Top)
                  0.0,  0.0, -1.0;   % 2. Tail Center (Bottom)
                  0.8,  0.0, -1.5;   % 3. Leg +X
                 -0.8,  0.0, -1.5;   % 4. Leg -X
                  0.0,  0.8, -1.5;   % 5. Leg +Y
                  0.0, -0.8, -1.5];  % 6. Leg -Y
                 
    bodyVerts = bodyVerts * scale;
    
    % Connect Nose to Legs, and Tail to Legs
    faces = [1 3 5; 1 5 4; 1 4 6; 1 6 3;   % Upper body
             2 3 5; 2 5 4; 2 4 6; 2 6 3];  % Lower body

    hVehicle = patch(ax, 'Vertices', bodyVerts, 'Faces', faces, ...
        'FaceColor', 'cyan', 'FaceAlpha', 0.8, 'EdgeColor', 'w', 'LineWidth', 1.5);

    % 4. Define Body Axes (Length 2.0)
    % We plot these to visualize the Local Frame orientation
    axisLen = 2.0 * scale;
    hAxisX = plot3(ax, [0 0], [0 0], [0 0], 'r', 'LineWidth', 2); % Red = X
    hAxisY = plot3(ax, [0 0], [0 0], [0 0], 'g', 'LineWidth', 2); % Green = Y
    hAxisZ = plot3(ax, [0 0], [0 0], [0 0], 'b', 'LineWidth', 2); % Blue = Z

    % Set Axis Limits (Tracking the flight)
    xlim(ax, [min(r(:,1))-1, max(r(:,1))+1]);
    ylim(ax, [min(r(:,2))-1, max(r(:,2))+1]);
    zlim(ax, [min(r(:,3)), max(r(:,3))+1]);

    % 5. Real-Time Animation Loop
    title(ax, 'Preparing Animation...', 'Color', 'w');
    
    % Base vectors for axes
    vX = [axisLen 0 0];
    vY = [0 axisLen 0];
    vZ = [0 0 axisLen];

    pause(10);
    startTime = tic;
    finalTime = t(end);
    while true
        % Get current wall-clock time relative to start
        tApp = toc(startTime);
        
        if tApp > finalTime
            break; 
        end
        
        % Find the closest simulation index for the current time
        % This effectively skips frames if drawing is slow, maintaining sync.
        % 'find' is fast enough for this array size.
        idx = find(t >= tApp, 1);
        if isempty(idx), idx = length(t); end
        
        % --- Update Visuals ---
        pos = r(idx, :);
        q_curr = q(idx, :); 
        
        % Rotate Vertices
        rotVerts = RotateVector(bodyVerts, q_curr); 
        
        % Translate
        worldVerts = rotVerts + pos;
        set(hVehicle, 'Vertices', worldVerts);
        
        % Update Axes
        x_rot = RotateVector(vX, q_curr);
        y_rot = RotateVector(vY, q_curr);
        z_rot = RotateVector(vZ, q_curr);
        
        set(hAxisX, 'XData', [pos(1), pos(1)+x_rot(1)], 'YData', [pos(2), pos(2)+x_rot(2)], 'ZData', [pos(3), pos(3)+x_rot(3)]);
        set(hAxisY, 'XData', [pos(1), pos(1)+y_rot(1)], 'YData', [pos(2), pos(2)+y_rot(2)], 'ZData', [pos(3), pos(3)+y_rot(3)]);
        set(hAxisZ, 'XData', [pos(1), pos(1)+z_rot(1)], 'YData', [pos(2), pos(2)+z_rot(2)], 'ZData', [pos(3), pos(3)+z_rot(3)]);
        
        title(ax, sprintf('Time: %.2f s', t(idx)), 'Color', 'w');
        
        drawnow limitrate; 
    end
end

function v_rot = RotateVector(v, q)
    % Rotates vector v by quaternion q [w, x, y, z]
    % v can be an Nx3 matrix
    
    w = q(1);
    u = q(2:4);
    
    % Standard Quaternion Rotation Formula: v' = v + 2w(u x v) + 2(u x (u x v))
    % Vectorized for N rows
    
    numV = size(v, 1);
    u_rep = repmat(u, numV, 1);
    
    uv = cross(u_rep, v, 2);
    uuv = cross(u_rep, uv, 2);
    
    v_rot = v + 2 * w * uv + 2 * uuv;
end