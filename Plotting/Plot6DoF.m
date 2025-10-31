function Plot6DoF(Checkpoints)
    % Load the Simulink model
    load_system("SimulationLoop.slx");
    
    % Run the simulation
    simOut = sim("SimulationLoop.slx");
    
    % Extract state timeseries
    stateTimeSeries = simOut.state_log;
    t = stateTimeSeries.Time;  % Time vector
    x = stateTimeSeries.Data;  % State matrix (15 x time)
    
    % Extract states (q_vec is 3x1 vector part)
    % q_vec = x(1:3, :)';  % [qx qy qz]
    r = x(4:6, :)';      % Position [x y z]
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
    fig = figure(2);
    fig.Position = [50 100 700 400];  % Optional: Smaller figure size (adjust as needed)
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
end
