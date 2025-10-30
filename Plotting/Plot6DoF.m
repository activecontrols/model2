function Plot6DoF
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
    figure;
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
    plot3(2,5,5,"ro", 'MarkerSize',20, 'MarkerMode','manual')
    plot3(5,5,5,"ro", 'MarkerSize',20)
    plot3(5,5,8,"ro", 'MarkerSize',20)
    plot3(0,0,0,"ro", 'MarkerSize',20)

    hold off;
end
