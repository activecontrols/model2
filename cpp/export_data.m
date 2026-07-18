%% Load Data

data = load("C:\Users\rober\Purdue\Clubs\PSP-AC-GNC\model2\cpp\full_v2_controller_run.mat");
MAX = 25000;
STEP = 25;

exp_controller_output = data.data{1}.Values.Data;
state_arr = data.data{2}.Values.Data;
z_arr = data.data{3}.Values.Data;
GND_arr = data.data{4}.Values.Data;
disc_controller_output = zeros(4, MAX);
disc_state = zeros(16, MAX);
disc_target = zeros(3, MAX);
disc_dnf_out = zeros(15, MAX);
disc_last_cmd_thrust = zeros(1, MAX);
disc_xkf = zeros(13, MAX);
disc_P = zeros(144, MAX);
dT = 0.002;

clear EstimateStateFCN
clear ASTRAv2_Controller
clear EMA_Gyros_MLFUNC
clear DigitalNF

%% Discretize Output

x_est = zeros(13,1);
x_est(1) = 1;
last_cmd_thrust = constantsASTRA.m * constantsASTRA.g;

for i = 1:STEP:MAX
    ANF_IMU = DigitalNF(z_arr(1:9,i), GND_arr(i), last_cmd_thrust, dT);
    Y_FILT = [ANF_IMU; z_arr(10:15,i)];
    % Y_FILT = z_arr(:,i); % bypass filter
    
    x_est = EstimateStateFCN(x_est, constantsASTRA, Y_FILT, dT * STEP, GND_arr(i));
    EMA_G = EMA_Gyros_MLFUNC(Y_FILT);
    X = [x_est(1:4); x_est(5:7); x_est(8:10); EMA_G - x_est(11:13); x_est(11:13)];

    
    [error, trg] = ref_generator3(X, dT * i, Checkpoints, 0);
    [raw_co, VEI] = ASTRAv2_Controller(trg, X, constantsASTRA, dT * i / STEP);
    
    if (GND_arr(i) == 1)
        raw_co = zeros(4, 1);
    end

    last_cmd_thrust = raw_co(3);

    for j = i:1:i+STEP
        tp_index = min(floor(dT * j / 5) + 1, 8); % step through 1-8, advancing every 5 secs
        disc_dnf_out(:,j) = Y_FILT;
        disc_target(:,j) = Checkpoints(:,tp_index);
        disc_state(:,j) = X;
        disc_controller_output(:,j) = raw_co;
        disc_last_cmd_thrust(:,j) = last_cmd_thrust;
        disc_xkf(:,j) = x_est;
        disc_P(:,j) = reshape(P,[144,1]);
    end
end

%% Plots
DO_PLOTS = 0;

if (DO_PLOTS)
    figure;
    for j = 1:15
        subplot(4,4,j);
        plot(disc_state(j, 1:MAX), 'b-', 'LineWidth', 1.5); hold on;
        plot(state_arr(j, 1:MAX), 'r--', 'LineWidth', 1.2);
        ylabel(['Output ', num2str(j)]);
        grid on;
        if j == 1
            title('Discretized vs Simulink Controller State');
        end
        if j == 4
            xlabel('Time (s)');
        end
        legend('Discretized', 'Simulink');
    end
    
    figure;
    for j = 1:4
        subplot(4,1,j);
        plot(disc_controller_output(j, 1:MAX), 'b-', 'LineWidth', 1.5); hold on;
        plot(exp_controller_output(j, 1:MAX), 'r--', 'LineWidth', 1.2);
        ylabel(['Output ', num2str(j)]);
        grid on;
        if j == 1
            title('Discretized vs Simulink Controller Output');
        end
        if j == 4
            xlabel('Time (s)');
        end
        legend('Discretized', 'Simulink');
    end
end


%% Export Data

DO_EXPORT = 1;
if (DO_EXPORT)
    fileID = fopen('sample_data.h','w');

    fprintf(fileID, "#pragma once\n");
    fprintf(fileID, "#define MAX_IDX %d\n", MAX/STEP);
    fprintf(fileID, "#define dT %.4f\n\n", dT * STEP);
    
    fprintf(fileID, "float z_arr[MAX_IDX][15] = {\n");
    for i = 1:STEP:MAX
        fprintf(fileID, "    {");
        for col = 1:1:14 
         fprintf(fileID, "%.8f, ", z_arr(col,i));
        end
        fprintf(fileID, "%.8f", z_arr(15,i));
        fprintf(fileID, "},\n");
    end
    fprintf(fileID, "};\n");
    
    fprintf(fileID, "float GND_arr[MAX_IDX] = {\n");
    for i = 1:STEP:MAX
        fprintf(fileID, "%.8f,\n", GND_arr(i));
    end
    fprintf(fileID, "};\n");
    
    fprintf(fileID, "float target_pos_arr[MAX_IDX][3] = {\n");
    for i = 1:STEP:MAX
        fprintf(fileID, "    {");
        for col = 1:1:2 
         fprintf(fileID, "%.8f, ", disc_target(col,i));
        end
        fprintf(fileID, "%.8f", disc_target(3,i));
        fprintf(fileID, "},\n");
    end
    fprintf(fileID, "};\n");
    
    fprintf(fileID, "float exp_controller_output[MAX_IDX][4] = {\n");
    for i = 1:STEP:MAX
        fprintf(fileID, "    {");
        for col = 1:1:3 
         fprintf(fileID, "%.8f, ", disc_controller_output(col, i));
        end
        fprintf(fileID, "%.8f", disc_controller_output(4, i));
        fprintf(fileID, "},\n");
    end
    fprintf(fileID, "};\n");
    
    fprintf(fileID, "float dnf_out_arr[MAX_IDX][15] = {\n");
    for i = 1:STEP:MAX
        fprintf(fileID, "    {");
        for col = 1:1:14
         fprintf(fileID, "%.8f, ", disc_dnf_out(col, i));
        end
        fprintf(fileID, "%.8f", disc_dnf_out(15, i));
        fprintf(fileID, "},\n");
    end
    fprintf(fileID, "};\n");
    
    fprintf(fileID, "float last_cmd_thurst_arr[MAX_IDX][1] = {\n");
    for i = 1:STEP:MAX
        fprintf(fileID, "    {");
        fprintf(fileID, "%.8f", disc_last_cmd_thrust(1, i));
        fprintf(fileID, "},\n");
    end
    fprintf(fileID, "};\n");

    fprintf(fileID, "float exp_state[MAX_IDX][16] = {\n");
    for i = 1:STEP:MAX
        fprintf(fileID, "    {");
        for col = 1:1:15 
         fprintf(fileID, "%.8f, ", disc_state(col, i));
        end
        fprintf(fileID, "%.8f", disc_state(16, i));
        fprintf(fileID, "},\n");
    end
    fprintf(fileID, "};\n");

    fprintf(fileID, "float exp_xkf[MAX_IDX][13] = {\n");
    for i = 1:STEP:MAX
        fprintf(fileID, "    {");
        for col = 1:1:12
         fprintf(fileID, "%.8f, ", disc_xkf(col, i));
        end
        fprintf(fileID, "%.8f", disc_xkf(13, i));
        fprintf(fileID, "},\n");
    end
    fprintf(fileID, "};\n");

    % fprintf(fileID, "float exp_P[MAX_IDX][144] = {\n");
    % for i = 1:STEP:MAX
    %     fprintf(fileID, "    {");
    %     for col = 1:1:143 
    %      fprintf(fileID, "%.8f, ", disc_P(col, i));
    %     end
    %     fprintf(fileID, "%.8f", disc_P(144, i));
    %     fprintf(fileID, "},\n");
    % end
    % fprintf(fileID, "};\n");
end