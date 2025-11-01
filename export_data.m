data = load("C:\Users\rober\Purdue\Clubs\PSP-AC-GNC\model2\sim_run.mat");
MAX = 10000;

z_arr = data.data{5}.Values.Data;
GND_arr = data.data{3}.Values.Data;
x_est_arr = data.data{1}.Values.Data;
clear EstimateStateFCN

fprintf("#define MAX_IDX %d\n\n", MAX/10);

fprintf("double z_arr[MAX_IDX][15] = {\n")
for i = 1:10:MAX
    fprintf("    {")
    for col = 1:1:14 
     fprintf("%.8f, ", z_arr(col,i));
    end
    fprintf("%.8f", z_arr(15,i));
    fprintf("},\n")
end
fprintf("};\n");

fprintf("double GND_arr[MAX_IDX] = {\n")
for i = 1:10:MAX
    fprintf("%.8f,\n", GND_arr(i));
end
fprintf("};\n");

fprintf("double x_est_arr[MAX_IDX][13] = {\n")
for i = 1:10:MAX
    fprintf("    {")
    for col = 1:1:12 
     fprintf("%.8f, ", x_est_arr(i,col));
    end
    fprintf("%.8f", x_est_arr(i,13));
    fprintf("},\n")
end
fprintf("};\n");

fprintf("double exp_x_est_arr[MAX_IDX][13] = {\n")
for i = 1:10:MAX
    out = EstimateStateFCN(x_est_arr(i,:)', constantsASTRA, z_arr(:,i), 0.001, GND_arr(i));

    fprintf("    {")
    for col = 1:1:12 
     fprintf("%.8f, ", out(col));
    end
    fprintf("%.8f", out(13));
    fprintf("},\n")
end
fprintf("};\n");