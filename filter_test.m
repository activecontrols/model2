%LoadSimulation;
data = load("C:\Users\rober\Purdue\Clubs\PSP-AC-GNC\model2\sim_run.mat");
z_arr = data.data{5}.Values.Data;
GND_arr = data.data{3}.Values.Data;
exp_x_est_arr = data.data{4}.Values.Data;
x_est_arr = data.data{1}.Values.Data;

clear EstimateStateFCN


for i = 1:1:69
  fprintf("Run %d\n", i);
  out = EstimateStateFCN(x_est_arr(i,:)', constantsASTRA, z_arr(:,i), 0.001, GND_arr(i));
  t = out(1:13)'
end