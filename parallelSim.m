mdl = "SimulationLoop";

covariences = 0:0.5:5
coVarNum = length(covariences);
gainNum = 100;

a = 1.2;
b = 2;
gains  = wblrnd(a, b, 1, gainNum);

simIn(1:gainNum * coVarNum) = Simulink.SimulationInput(mdl);
for gainIdx = 1:gainNum
    for coVarIdx = 1:coVarNum
        simIn(((gainIdx - 1) * coVarNum) + coVarIdx) = setBlockParameter(simIn(((gainIdx - 1) * coVarNum) + coVarIdx),...
                             ("SimulationLoop/Plant Simulation/Disturbance Models and Integrator/Wind/Wind Gain"),...
                              "Gain",num2str(gains(gainIdx)));
        simIn(((gainIdx - 1) * coVarNum) + coVarIdx) = setBlockParameter(simIn(((gainIdx - 1) * coVarNum) + coVarIdx),...
                             ("SimulationLoop/Plant Simulation/Disturbance Models and Integrator/Wind/Covar"),...
                              "Value",num2str(covariences(coVarIdx)));
    end
end

simOut = parsim(simIn, 'ShowSimulationManager', 'on', TransferBaseWorkspaceVariables='on');