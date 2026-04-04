mdl = 'SimulationLoop.slx';

numWind = 4;
a = 1.6;
b = 2;
windGains  = wblrnd(a, b, 1, numWind);

numCoVar = 2;
coVars = linspace(0, 5, coVarNum);

numMoI = 3;
MoIGains = linspace(-4, 4, numMoI);
constsArrJ(1:numMoI) = constantsASTRA;
for i = 1:length(MoIGains)
    constsArrJ(i).J = constsArrJ(i).J(:,:) * MoIGains(i);
end

massGains = 0.95:0.05:1.05;
numMasses = length(massGains);
constsArrM(1:numMass) = constantsASTRA;
for i = 1:length(massGains)
    constsArrM(i).m = constsArrM(i).m * massGains(i);
end

simIn(numWind, numCoVar, numMoI) = Simulink.SimulationInput(mdl);