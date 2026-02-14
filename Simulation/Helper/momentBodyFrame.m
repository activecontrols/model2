% Computes body frame moment for a given input vector and center of mass
% location
function MB = momentBodyFrame(u, rTB, mode)
    
    TB = thrustBodyFrame(u);

    % Off center moments for Simulation
    TBx = -0.008 * (mode == 1);
    TBy = 0.0120 * (mode == 1);
    
    MB = zetaCross([TBx; TBy; rTB])*TB;
end