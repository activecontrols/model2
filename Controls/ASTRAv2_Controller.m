% Version 2 Controller formulation for ASTRAv2. Structure consists of 3
% cascaded loops.
%
% First Loop is a P Loop in charge of Pos -> Vel commands.
%
% Second loop is a PI Loop in charge of Vel -> Acceleration commands, fitted
% with anti-windup and soft-gatingcbased on attitude error.
%
% Second loop is an LQRi Loop in charge of Target Attitude -> Gimbal +
% Torque commands. Integral action is fitted with anti-windup clamps and is
% self soft-gated.
% Tuning of the second loop is done by using a Genetic algorithm to
% maximize a Crossover Frequency vs. Disk Margin tradeoff on all actuator
% channels.
%
% By: Pablo Plata   -   11/27/25 (Happy Thanksgiving!)
function U = ASTRAv2_Controller(PosTarget, X, constantsASTRA, t)

% Time Counter
persistent lastT VelErrorI AttErrorI
if isempty(error_pos)
    lastT = 0;
    VelErrorI = zeros(3,1);
    AttErrorI = zeros(3,1);
end
dT = t - lastT;

%% First Loop (P Loop)
    % Position Error Vector
    PosError = PosTarget - X(5:7);
    
    % Velocity Command
    K_P = [0.7; 0.7; 0.7];
    VelTarget = VelGain_P .* PosError;

    % ADD Velocity Saturation Step

%% Second Loop (PI Loop)
    % Velocity Error Vector
    VelError = VelTarget - X(8:10);

    % Integral Accumulator
    K_I = [0.05; 0.05; 0.05];
    MaxAttError = [0.2; 0.2; 0.2];
    Leak = 0.1;
    Clamp = [3; 3; 5];

    % Soft Gating for Integral Accumulator and Clamping
    Gate = max(min(1 - abs(AttError) ./ MaxAttError, 1), Leak);
    K_I = K_I * Gate;
    VelErrorI = VelErrorI + K_I .* VelError .* dT;
    VelErrorI = max(min(VelErrorI, Clamp), -Clamp);
    K_P = [0.1; 0.1; 0.1];

    % Acceleration Target
    AccelTarget = K_P .* VelError + K_I * VelErrorI;

    % ADD Acceleration Saturation Step. Constraint to 20° Cone for laterals

%% Kinematics Step
    % Compute thrust target
    TargetForce_I = constantsASTRA.m * AccelTarget + [0; 0; constantsASTRA.g];
    TargetForce_B = quatRot(X(1:4)) * TargetForce_I;
    U(3) = TargetForce_B(3);
    
    % Compute target attitude via GSP.
    AccelTarget(3) = max(AccelTarget(3), constantsASTRA.g);
    Z = AccelTarget / norm(AccelTarget);

    % Heading reference (+X axis rolled to north)
    HDGRef = [1; 0; 0];
    Y = cross(Z, HDGRef);
    Y = Y / norm(Y);

    % Complete the triad
    X = cross(Y, Z);

    % Create DCM and convert to quaternion
    DCM = [X Y Z];
    TargetAtt = DCM_Quat_Conversion;


%% Third Loop (LQRi)
    

