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
persistent lastT VelErrorI AttErrorI lastAttError
if isempty(lastT)
    lastT = 0;
    VelErrorI = zeros(3,1);
    AttErrorI = zeros(4,1);
    lastAttError = zeros(4,1);
end
dT = t - lastT;
lastT = t;
K_Att = constantsASTRA.K_Att;
%% First Loop (P Loop)
    % Position Error Vector
    PosError = PosTarget - X(5:7);
    
    % Velocity Command
    K_P = [0.7; 0.7; 0.7];
    VelTarget = K_P .* PosError;

    % Velocity Saturation Step
    MaxVel = [1 1 2]';
    VelTarget = max(min(VelTarget, MaxVel), -MaxVel);

%% Second Loop (PI Loop)
    % Velocity Error Vector
    VelError = VelTarget - X(8:10);

    % Integral Accumulator
    K_I = [0.05; 0.05; 0.05];
    MaxAttError = [0.2; 0.2; 0.2];
    Leak = 0.1;
    Clamp = [3; 3; 5];

    % Soft Gating for Integral Accumulator and Clamping
    Gate = max(min(1 - abs(lastAttError(2:4)) ./ MaxAttError, 1), Leak);
    K_I = K_I .* Gate;
    VelErrorI = VelErrorI + K_I .* VelError .* dT;
    VelErrorI = max(min(VelErrorI, Clamp), -Clamp);
    K_P = [0.1; 0.1; 0.1];

    % Acceleration Target
    AccelTarget = K_P .* VelError + VelErrorI  + [0; 0; constantsASTRA.g];

    % Acceleration Saturation Step
    MaxAccelUp = [2 2 15]';
    MaxAccelDown = [-2 -2 5]';
    AccelTarget = max(min(AccelTarget, MaxAccelUp), MaxAccelDown);

%% Kinematics Step
    % Compute thrust target
    TargetForce_I = constantsASTRA.m * AccelTarget;
    TargetForce_B = quatRot(X(1:4)) * TargetForce_I;
    U(3) = TargetForce_B(3);
    
    % Compute target attitude via GSP.
    AccelTarget(3) = max(AccelTarget(3), constantsASTRA.g);
    Z_b = AccelTarget / norm(AccelTarget);

    % Heading reference (+X axis rolled to north)
    HDGRef = [0; -1; 0];
    Y_b = cross(Z_b, HDGRef);
    Y_b = Y_b / norm(Y_b);

    % Complete the triad
    X_b = cross(Y_b, Z_b);

    % Create DCM and convert to quaternion
    DCM = [X_b Y_b Z_b];
    TargetAtt = DCM_Quat_Conversion(DCM);

%% Third Loop (LQRi)
    % Attitude Error computation
    AttError = HamiltonianProd([X(1); -X(2:4)]) * TargetAtt;
    lastAttError = AttError;

    % Error accumulation and clamping
    Clamp = [0.1; 0.1; 0.1];
    AttErrorI = AttErrorI + AttError(2:4) .* dT;
    AttErrorI = max(min(AttErrorI, Clamp), -Clamp);

    % State vector and error
    X_Err = [AttError(2:4); X(11:13); AttErrorI];

    % LQR Controller
    U([1 2 4]) = -K_Att * X_Err;


    

