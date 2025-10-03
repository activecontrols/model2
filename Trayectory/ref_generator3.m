function ref = ref_generator3(x, t)
    
    x = x(1:12,1);
    MaxAscentSpeed = 5;         %m/s
    MaxDescentSpeed = -5;       %m/s
    MaxLatSpeed = 3;            %m/s
    HoldTimeReqs = [1, 7, 0.2, 0.2];    % Time needed to hold at each checkpoint

    persistent timeFlag
    persistent i
    persistent timeCounter
    persistent prevTime

    if isempty(timeFlag)
        timeFlag = 999;
        i = 1;
        timeCounter = 0;
        prevTime = 0;
    end
    dt = t - prevTime;
    prevTime = t;

    % TargetX1 = [0, 5, 0, 0];
    % TargetX2 = [0, 50, 0, 0];

    TargetPos = [0, 5,   0, 0;
                 0, 10,  0, 0;
                 0, 50,  0, 0];
    TargetAttitude = zeros(size(HoldTimeReqs, 1), 3);

    % X1Gain = 0.075;
    % X1Error = TargetX1(i) - x(1);
    % TargetX3 = X1Gain * X1Error;
    % TargetX3 = max(TargetX3, -MaxLatSpeed);
    % TargetX3 = min(TargetX3, MaxLatSpeed);
    % 
    % X2Gain = 0.65;
    % X2Error = TargetX2(i) - x(2);
    % TargetX4 = X2Gain * X2Error;
    % TargetX4 = max(TargetX4, MaxDescentSpeed);
    % TargetX4 = min(TargetX4, MaxAscentSpeed);

    PosGain = [0.2; 0.2; 1.25];
    PosError = TargetPos(:, i) - x(4:6);
    TargetVel = PosGain .* PosError;
    TargetVel(1) = max(min(TargetVel(1), MaxAscentSpeed), MaxDescentSpeed);
    TargetVel(2:3) = max(min(TargetVel(2:3), MaxLatSpeed), -MaxLatSpeed);

    %TargetVec = [TargetX1(i); TargetX2(i); TargetX3; TargetX4; TargetX5(i); TargetX6; 0];
    TargetVec = [zeros(3,1); TargetPos(:, i); TargetVel; zeros(3,1)];

    ref = x - TargetVec;

    if abs(ref(4:6,1)) < 3
        timeCounter = timeCounter + dt;
    end

    if timeCounter > HoldTimeReqs(i) && i < size(HoldTimeReqs,2)
        i = i + 1;
        timeCounter = 0;
    end