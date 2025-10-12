function ref = ref_generator3(x, t)
    
    % Sets the time to execute an abort at
    % A value of 0 result in no abort being run
    ABORT = 0;

    x = x(1:12,1);
    MaxAscentSpeed = 4;         %m/s
    MaxDescentSpeed = -4;       %m/s
    MaxLatSpeed = 2;            %m/s
    HoldTimeReqs = [5, 5, 0.2, 0.2];    % Time needed to hold at each checkpoint

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

    TargetPos = [0, 5,   0, 0;
                 0, 5,  0, 0;
                 0, 50,  0, 0];

    % Ignores lateral position gain if time is past set abort value
    if ABORT > 0 & t >= ABORT 
        PosGain   = [0; 0; 0.8];
        TargetPos = zeros(size(TargetPos));    
    else
        PosGain = [0.35; 0.35; 0.7];
    end
    
    PosError = TargetPos(:, i) - x(4:6);
    TargetVel = PosGain .* PosError;
    TargetVel(1:2) = max(min(TargetVel(1:2), MaxLatSpeed), -MaxLatSpeed);
    TargetVel(3) = max(min(TargetVel(3), MaxAscentSpeed), MaxDescentSpeed);

    TargetVec = [zeros(3,1); TargetPos(:, i); TargetVel; zeros(3,1)];

    ref = x - TargetVec;

    if abs(ref(4:6,1)) < 3
        timeCounter = timeCounter + dt;
    end

    if timeCounter > HoldTimeReqs(i) && i < size(HoldTimeReqs,2)
        i = i + 1;
        timeCounter = 0;
    end