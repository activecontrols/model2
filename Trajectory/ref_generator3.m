function ref = ref_generator3(x, t, TargetPos, HoldTimeReqs)
    
    % Sets the time to execute an abort at
    % A value of 0 result in no abort being run
    ABORT = 0;

    x = x(1:12,1);
    MaxAscentSpeed = 4;         %m/s
    MaxDescentSpeed = -4;       %m/s
    MaxLatSpeed = 1;            %m/s
    % HoldTimeReqs = [4, 3, 3, 3, 3, 3, 0.2];    % Time needed to hold at each checkpoint

    persistent timeFlag
    persistent i
    persistent timeCounter
    persistent prevTime
    persistent HoldMode

    if isempty(timeFlag)
        timeFlag = 999;
        i = 1;
        timeCounter = 0;
        prevTime = 0;
        HoldMode = 0;
    end
    dt = t - prevTime;
    prevTime = t;

    % Ignores lateral position gain if time is past set abort value
    % ABORT MODE LOGIC: If artificial abort is triggered, enter abort mode
    % loop. If the position hold mode is disabled, set lateral velocity
    % references to zero by setting their gains to zero. If lateral
    % velocities are below threshold, pick current position as hold and
    % activate HoldMode.

    PosGain = [0.55; 0.55; 0.75];
    isABORT = ABORT > 0 && t >= ABORT;
    if isABORT
        if HoldMode == 0
            PosGain   = [0; 0; 0.7];
            TargetPos(:,i) = zeros(3,1);
        end
        if norm(x(7:8)) < 0.2 && HoldMode == 0
            TargetPos(:,i) = [x(4:5); 0];
            HoldMode = 1;
        end
    end
    
    PosError = TargetPos(:, i) - x(4:6);
    TargetVel = PosGain .* PosError;
    TargetVel(1:2) = max(min(TargetVel(1:2), MaxLatSpeed), -MaxLatSpeed);
    TargetVel(3) = max(min(TargetVel(3), MaxAscentSpeed), MaxDescentSpeed);

    TargetVec = [zeros(3,1); TargetPos(:, i); TargetVel; zeros(3,1)];

    ref = x - TargetVec;

    if norm(ref(4:6,1)) < sqrt(3) && isABORT == 0 
        timeCounter = timeCounter + dt;
    end
    if timeCounter > HoldTimeReqs(i) && i < size(HoldTimeReqs,2)
        i = i + 1;
        timeCounter = 0;
    end