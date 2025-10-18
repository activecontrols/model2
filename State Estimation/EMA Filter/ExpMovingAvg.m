function Output = ExpMovingAvg(Input)
    % Basic Exponential Moving Average Implementation to pre-process
    % measurements.

    % Initialize output array
    Output = zeros(size(Input)); 

    % Initialize peristent variable
    persistent lastOut
    if isempty(lastOut)
        lastOut = Output;
    end

    % Filter parameter
    Alpha = 0.32;

    % Filter difference equation
    Output = Alpha * Input + (1 - Alpha) * lastOut;
    lastOut = Output;
end
