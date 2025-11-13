%% Digital Sequential Notch Filter implementation for ASTRAv2.
%  All IMU sensors are meant to be passed to this array always. Includes a
%  GND switch option so the filter does not run on the ground when no
%  vibrations are present. 

function OUT = DigitalNF(IN, GND, THRUST, dT)    

% Initialize past output / input memory column 
persistent X1 X2 X3 Y1 Y2 Y3
if isempty(X1)
    X1 = zeros(size(IN, 1), 2) .* IN;
    X2 = X1;
    X3 = X1;
    Y1 = X1;
    Y2 = X1;
    Y3 = X1;
end

% Center Freq. vs Thrust Tracks, Sampling Freq, and Notch Width
TRACK = [1.7672    47.4512;
         3.1740    109.242];
fs = 1 / dT;
width1 = 20;     % Hz
width2 = width1 + 0.35 * THRUST;

% Sequential Notch Filter
if ~GND
    %% Notch #1
        % Setup first notch at constant frequency
        f0 = 94;    %Hz
        w0 = 2 * pi * f0 / fs;
        r = exp(-pi * width1 / fs);
        OUT = IN - 2*cos(w0)*X1(:, 1) + X1(:, 2) + 2*r*cos(w0)*Y1(:,1) - r^2*Y1(:,2); 
    
        % Update memory for first Notch
        X1 = [IN    X1(:, 1)];
        Y1 = [OUT   Y1(:, 1)];
        IN = OUT;
    
    %% Notch #2
        % Setup second notch following track #1
        f0 = TRACK(1, 1) * THRUST + TRACK(1, 2);    %Hz
        w0 = 2 * pi * f0 / fs;
        r = exp(-pi * width2 / fs);
        OUT = IN - 2*cos(w0)*X2(:, 1) + X2(:, 2) + 2*r*cos(w0)*Y2(:,1) - r^2*Y2(:,2); 
    
        % Update memory for second Notch
        X2 = [IN    X2(:, 1)];
        Y2 = [OUT   Y2(:, 1)];
        IN = OUT;

    %% Notch #3
        % Setup third notch following track #2
        f0 = TRACK(2, 1) * THRUST + TRACK(2, 2);    %Hz
        w0 = 2 * pi * f0 / fs;
        r = exp(-pi * width2 / fs);
        OUT = IN - 2*cos(w0)*X3(:, 1) + X3(:, 2) + 2*r*cos(w0)*Y3(:,1) - r^2*Y3(:,2); 
    
        % Update memory for second Notch
        X3 = [IN    X3(:, 1)];
        Y3 = [OUT   Y3(:, 1)];
end