% Generates both the analog and discrete transfer functions for the ANF-S
% filter for ASTRAv2.

function [TFC, TFD] = FilterTF_Gen(thrust)
    function TF = Notch_TFC(f0, width)
        % Define center frequency and Q
        w0 = 2 * pi * f0;
        Q = f0 / width;
    
        % Define Laplace variable and transfer function.
        s = tf('s');
        TF = (s^2 + w0^2) / (s^2 + (w0 / Q) * s + w0^2);
    end
    function TF = Notch_TFD(f0, width, fs)
        % Sampling time
        Ts = 1 / fs;
    
        % Define center frequency and r
        w0 = 2 * pi * f0 / fs;
        r = exp(-pi * width / fs);
    
        % Calculate Normalization Gain
        G_num = 1 - 2*r*cos(w0) + r^2;
        G_den = 2 - 2*cos(w0);
        G = G_num / G_den;
    
        % Define z-domain variable and discrete TF
        z = tf('z', Ts);
        TF = G * (1 - 2 * cos(w0) * z^-1 + z^-2) / (1 - 2 * r *cos(w0) * z^-1 + r^2 * z^-2);
    end
    function TF = LPF_D(Cutoff, fs)
        % OUTDATED (!!!)
        Cutoff = Cutoff * 2 * pi;
        Ts = 1 / fs;
        z = tf('z', Ts);
        TF = (Cutoff + Cutoff * z^-1) / (2/Ts + Cutoff + (Cutoff - 2/Ts) * z^-1);
    end
    function TF = LPF_C(Cutoff)
        Cutoff = Cutoff * 2 * pi;
        s = tf('s');
        TF = Cutoff^2 / (s^2 + sqrt(2)*Cutoff*s + Cutoff^2);
    end
    function TF = convTF(TF_Array)
        n = size(TF_Array, 1);
        TF = 1;
        for i = 1:1:n
            TF = TF_Array(i,:) .* TF;
        end
    end

    % Create Notch Filters
    f0 = 94;
    width1 = 25;
    fs = 1000;
    numNotch = 2;
    res = 200;
    NotchC_Array = zeros(numNotch, 1) * tf('s');
    NotchD_Array = NotchC_Array;
    
    % Center Freq. vs Thrust Tracks
    tracks = [1.3525    42.6278;
              2.6867    84.8000];

    % Build Adaptive notch
    width2 = width1; %+ thrust / 100 * 20;
    NotchFreq = tracks(1, 1) * thrust + tracks(1, 2);
    NotchC_Array(1,1) = Notch_TFC(NotchFreq, width2);
    NotchD_Array(1,1) = Notch_TFD(NotchFreq, width2, fs);

    % Build LPF
    Cutoff = 30;
    NotchC_Array(2,1) = LPF_C(Cutoff);
    NotchD_Array(2,1) = LPF_D(Cutoff, fs);

    % Convolve the transfer functions
    TFC = convTF(NotchC_Array);
    TFD = convTF(NotchD_Array);
end