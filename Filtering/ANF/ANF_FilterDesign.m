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

    % Define z-domain variable and discrete TF
    z = tf('z', Ts);
    TF = (1 - 2 * cos(w0) * z^-1 + z^-2) / (1 - 2 * r *cos(w0) * z^-1 + r^2 * z^-2);
end
function TF = convTF(TF_Array)
    n = size(TF_Array, 1);
    TF = 1;
    for i = 1:1:n
        TF = TF_Array(i,:) .* TF;
    end
end

% Clear workspace
clear;
close;

%% Filter structure design
    % 6th order filter composed of 3 sequential ANF's (Adaptive Notch Filters).
    % The 1st Notch filter is set to a constant frequency os ~120Hz. Other two
    % ANF's follow a linear trace based on thrust (see RollTestAnalysis.m for
    % that).
% Create Notch Filters
f0 = 94;
width1 = 20;
fs = 1000;
n = 3;
res = 200;
thrustArray = linspace(1, 100, res);
NotchC_Array = zeros(n, res) * tf('s');

% Center Freq. vs Thrust Tracks
tracks = [1.7672    47.4512;
          3.1740    109.242];

% Build constant Notch at 120 Hz
NotchC_Array(1,:) = Notch_TFD(f0, width1, fs);
for thrust = 1:1:res
    for track = 2:1:n
        width2 = width1 + 0.35 * thrust;
        NotchFreq = tracks(track-1, 1) * thrustArray(thrust) + tracks(track-1, 2);
        NotchC_Array(track,thrust) = Notch_TFD(NotchFreq, width2, fs);
    end
end

NotchC = convTF(NotchC_Array);

% Create a 2nd order LPF for comparaison
s = tf('s');
wc = 0.8 * f0 * 2 * pi;
phi = (1 + sqrt(5))/ 2;
LPF = wc^2 / (s^2 + sqrt(2) * wc * s + wc^2);

% Frequency range in Hz
f = linspace(0, 500, 2000);    % Linear spacing 0–500 Hz
w = 2 * pi * f;                % Convert to rad/s for 'bode' or 'freqresp'

% Get frequency response at each thrust level
magC = zeros(res, size(f, 2));
phaseC = magC;
for thrust = 1:1:res
    [mag, phase] = bode(NotchC(1, thrust), w);
    mag = squeeze(mag);
    phase = squeeze(phase);
    phase = wrapTo180(phase);
    magC(thrust, :) = mag; 
    phaseC(thrust, :) = phase;
end

% Get frequency response of the LPF
[magLPF, phaseLPF] = bode(LPF, w);
magLPF = squeeze(magLPF);
phaseLPF = squeeze(phaseLPF);

% Wrap the unwrapped phase from bode() to the [-180, 180] range
phaseLPF = wrapTo180(phaseLPF);

% Plots
% --- 1. Fix the Phase Wrap Artifact ---

% 'unwrap' works in radians, so we convert, unwrap, and convert back
% We unwrap along dimension 2 (the rows), which is your frequency axis
magC_dB = 20 * log10(magC);
phaseC_rad = deg2rad(phaseC); 
phaseC_unwrapped_rad = unwrap(phaseC_rad, [], 2);
phaseC_unwrapped_deg = rad2deg(phaseC_unwrapped_rad);

% --- 3. Create the 3D Surface Plots (Convention: X=Thrust, Y=Freq) ---
figure;

% --- Top plot for Magnitude ---
subplot(2,1,1);
colormap turbo
% Swap axes (f, thrustArray -> thrustArray, f) and transpose data (magC_dB -> magC_dB')
surf(thrustArray, f, magC_dB', 'EdgeColor', 'none');
shading interp; 
xlabel('Thrust [%]');           % <-- Swapped
ylabel('Frequency [Hz]');      % <-- Swapped
zlabel('Magnitude [dB]');
title('ANF Magnitude (3D Surface)');
colorbar;
clim([-5, 0]);      % Anything under 5dB is considered cutoff.
view(-90,90);

% --- Bottom plot for Phase (using unwrapped data) ---
subplot(2,1,2);
colormap turbo
% Swap axes and transpose data
surf(thrustArray, f, phaseC_unwrapped_deg', 'EdgeColor', 'none'); 
shading interp;
xlabel('Thrust [%]');           % <-- Swapped
ylabel('Frequency [Hz]');      % <-- Swapped
zlabel('Phase [degrees]');
title('ANF Phase (3D Surface - Unwrapped)');
colorbar;
clim([-90 90]);
view(-90,90);
yline(10, 'r--');

% Link the camera angles so they rotate together
linkaxes([subplot(2,1,1), subplot(2,1,2)], 'xy');
sgtitle('Sequential Adaptive Notch Filter Design for ASTRAv2');