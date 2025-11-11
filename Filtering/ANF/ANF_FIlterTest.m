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
        TF = TF_Array(i) * TF;
    end
end

% Clear workspace
clear;

% Create Notch Filters
f0 = 130;
width = 6;
fs = 1000;
n = 3;
NotchC_Array = zeros(n,1) * tf('s');
NotchD_Array = NotchC_Array;
for i = 1:1:n
    NotchC_Array(i,1) = Notch_TFC(i * f0, width);
    NotchD_Array(i,1) = Notch_TFD(i * f0, width, fs);
end

NotchC = convTF(NotchC_Array);
NotchD = convTF(NotchD_Array);

% Create a 2nd order LPF for comparaison
s = tf('s');
wc = 0.8 * f0 * 2 * pi;
phi = (1 + sqrt(5))/ 2;
LPF = wc^2 / (s^2 + sqrt(2) * wc * s + wc^2);
% LPF = wc^2 / ((s + 1)*(s^2 + phi^-1 * s + 1)*(s^2 + phi*s + 1));

% Frequency range in Hz
f = linspace(0, 500, 2000);    % Linear spacing 0–500 Hz
w = 2 * pi * f;                % Convert to rad/s for 'bode' or 'freqresp'

% Get frequency response
[magC, phaseC] = bode(NotchC, w);
[magD, phaseD] = bode(NotchD, w);
[magLPF, phaseLPF] = bode(LPF, w);
magC = squeeze(magC);
phaseC = squeeze(phaseC);
magLPF = squeeze(magLPF);
phaseLPF = squeeze(phaseLPF);
magD = squeeze(magD);
phaseD = squeeze(phaseD);

% Wrap the unwrapped phase from bode() to the [-180, 180] range
phaseC = wrapTo180(phaseC);
phaseD = wrapTo180(phaseD);
phaseLPF = wrapTo180(phaseLPF);

% Plots
figure;
Color = colororder("glow");

subplot(2,1,1)
plot(f, 20*log10(magC), 'LineWidth', 1.5, 'Color', Color(2,:)); hold on;
% plot(f, 20*log10(magD), 'LineWidth', 1.5, 'Color', Color(1,:));
plot(f, 20*log10(magLPF), 'LineWidth', 1.5, 'Color', Color(4,:));
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
legend('Continious ANF', 'LPF');
title('Linear-Frequency Bode Magnitude');
grid on;
xlim([0 500]);
ylim([min(20*log10(magC)) * 1.1, 2]) 

subplot(2,1,2)
plot(f, phaseC, 'LineWidth', 1.5, 'Color', Color(2,:)); hold on;
% plot(f, phaseD, 'LineWidth', 1.5, 'Color', Color(1,:));
plot(f, phaseLPF, 'LineWidth', 1.5, 'Color', Color(4,:));
xlabel('Frequency (Hz)');
ylabel('Phase (deg)');
legend('Continious ANF', 'LPF');
title('Linear-Frequency Bode Phase (Wrapped)');
grid on;
xlim([0 500]);