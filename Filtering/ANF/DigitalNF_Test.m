% This section executes the filter test.
clear; 
clear DigitalNF;

% --- Filter Parameters (from ANF_FilterDesign.m) ---
fs = 1000; % Sampling frequency (Hz)
dT = 1 / fs; % Sampling time (s)

% --- Signal Parameters ---
T = 10;    % Duration of the signal (seconds) - Good for FFT resolution
N = T * fs; % Total number of samples

% --- Generate White Noise Input ---
% Use a single column vector (1-channel IMU data)
IN_Noise = randn(N, 1) * 0.1; 

% --- Dynamic Thrust Profile (To excite adaptive behavior) ---
THRUST = 68;
THRUST_TIME = THRUST * ones(1, N);

% --- Calculate expected notches ---
TRACK = [1.3525    42.6278;
         2.6867    84.8000];
Notches = [TRACK(1, 1) * THRUST + TRACK(1, 2); 
           TRACK(2, 1) * THRUST + TRACK(2, 2)];

% --- Setup for Filter Run ---
GND = false; % Filter is active
OUT_Filtered = zeros(N, 1);

% --- Simulation Loop ---
disp('Running sequential notch filter simulation...');
for k = 1:N
    % Call the filter function for the current sample 'k'
    OUT_Filtered(k) = DigitalNF(IN_Noise(k), GND, THRUST_TIME(k), dT);
end
disp('Simulation complete.');


%% 3. Frequency Analysis (FFT / PSD)
% The resulting spectrum is the AVERAGE magnitude response over the entire thrust range.

figure('Name', 'Sequential Adaptive Notch Filter Test');

% --- Input Spectrum (Should be flat) ---
subplot(2,1,1);
pwelch(IN_Noise, [], [], N, fs);
title('Input Signal Spectrum (White Noise)');
xlabel('Frequency (Hz)');
ylabel('Power/Frequency (dB/Hz)');
xlim([0 fs/2]);
grid on;

% --- Output Spectrum (Should show averaged Notch Dips) ---
subplot(2,1,2);
pwelch(OUT_Filtered, [], [], N, fs);
title('Filtered Signal Spectrum (Average Magnitude Response)');
xlabel('Frequency (Hz)');
ylabel('Power/Frequency (dB/Hz)');
xlim([0 fs/2]);
grid on;
xline(25, 'g--', 'LineWidth', 2)
xline(Notches(1), 'r--', 'LineWidth', 2);
legend('Signal', 'Cutoff LPF', 'Adaptive Notch')