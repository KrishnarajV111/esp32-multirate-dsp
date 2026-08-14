%% ========================================================================
%  MULTIRATE DSP RECEIVER — Real-Time Hardware-in-the-Loop
%  ========================================================================
%
%  PURPOSE:
%    Receives binary-framed ADC samples from an ESP32 over USB-Serial (or
%    generates simulated real-time data), performs real-time multirate
%    decimation (FIR anti-aliasing + downsample by M=4), computes assessment
%    metrics (latency, MAC count, Big-O), and displays a live 4-subplot figure.
%
%  USAGE:
%    1. Set USE_SIMULATION = true to test without hardware.
%    2. Set USE_SIMULATION = false when connecting an actual ESP32.
%    3. Set COM_PORT below to your ESP32's COM port.
%    4. Run this script in MATLAB (R2020a+ required for serialport).
%    5. Press Ctrl+C or close the figure to stop gracefully.
%
%  BINARY FRAME FORMAT (from ESP32 / Simulated):
%    [0xAA] [0x55] [Lo Byte] [Hi Byte]   — 4 bytes per sample
%    int16 little-endian, range: −2048 to +2047
%
%  Author : AI-Generated (Antigravity)
%  Date   : 2026-08-13
% =========================================================================

%% ──────────────────────── CONFIGURATION ────────────────────────────────
clear; clc; close all;

% ── Operating Mode ──
USE_SIMULATION = true;     % <<< Set to TRUE to run without ESP32 hardware

% ── Serial Port Settings ──
COM_PORT   = 'COM3';        % <<< CHANGE THIS to your ESP32's COM port
BAUD_RATE  = 500000;       % Must match ESP32 firmware
TIMEOUT_S  = 10;           % Serial read timeout (seconds)

% ── DSP Parameters ──
Fs         = 8000;         % Input sample rate (Hz)
M          = 4;            % Decimation factor
Fs_dec     = Fs / M;       % Output sample rate after decimation (Hz)
N_FIR      = 48;           % FIR filter length (taps) — even for symmetry

% ── Display Parameters ──
DISPLAY_SAMPLES   = 2048;  % Samples shown in time-domain plots
DISPLAY_DEC       = DISPLAY_SAMPLES / M;  % Decimated samples shown
READ_CHUNK_BYTES  = 4096;  % Bytes to read per serial chunk
UPDATE_INTERVAL_S = 0.10;  % Minimum seconds between plot refreshes

% ── Sync Marker ──
SYNC_HIGH = uint8(0xAA);
SYNC_LOW  = uint8(0x55);
FRAME_LEN = 4;             % Bytes per sample frame

%% ──────────────────────── FIR FILTER DESIGN ────────────────────────────
fc_hz = 900;                                % Cutoff frequency (Hz)
Wn    = fc_hz / (Fs / 2);                  % Normalized cutoff (0 to 1)
b     = fir1(N_FIR - 1, Wn);               % 48-tap FIR coefficients

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  MULTIRATE DSP RECEIVER — Assessment Metrics\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  Mode                 : %s\n', ternary(USE_SIMULATION, 'SIMULATION (No Hardware)', 'HARDWARE (ESP32 Serial)'));
fprintf('  Input Sample Rate    : %d Hz\n', Fs);
fprintf('  Decimation Factor    : %d\n', M);
fprintf('  Output Sample Rate   : %d Hz\n', Fs_dec);
fprintf('  FIR Filter Taps      : %d\n', N_FIR);
fprintf('  FIR Cutoff Frequency : %d Hz\n', fc_hz);
if ~USE_SIMULATION
    fprintf('  Serial Baud Rate     : %d\n', BAUD_RATE);
end
fprintf('───────────────────────────────────────────────────────────\n');

% ── Computational Complexity Analysis ──
MACs_standard  = N_FIR * Fs;
MACs_polyphase = N_FIR * Fs_dec;

fprintf('\n  ── Computational Complexity ──\n');
fprintf('  Standard FIR Decimation:\n');
fprintf('    MACs/sec     = N × Fs     = %d × %d = %s MACs/s\n', ...
        N_FIR, Fs, addCommas(MACs_standard));
fprintf('    Big-O        = O(N · Fs)  per second\n');
fprintf('  Polyphase Decimation (theoretical):\n');
fprintf('    MACs/sec     = N × Fs/M   = %d × %d = %s MACs/s\n', ...
        N_FIR, Fs_dec, addCommas(MACs_polyphase));
fprintf('    Big-O        = O(N · Fs/M) per second\n');
fprintf('    Efficiency gain: %.1f× fewer operations\n', ...
        MACs_standard / MACs_polyphase);
fprintf('───────────────────────────────────────────────────────────\n\n');

%% ──────────────────────── SERIAL PORT / SIM SETUP ──────────────────────
if ~USE_SIMULATION
    fprintf('  Opening serial port %s at %d baud...\n', COM_PORT, BAUD_RATE);
    try
        s = serialport(COM_PORT, BAUD_RATE);
        configureTerminator(s, 'LF');
        s.Timeout = TIMEOUT_S;
        fprintf('  ✓ Serial port opened successfully.\n');
    catch ME
        fprintf(2, '  ✗ Failed to open serial port: %s\n', ME.message);
        fprintf(2, '    → Set USE_SIMULATION = true to test code without an ESP32.\n');
        return;
    end

    cleanupObj = onCleanup(@() cleanupSerial(s));
    pause(1.0);
    flush(s);
    fprintf('  ✓ Serial buffer flushed.\n\n');
else
    s = [];
    fprintf('  ℹ Running in SIMULATION mode.\n');
    fprintf('  ℹ Generating test signal: 500 Hz (Passband) + 1500 Hz (Stopband) + Noise.\n\n');
end

%% ──────────────────────── INITIALIZE BUFFERS ───────────────────────────
inputBuffer   = zeros(DISPLAY_SAMPLES, 1);
decBuffer     = zeros(DISPLAY_DEC, 1);
zi            = zeros(N_FIR - 1, 1);
residualBytes = uint8([]);

latencyHistory = [];
bufferCount    = 0;
totalSamples   = 0;

% Simulation tracking counters
simSampleIdx = 0;
simLastTime  = tic;

%% ──────────────────────── FIGURE SETUP ─────────────────────────────────
fig = figure('Name', 'Real-Time Multirate DSP Receiver', ...
             'NumberTitle', 'off', ...
             'Color', [0.12 0.12 0.15], ...
             'Position', [100 100 1400 800], ...
             'CloseRequestFcn', @(~,~) assignin('base', 'figClosed', true));

figClosed = false;

t_input = (0:DISPLAY_SAMPLES-1)' / Fs * 1000;
t_dec   = (0:DISPLAY_DEC-1)' / Fs_dec * 1000;

N_FFT_in  = DISPLAY_SAMPLES;
N_FFT_dec = DISPLAY_DEC;
f_input   = (0:N_FFT_in/2-1)' * Fs / N_FFT_in;
f_dec     = (0:N_FFT_dec/2-1)' * Fs_dec / N_FFT_dec;

% ── Subplot 1: Input Time Domain ──
ax1 = subplot(2, 2, 1, 'Parent', fig);
h_input_time = plot(ax1, t_input, inputBuffer, 'Color', [0.25 0.80 1.0], 'LineWidth', 1.0);
title(ax1, 'Input Signal — Time Domain (8 kHz)', 'Color', 'w', 'FontSize', 12);
xlabel(ax1, 'Time (ms)', 'Color', [0.7 0.7 0.7]);
ylabel(ax1, 'Amplitude (ADC counts)', 'Color', [0.7 0.7 0.7]);
set(ax1, 'Color', [0.16 0.16 0.20], 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
ylim(ax1, [-2200 2200]);
grid(ax1, 'on'); ax1.GridColor = [0.3 0.3 0.35];

% ── Subplot 2: Input FFT Spectrum ──
ax2 = subplot(2, 2, 2, 'Parent', fig);
h_input_fft = plot(ax2, f_input, zeros(length(f_input), 1), 'Color', [1.0 0.55 0.25], 'LineWidth', 1.0);
title(ax2, 'Input Signal — FFT Spectrum', 'Color', 'w', 'FontSize', 12);
xlabel(ax2, 'Frequency (Hz)', 'Color', [0.7 0.7 0.7]);
ylabel(ax2, 'Magnitude (dB)', 'Color', [0.7 0.7 0.7]);
set(ax2, 'Color', [0.16 0.16 0.20], 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
ylim(ax2, [-80 60]);
grid(ax2, 'on'); ax2.GridColor = [0.3 0.3 0.35];

% ── Subplot 3: Decimated Time Domain ──
ax3 = subplot(2, 2, 3, 'Parent', fig);
h_dec_time = plot(ax3, t_dec, decBuffer, 'Color', [0.40 1.0 0.55], 'LineWidth', 1.2);
title(ax3, sprintf('Decimated Signal — Time Domain (%d Hz, M=%d)', Fs_dec, M), 'Color', 'w', 'FontSize', 12);
xlabel(ax3, 'Time (ms)', 'Color', [0.7 0.7 0.7]);
ylabel(ax3, 'Amplitude (ADC counts)', 'Color', [0.7 0.7 0.7]);
set(ax3, 'Color', [0.16 0.16 0.20], 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
ylim(ax3, [-2200 2200]);
grid(ax3, 'on'); ax3.GridColor = [0.3 0.3 0.35];

% ── Subplot 4: Decimated FFT Spectrum ──
ax4 = subplot(2, 2, 4, 'Parent', fig);
h_dec_fft = plot(ax4, f_dec, zeros(length(f_dec), 1), 'Color', [0.85 0.35 1.0], 'LineWidth', 1.2);
title(ax4, sprintf('Decimated Signal — FFT Spectrum (%d Hz)', Fs_dec), 'Color', 'w', 'FontSize', 12);
xlabel(ax4, 'Frequency (Hz)', 'Color', [0.7 0.7 0.7]);
ylabel(ax4, 'Magnitude (dB)', 'Color', [0.7 0.7 0.7]);
set(ax4, 'Color', [0.16 0.16 0.20], 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
ylim(ax4, [-80 60]);
grid(ax4, 'on'); ax4.GridColor = [0.3 0.3 0.35];

sgtitle('Real-Time Multirate DSP Receiver', 'Color', 'w', 'FontSize', 15, 'FontWeight', 'bold');

drawnow;
fprintf('  ✓ Live figure initialized. Close figure or Ctrl+C to stop.\n');
fprintf('  ⏳ Processing data stream...\n\n');

%% ──────────────────────── MAIN ACQUISITION LOOP ────────────────────────
lastPlotTime = tic;

try
    while ~figClosed && isvalid(fig)
        
        % ── 1. Acquire Bytes (Hardware OR Simulation) ──
        if ~USE_SIMULATION
            nAvail = s.NumBytesAvailable;
            if nAvail < FRAME_LEN
                pause(0.005);
                drawnow limitrate;
                continue;
            end
            nRead = min(nAvail, READ_CHUNK_BYTES);
            rawBytes = read(s, nRead, 'uint8');
        else
            % Simulate real-time streaming at rate Fs
            dt = toc(simLastTime);
            simLastTime = tic;
            nSamplesToGen = round(Fs * dt);
            
            if nSamplesToGen < 16
                pause(0.005);
                drawnow limitrate;
                continue;
            end

            t_sim = (simSampleIdx : simSampleIdx + nSamplesToGen - 1)' / Fs;
            simSampleIdx = simSampleIdx + nSamplesToGen;

            % 500 Hz (Passband) + 1500 Hz (Stopband / Anti-aliased out) + Noise
            sig = 1200 * sin(2*pi*500*t_sim) + 600 * sin(2*pi*1500*t_sim) + 40 * randn(size(t_sim));
            sig = max(-2048, min(2047, sig)); % Clamp to int16 12-bit range

            % Convert to ESP32 binary frames: [0xAA, 0x55, LoByte, HiByte]
            int16_vals = int16(sig);
            bytes_2d   = reshape(typecast(int16_vals, 'uint8'), 2, []);
            
            simFrameMat = uint8(zeros(4, nSamplesToGen));
            simFrameMat(1,:) = SYNC_HIGH;
            simFrameMat(2,:) = SYNC_LOW;
            simFrameMat(3,:) = bytes_2d(1,:);
            simFrameMat(4,:) = bytes_2d(2,:);
            
            rawBytes = simFrameMat(:)';
        end

        % ── 2. Parse Binary Frames ──
        rawBytes = [residualBytes, rawBytes(:)']; %#ok<AGROW>
        [newSamples, residualBytes] = parseBinaryFrames(rawBytes, SYNC_HIGH, SYNC_LOW);

        if isempty(newSamples)
            continue;
        end

        % ── 3. Timing & DSP Pipeline ──
        tic;
        nNew = length(newSamples);
        totalSamples = totalSamples + nNew;

        % FIR Anti-Aliasing Filter
        [filteredSamples, zi] = filter(b, 1, newSamples, zi);

        % Downsample by M=4
        decimatedSamples = filteredSamples(1:M:end);

        processingTime = toc;
        bufferCount = bufferCount + 1;
        MACs_this_buffer = N_FIR * nNew;
        latencyHistory = [latencyHistory; processingTime]; %#ok<AGROW>

        if mod(bufferCount, 50) == 0
            avgLatency = mean(latencyHistory(max(1,end-49):end));
            maxLatency = max(latencyHistory(max(1,end-49):end));
            fprintf(['  [Buffer #%5d] Samples: %4d | Latency: %.3f ms (avg) ' ...
                     '/ %.3f ms (max) | MACs: %s | Total: %s samples\n'], ...
                    bufferCount, nNew, avgLatency * 1000, maxLatency * 1000, ...
                    addCommas(MACs_this_buffer), addCommas(totalSamples));
        end

        % ── 4. Circular Display Buffers ──
        if nNew >= DISPLAY_SAMPLES
            inputBuffer = newSamples(end-DISPLAY_SAMPLES+1:end);
        else
            inputBuffer = [inputBuffer(nNew+1:end); newSamples];
        end

        nDec = length(decimatedSamples);
        if nDec >= DISPLAY_DEC
            decBuffer = decimatedSamples(end-DISPLAY_DEC+1:end);
        elseif nDec > 0
            decBuffer = [decBuffer(nDec+1:end); decimatedSamples];
        end

        % ── 5. Update Plots (Throttled) ──
        if toc(lastPlotTime) >= UPDATE_INTERVAL_S
            lastPlotTime = tic;

            set(h_input_time, 'YData', inputBuffer);

            Y_in   = fft(inputBuffer .* hanning(DISPLAY_SAMPLES), N_FFT_in);
            mag_in = 20 * log10(abs(Y_in(1:N_FFT_in/2)) / N_FFT_in + 1e-12);
            set(h_input_fft, 'YData', mag_in);

            set(h_dec_time, 'YData', decBuffer);

            Y_dec   = fft(decBuffer .* hanning(DISPLAY_DEC), N_FFT_dec);
            mag_dec = 20 * log10(abs(Y_dec(1:N_FFT_dec/2)) / N_FFT_dec + 1e-12);
            set(h_dec_fft, 'YData', mag_dec);

            drawnow limitrate;
        end
    end
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
        fprintf(2, '\n  ✗ Error in main loop: %s\n', ME.message);
        fprintf(2, '    %s (line %d)\n', ME.stack(1).file, ME.stack(1).line);
    end
end

%% ──────────────────────── FINAL REPORT ─────────────────────────────────
fprintf('\n═══════════════════════════════════════════════════════════\n');
fprintf('  SESSION SUMMARY\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  Total samples processed : %s\n', addCommas(totalSamples));
fprintf('  Total buffers processed : %d\n', bufferCount);
if ~isempty(latencyHistory)
    fprintf('  Avg processing latency  : %.4f ms\n', mean(latencyHistory) * 1000);
    fprintf('  Max processing latency  : %.4f ms\n', max(latencyHistory) * 1000);
end
fprintf('═══════════════════════════════════════════════════════════\n');

%% ════════════════════════ HELPER FUNCTIONS ══════════════════════════════

function [samples, residual] = parseBinaryFrames(rawBytes, syncHi, syncLo)
    nBytes  = length(rawBytes);
    samples = zeros(floor(nBytes / 4), 1);
    count   = 0;
    i       = 1;

    while i <= nBytes - 3
        if rawBytes(i) == syncHi && rawBytes(i+1) == syncLo
            lo = uint16(rawBytes(i + 2));
            hi = uint16(rawBytes(i + 3));
            raw16 = bitor(bitshift(hi, 8), lo);

            if raw16 >= 32768
                val = double(raw16) - 65536;
            else
                val = double(raw16);
            end

            count = count + 1;
            samples(count) = val;
            i = i + 4;
        else
            i = i + 1;
        end
    end

    samples = samples(1:count);
    if i <= nBytes
        residual = rawBytes(i:end);
    else
        residual = uint8([]);
    end
end

function cleanupSerial(s)
    if isempty(s) || ~isvalid(s)
        return;
    end
    fprintf('\n  🔌 Cleaning up serial port...\n');
    try
        flush(s);
        delete(s);
        fprintf('  ✓ Serial port closed successfully.\n');
    catch
        fprintf(2, '  ⚠ Serial port cleanup encountered an error.\n');
    end
end

function str = addCommas(num)
    str = num2str(num);
    n   = length(str);
    if n <= 3, return; end
    nCommas = floor((n - 1) / 3);
    result  = char(zeros(1, n + nCommas));
    j = length(result);
    digitCount = 0;
    for k = n:-1:1
        digitCount = digitCount + 1;
        result(j) = str(k);
        j = j - 1;
        if mod(digitCount, 3) == 0 && k > 1
            result(j) = ',';
            j = j - 1;
        end
    end
    str = result;
end

function val = ternary(cond, trueVal, falseVal)
    if cond, val = trueVal; else, val = falseVal; end
end