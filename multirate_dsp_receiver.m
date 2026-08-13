%% ========================================================================
%  MULTIRATE DSP RECEIVER — Real-Time Hardware-in-the-Loop
%  ========================================================================
%
%  PURPOSE:
%    Receives binary-framed ADC samples from an ESP32 over USB-Serial,
%    performs real-time multirate decimation (FIR anti-aliasing + downsample
%    by M=4), computes assessment metrics (latency, MAC count, Big-O),
%    and displays a live 4-subplot figure.
%
%  USAGE:
%    1. Upload esp32_multirate_dsp.ino to your ESP32.
%    2. Set COM_PORT below to your ESP32's COM port.
%    3. Run this script in MATLAB (R2020a+ required for serialport).
%    4. Press Ctrl+C or close the figure to stop gracefully.
%
%  BINARY FRAME FORMAT (from ESP32):
%    [0xAA] [0x55] [Lo Byte] [Hi Byte]   — 4 bytes per sample
%    int16 little-endian, range: −2048 to +2047
%
%  Author : AI-Generated (Antigravity)
%  Date   : 2026-08-13
% =========================================================================

%% ──────────────────────── CONFIGURATION ────────────────────────────────
clear; clc; close all;

% ── Serial Port Settings ──
COM_PORT   = 'COM3';       % <<< CHANGE THIS to your ESP32's COM port
BAUD_RATE  = 500000;       % Must match ESP32 firmware
TIMEOUT_S  = 10;           % Serial read timeout (seconds)

% ── DSP Parameters ──
Fs         = 8000;         % Input sample rate from ESP32 (Hz)
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
% Anti-aliasing low-pass filter for decimation by M = 4.
%
% Requirements:
%   - Passband  : 0 – 800 Hz  (preserve signal content)
%   - Stopband  : > 1000 Hz   (Nyquist of decimated rate = Fs_dec/2)
%   - Transition: 800 – 1000 Hz
%   - Stopband attenuation: ≥ 60 dB
%
% We use fir1() (windowed-sinc with Hamming window) for simplicity.
% Normalized cutoff: Wn = fc / (Fs/2) = 900 / 4000 = 0.225

fc_hz = 900;                              % Cutoff frequency (Hz)
Wn    = fc_hz / (Fs / 2);                 % Normalized cutoff (0 to 1)
b     = fir1(N_FIR - 1, Wn);              % 48-tap FIR coefficients

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  MULTIRATE DSP RECEIVER — Assessment Metrics\n');
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  Input Sample Rate    : %d Hz\n', Fs);
fprintf('  Decimation Factor    : %d\n', M);
fprintf('  Output Sample Rate   : %d Hz\n', Fs_dec);
fprintf('  FIR Filter Taps      : %d\n', N_FIR);
fprintf('  FIR Cutoff Frequency : %d Hz\n', fc_hz);
fprintf('  Serial Baud Rate     : %d\n', BAUD_RATE);
fprintf('───────────────────────────────────────────────────────────\n');

% ── Computational Complexity Analysis ──
% Standard FIR Decimation (filter ALL, then downsample):
%   - Each output sample requires N_FIR multiply-accumulate (MAC) operations.
%   - We compute Fs output samples/sec, then discard (M-1)/M of them.
%   - Total MACs/sec = N_FIR × Fs
%
% Polyphase Decimation (theoretical optimum):
%   - Only compute samples that survive downsampling.
%   - Total MACs/sec = N_FIR × (Fs / M)
%
% Big-O Notation:
%   - Standard : O(N · Fs)     per second
%   - Polyphase: O(N · Fs/M)   per second
%   - Per buffer of L samples:
%     Standard : O(N · L)
%     Polyphase: O(N · L/M)

MACs_standard  = N_FIR * Fs;
MACs_polyphase = N_FIR * Fs_dec;

fprintf('\n  ── Computational Complexity ──\n');
fprintf('  Standard FIR Decimation:\n');
fprintf('    MACs/sec     = N × Fs     = %d × %d = %s MACs/s\n', ...
        N_FIR, Fs, addCommas(MACs_standard));
fprintf('    Big-O        = O(N · Fs)  per second\n');
fprintf('    Big-O/buffer = O(N · L)   per buffer of L samples\n');
fprintf('  Polyphase Decimation (theoretical):\n');
fprintf('    MACs/sec     = N × Fs/M   = %d × %d = %s MACs/s\n', ...
        N_FIR, Fs_dec, addCommas(MACs_polyphase));
fprintf('    Big-O        = O(N · Fs/M) per second\n');
fprintf('    Efficiency gain: %.1f× fewer operations\n', ...
        MACs_standard / MACs_polyphase);
fprintf('───────────────────────────────────────────────────────────\n\n');

%% ──────────────────────── SERIAL PORT SETUP ────────────────────────────
fprintf('  Opening serial port %s at %d baud...\n', COM_PORT, BAUD_RATE);

try
    s = serialport(COM_PORT, BAUD_RATE);
    configureTerminator(s, 'LF');  % Default terminator (unused for binary reads)
    s.Timeout = TIMEOUT_S;
    fprintf('  ✓ Serial port opened successfully.\n');
catch ME
    fprintf(2, '  ✗ Failed to open serial port: %s\n', ME.message);
    fprintf(2, '    → Check COM_PORT setting and ensure ESP32 is connected.\n');
    fprintf(2, '    → Ensure no other application (Serial Monitor) has the port open.\n');
    return;
end

% ── Robust Cleanup Handler ──
% Ensures the serial port is closed even if the script is interrupted
% with Ctrl+C or an error occurs. Uses onCleanup() which fires when
% the enclosing function/script scope exits for any reason.
cleanupObj = onCleanup(@() cleanupSerial(s));

% Flush any stale data in the serial buffer (ESP32 banner text, etc.)
pause(1.0);  % Wait for ESP32 to finish transmitting its startup banner
flush(s);
fprintf('  ✓ Serial buffer flushed.\n\n');

%% ──────────────────────── INITIALIZE BUFFERS ───────────────────────────

% Circular display buffers for the high-rate and decimated signals.
inputBuffer  = zeros(DISPLAY_SAMPLES, 1);   % High-rate signal history
decBuffer    = zeros(DISPLAY_DEC, 1);        % Decimated signal history

% FIR filter state — persistent across processing blocks.
% filter() with zi allows seamless filtering across buffer boundaries
% without transient artifacts after the first buffer.
zi = zeros(N_FIR - 1, 1);

% Residual bytes from the last serial read that didn't form a complete frame.
residualBytes = uint8([]);

% Latency tracking
latencyHistory = [];
bufferCount    = 0;
totalSamples   = 0;

%% ──────────────────────── FIGURE SETUP ─────────────────────────────────
% Create a figure with 4 subplots arranged in a 2×2 grid:
%   Top row    : High-rate input signal (time domain + FFT)
%   Bottom row : Decimated output signal (time domain + FFT)

fig = figure('Name', 'Real-Time Multirate DSP — Hardware-in-the-Loop', ...
             'NumberTitle', 'off', ...
             'Color', [0.12 0.12 0.15], ...    % Dark background
             'Position', [100 100 1400 800], ...
             'CloseRequestFcn', @(~,~) assignin('base', 'figClosed', true));

% Flag for clean shutdown on figure close
figClosed = false;

% ── Time axes ──
t_input = (0:DISPLAY_SAMPLES-1)' / Fs * 1000;       % ms
t_dec   = (0:DISPLAY_DEC-1)' / Fs_dec * 1000;        % ms

% ── Frequency axes ──
N_FFT_in  = DISPLAY_SAMPLES;
N_FFT_dec = DISPLAY_DEC;
f_input   = (0:N_FFT_in/2-1)' * Fs / N_FFT_in;       % Hz
f_dec     = (0:N_FFT_dec/2-1)' * Fs_dec / N_FFT_dec;  % Hz

% ── Subplot 1: Input Time Domain ──
ax1 = subplot(2, 2, 1, 'Parent', fig);
h_input_time = plot(ax1, t_input, inputBuffer, 'Color', [0.25 0.80 1.0], 'LineWidth', 1.0);
title(ax1, 'Input Signal — Time Domain (8 kHz)', 'Color', 'w', 'FontSize', 12);
xlabel(ax1, 'Time (ms)', 'Color', [0.7 0.7 0.7]);
ylabel(ax1, 'Amplitude (ADC counts)', 'Color', [0.7 0.7 0.7]);
set(ax1, 'Color', [0.16 0.16 0.20], 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
ylim(ax1, [-2200 2200]);
grid(ax1, 'on');
ax1.GridColor = [0.3 0.3 0.35];

% ── Subplot 2: Input FFT Spectrum ──
ax2 = subplot(2, 2, 2, 'Parent', fig);
h_input_fft = plot(ax2, f_input, zeros(length(f_input), 1), ...
                   'Color', [1.0 0.55 0.25], 'LineWidth', 1.0);
title(ax2, 'Input Signal — FFT Spectrum', 'Color', 'w', 'FontSize', 12);
xlabel(ax2, 'Frequency (Hz)', 'Color', [0.7 0.7 0.7]);
ylabel(ax2, 'Magnitude (dB)', 'Color', [0.7 0.7 0.7]);
set(ax2, 'Color', [0.16 0.16 0.20], 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
ylim(ax2, [-80 60]);
grid(ax2, 'on');
ax2.GridColor = [0.3 0.3 0.35];

% ── Subplot 3: Decimated Time Domain ──
ax3 = subplot(2, 2, 3, 'Parent', fig);
h_dec_time = plot(ax3, t_dec, decBuffer, 'Color', [0.40 1.0 0.55], 'LineWidth', 1.2);
title(ax3, sprintf('Decimated Signal — Time Domain (%d Hz, M=%d)', Fs_dec, M), ...
      'Color', 'w', 'FontSize', 12);
xlabel(ax3, 'Time (ms)', 'Color', [0.7 0.7 0.7]);
ylabel(ax3, 'Amplitude (ADC counts)', 'Color', [0.7 0.7 0.7]);
set(ax3, 'Color', [0.16 0.16 0.20], 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
ylim(ax3, [-2200 2200]);
grid(ax3, 'on');
ax3.GridColor = [0.3 0.3 0.35];

% ── Subplot 4: Decimated FFT Spectrum ──
ax4 = subplot(2, 2, 4, 'Parent', fig);
h_dec_fft = plot(ax4, f_dec, zeros(length(f_dec), 1), ...
                 'Color', [0.85 0.35 1.0], 'LineWidth', 1.2);
title(ax4, sprintf('Decimated Signal — FFT Spectrum (%d Hz)', Fs_dec), ...
      'Color', 'w', 'FontSize', 12);
xlabel(ax4, 'Frequency (Hz)', 'Color', [0.7 0.7 0.7]);
ylabel(ax4, 'Magnitude (dB)', 'Color', [0.7 0.7 0.7]);
set(ax4, 'Color', [0.16 0.16 0.20], 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5]);
ylim(ax4, [-80 60]);
grid(ax4, 'on');
ax4.GridColor = [0.3 0.3 0.35];

% ── Supertitle ──
sgtitle('Real-Time Multirate DSP — ESP32 Hardware-in-the-Loop', ...
        'Color', 'w', 'FontSize', 15, 'FontWeight', 'bold');

% Force initial render
drawnow;
fprintf('  ✓ Live figure initialized. Close figure or Ctrl+C to stop.\n');
fprintf('  ⏳ Waiting for data...\n\n');

%% ──────────────────────── MAIN ACQUISITION LOOP ────────────────────────
% Runs indefinitely until the figure is closed or Ctrl+C is pressed.
% Each iteration:
%   1. Reads available bytes from serial port
%   2. Parses binary frames (with residual byte handling)
%   3. Applies FIR anti-aliasing filter
%   4. Downsamples by factor M
%   5. Updates circular display buffers
%   6. Refreshes live plots at a throttled rate
%   7. Computes and prints latency metrics

lastPlotTime = tic;

try
    while ~figClosed && isvalid(fig)
        % ── 1. Read Available Bytes ──
        nAvail = s.NumBytesAvailable;
        if nAvail < FRAME_LEN
            % Not enough data for even one frame — yield CPU briefly.
            pause(0.005);
            drawnow limitrate;  % Check for figure close events
            continue;
        end

        % Limit read size to prevent MATLAB from stalling on very large
        % serial buffers.
        nRead = min(nAvail, READ_CHUNK_BYTES);
        rawBytes = read(s, nRead, 'uint8');

        % ── 2. Parse Binary Frames ──
        % Prepend any residual bytes from the previous iteration.
        rawBytes = [residualBytes, rawBytes(:)'];  %#ok<AGROW>

        [newSamples, residualBytes] = parseBinaryFrames(rawBytes, ...
                                                        SYNC_HIGH, SYNC_LOW);

        if isempty(newSamples)
            continue;  % No complete frames found
        end

        % ── 3. Timing Measurement ──
        tic;

        nNew = length(newSamples);
        totalSamples = totalSamples + nNew;

        % ── 4. FIR Anti-Aliasing Filter ──
        % filter(b, a, x, zi) maintains internal state across calls,
        % ensuring seamless filtering across buffer boundaries.
        [filteredSamples, zi] = filter(b, 1, newSamples, zi);

        % ── 5. Downsample by M ──
        % Take every M-th sample from the filtered output.
        % This is the standard decimation approach: filter first, then
        % select every M-th sample.
        decimatedSamples = filteredSamples(1:M:end);

        % ── 6. Measure Processing Latency ──
        processingTime = toc;  % seconds
        bufferCount = bufferCount + 1;

        % Compute per-buffer metrics
        MACs_this_buffer = N_FIR * nNew;  % Standard decimation: filter ALL samples
        latencyHistory = [latencyHistory; processingTime];  %#ok<AGROW>

        % Print metrics every 50 buffers (~5 seconds)
        if mod(bufferCount, 50) == 0
            avgLatency = mean(latencyHistory(max(1,end-49):end));
            maxLatency = max(latencyHistory(max(1,end-49):end));
            fprintf(['  [Buffer #%5d] Samples: %4d | Latency: %.3f ms (avg) ' ...
                     '/ %.3f ms (max) | MACs: %s | Total: %s samples\n'], ...
                    bufferCount, nNew, avgLatency * 1000, maxLatency * 1000, ...
                    addCommas(MACs_this_buffer), addCommas(totalSamples));
        end

        % ── 7. Update Circular Display Buffers ──
        % Shift old data left, append new data on the right.

        % High-rate input buffer
        if nNew >= DISPLAY_SAMPLES
            inputBuffer = newSamples(end-DISPLAY_SAMPLES+1:end);
        else
            inputBuffer = [inputBuffer(nNew+1:end); newSamples];
        end

        % Decimated buffer
        nDec = length(decimatedSamples);
        if nDec >= DISPLAY_DEC
            decBuffer = decimatedSamples(end-DISPLAY_DEC+1:end);
        elseif nDec > 0
            decBuffer = [decBuffer(nDec+1:end); decimatedSamples];
        end

        % ── 8. Update Live Plot (Throttled) ──
        if toc(lastPlotTime) >= UPDATE_INTERVAL_S
            lastPlotTime = tic;

            % -- Input time domain --
            set(h_input_time, 'YData', inputBuffer);

            % -- Input FFT (magnitude in dB) --
            Y_in    = fft(inputBuffer .* hanning(DISPLAY_SAMPLES), N_FFT_in);
            mag_in  = 20 * log10(abs(Y_in(1:N_FFT_in/2)) / N_FFT_in + 1e-12);
            set(h_input_fft, 'YData', mag_in);

            % -- Decimated time domain --
            set(h_dec_time, 'YData', decBuffer);

            % -- Decimated FFT (magnitude in dB) --
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
fprintf('  Total samples acquired : %s\n', addCommas(totalSamples));
fprintf('  Total buffers processed: %d\n', bufferCount);
if ~isempty(latencyHistory)
    fprintf('  Avg processing latency : %.4f ms\n', mean(latencyHistory) * 1000);
    fprintf('  Max processing latency : %.4f ms\n', max(latencyHistory) * 1000);
    fprintf('  Min processing latency : %.4f ms\n', min(latencyHistory) * 1000);
end
fprintf('  Standard FIR MACs/sec  : %s\n', addCommas(MACs_standard));
fprintf('  Polyphase MACs/sec     : %s (theoretical optimum)\n', addCommas(MACs_polyphase));
fprintf('  Polyphase speedup      : %.1f×\n', MACs_standard / MACs_polyphase);
fprintf('═══════════════════════════════════════════════════════════\n');

%% ════════════════════════ HELPER FUNCTIONS ══════════════════════════════

function [samples, residual] = parseBinaryFrames(rawBytes, syncHi, syncLo)
% PARSEBINARYFRAMES  Extract int16 samples from binary-framed byte stream.
%
%   [samples, residual] = parseBinaryFrames(rawBytes, syncHi, syncLo)
%
%   Scans rawBytes for the sync pattern [syncHi syncLo], extracts the
%   following two bytes as a little-endian int16 sample, and advances
%   by 4 bytes. Any bytes that don't match the sync pattern are skipped
%   (re-synchronization). Incomplete frames at the end are returned
%   as 'residual' for prepending to the next read.
%
%   INPUTS:
%     rawBytes  — uint8 row vector of raw serial bytes
%     syncHi    — first sync byte (0xAA)
%     syncLo    — second sync byte (0x55)
%
%   OUTPUTS:
%     samples   — column vector of double-precision sample values
%     residual  — uint8 row vector of leftover bytes (0 to 3 bytes)

    nBytes  = length(rawBytes);
    samples = zeros(floor(nBytes / 4), 1);  % Pre-allocate (upper bound)
    count   = 0;
    i       = 1;

    while i <= nBytes - 3  % Need at least 4 bytes for one complete frame
        if rawBytes(i) == syncHi && rawBytes(i+1) == syncLo
            % ── Valid sync found ──
            % Extract little-endian int16 from bytes [i+2, i+3]
            lo = uint16(rawBytes(i + 2));
            hi = uint16(rawBytes(i + 3));
            raw16 = bitor(bitshift(hi, 8), lo);

            % Convert uint16 to signed int16 (two's complement)
            if raw16 >= 32768
                val = double(raw16) - 65536;
            else
                val = double(raw16);
            end

            count = count + 1;
            samples(count) = val;
            i = i + 4;  % Advance past this frame
        else
            % ── Sync mismatch — scan forward one byte ──
            % This handles byte misalignment after noise/startup.
            i = i + 1;
        end
    end

    % Trim pre-allocated array to actual sample count
    samples = samples(1:count);

    % Save any remaining bytes (0 to 3) for the next call
    if i <= nBytes
        residual = rawBytes(i:end);
    else
        residual = uint8([]);
    end
end

function cleanupSerial(s)
% CLEANUPSERIAL  Safely close the serial port and release resources.
%   Called automatically by onCleanup when the script exits (normally,
%   via Ctrl+C, or due to an error).

    fprintf('\n  🔌 Cleaning up serial port...\n');
    try
        flush(s);           % Flush any remaining buffered data
        delete(s);          % Close and release the serial port
        clear s;            % Remove from workspace
        fprintf('  ✓ Serial port closed successfully.\n');
    catch
        fprintf(2, '  ⚠ Serial port cleanup encountered an error (may already be closed).\n');
    end
end

function str = addCommas(num)
% ADDCOMMAS  Format a number with comma separators for readability.
%   addCommas(384000) → '384,000'

    str = num2str(num);
    n   = length(str);
    if n <= 3
        return;
    end
    % Insert commas from right to left
    nCommas  = floor((n - 1) / 3);
    result   = char(zeros(1, n + nCommas));
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
