# Real-Time Audio DSP: ESP32 & MATLAB

This project connects a physical microcontroller (an ESP32) to a computer program (MATLAB) to process audio signals in real-time. 

In engineering terms, this is called a **Hardware-in-the-Loop (HIL)** setup. It just means we are testing a real piece of physical hardware (the ESP32) alongside a software simulation (MATLAB).

## What does this project do?

The goal of this project is to take an analog audio signal (like a tone from your phone), read it into the ESP32, send that data to your computer as fast as possible, and use MATLAB to process and display the signal.

Specifically, the MATLAB code applies **Multirate Digital Signal Processing**. It takes a fast signal (8000 samples per second) and slows it down to a lower rate (2000 samples per second). 

## What is inside this repository?

There are two main folders in this project:

1. **`esp32_firmware`**: Contains the C++ code for the ESP32 (to be opened in the Arduino IDE).
2. **`matlab`**: Contains the MATLAB script (`.m` file) that runs on your computer.

## How it works (and why we made certain choices)

### 1. Reading the Audio (ESP32)
We need to read the audio signal exactly 8,000 times every second. 
* **Why not use a simple `delay()`?** If we just loop and wait, any small slowdown in the code will ruin the timing. 
* **Our solution:** We use a **Hardware Timer**. This is a physical clock inside the ESP32 that forces the chip to read the audio at an exact, uninterrupted schedule.

### 2. Sending the Data to the Computer (ESP32 to USB)
Once the ESP32 reads the audio, it needs to send it to the computer over the USB cable (Serial connection).
* **Why not send text?** If we send the number `1500` as text, it takes up 4 to 6 bytes of space. At 8000 times a second, that is too much data, and the connection will crash.
* **Our solution:** We send the data in **Binary**. We pack the number into exactly 2 bytes. We also add a secret "sync code" (like a secret handshake) so MATLAB knows exactly where a new number starts. We also set the connection speed (baud rate) to 500,000, which is very fast.

### 3. Processing the Data (MATLAB)
MATLAB receives the fast audio signal (8000 samples/sec). We want to shrink this down to 2000 samples/sec (divide by 4). This is called **decimation**.
* **Why not just throw away 3 out of every 4 numbers?** If you do this with audio, high-pitched noises will glitch and turn into low-pitched rumbles. This math error is called **aliasing**.
* **Our solution:** Before we throw away any numbers, we pass the audio through a **Low-Pass Filter** (specifically, an FIR filter). This filter acts like a bouncer at a club, kicking out all the high frequencies. Once the high frequencies are gone, it is safe to throw away the extra numbers. 

MATLAB then plots the original fast signal and the new, slower signal so you can compare them side-by-side.

## How to use this project

### Hardware Setup
1. You need an ESP32 Development Board.
2. Connect your audio source to **GPIO Pin 34** on the ESP32. 
3. *Note on Audio:* The ESP32 cannot read negative voltages. You need to push your audio signal up by 1.65 Volts (using a simple resistor divider circuit) so the wave sits in the middle of the ESP32's reading range (0 to 3.3 Volts). The ESP32 code will automatically subtract this extra voltage later.
4. Plug the ESP32 into your computer via USB.

### Software Setup
1. **ESP32 Code:** 
   * Open the `.ino` file in the `esp32_firmware` folder using the Arduino IDE.
   * Upload the code to your ESP32.
   * **Important:** Keep the Arduino "Serial Monitor" closed. If it is open, MATLAB will not be able to connect to the ESP32.
2. **MATLAB Code:**
   * Open the `.m` script in the `matlab` folder.
   * Look for the line that says `COM_PORT = 'COM3';` and change `'COM3'` to match the port your ESP32 is plugged into (you can check this in your computer's Device Manager or the Arduino IDE).
   * Run the script. A window will pop up showing live graphs of your audio signal!
   * To stop it, just close the graph window.
