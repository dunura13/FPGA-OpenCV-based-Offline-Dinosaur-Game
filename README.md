# Ofline Dinosaur Game 🦖

📖 Project Overview
This project replaces the traditional keyboard input of the "T-Rex Runner" game with full-body motion control. A Python script running on a host computer uses a webcam to track the player's movements (Jumping, Ducking, Idling) and transmits these commands via UART to an Altera Cyclone V FPGA (DE1-SoC).

The FPGA hosts the entire game logic, physics engine, and VGA rendering pipeline in hardware, offering a lag-free, hardware-accelerated gaming experience.

🚀 Key Features
Hybrid Architecture: Combines high-level software processing (Python/OpenCV) with low-level hardware logic (Verilog).

Computer Vision Controller: Uses MediaPipe Pose to track nose and body position relative to a calibrated baseline.

Jump: Detects upward movement.

Duck: Detects downward movement.

Calibration: Includes an auto-calibration routine (holding an open palm above the head).

Hardware Game Engine:

Custom physics FSM for gravity and projectile motion.

Sprite-based rendering (Dino, Obstacles, Game Over screens).

Dynamic scoring and difficulty scaling.

Collision detection using Axis-Aligned Bounding Boxes (AABB).

UART Communication: Custom 115200 baud UART receiver implemented in Verilog to bridge the PC and FPGA.

🛠️ System Architecture
The system consists of two main subsystems communicating over a serial link:

Host PC (Vision Processing): Captures video, calculates body metrics, and sends single-byte commands ('J', 'D', 'I').

FPGA (Game Logic): Receives commands, updates game state, and drives the VGA display.

(Use the high-level block diagrams discussed previously here)

🔧 Hardware & Software Requirements
Hardware
FPGA Board: DE1-SoC (Cyclone V) or similar.

Display: VGA Monitor (320x240 resolution).

Communication: USB-to-TTL Serial Adapter (FTDI FT232RL / SH-U09C).

Camera: Standard USB Webcam.

🧠 Technical Highlights
Sprite Rendering: Implemented a "Ghost Cleanup" logic in the hardware to erase the previous frame's sprite position before drawing the new one, ensuring smooth motion without artifacts.

Debouncing & Smoothing: The Python script uses Exponential Moving Average (EMA) to smooth out camera jitter, and the Verilog UART receiver uses oversampling to prevent false triggers.

Dynamic Hitboxes: The collision logic adjusts the dinosaur's bounding box in real-time when the "Duck" signal is received.


👥 Credits
Dunura Epasinghe, Seeron Sivashankumar, Neo Li - System Architecture, Verilog Implementation, Computer Vision Integration.


























