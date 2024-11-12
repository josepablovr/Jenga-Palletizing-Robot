# Palletizing Robot

Welcome to the **Palletizing Robot** project! This is a **4 DOF robot** that combines precision, automation, and repurposed hardware from an old printer (yes, really!). It uses **3 prismatic actuators** and a **rotational joint** in the end effector to perform efficient pick-and-place operations.

## What’s Inside?

### The Mechanics
- **3 Prismatic Actuators**: These are driven by **DC motors with encoders**, each controlled by an optimized **PID controller** in **Arduino**. The standout feature? **Interrupts** handle position updates without delay, ensuring the robot responds immediately.
- **1 Rotational Joint**: This allows the robot to rotate and pick up items with high precision.
- **Personalized Gripper**: DC actuated gripper built for picking up Jenga pieces.

**Note**: The actuators and sensors came from old printers meant for recycling.

### Control System
This robot doesn't rely on a joystick or complex software interface. Instead, it is controlled via a set of **physical buttons**. The core of the control system is a **finite state machine (FSM)** that governs the robot’s operations, so you can easily **Start**, **Pause**, **Stop**, or **Restart** the robot without any delays.

### Pick-and-Place: It’s Preprogrammed
The robot operates with a **predefined pick-and-place sequence**, which simplifies the task:
- A fixed **starting position** and **target angle**.
- It calculates the **positions of 6 pallets** based on these fixed parameters.
Just press a button, and the robot will execute the sequence—no recalibration needed.

### No Delays, Just Action
The system is built to work with **no delays**. Everything runs based on internal logic, meaning the robot is always ready to move, and there’s no blocking code. The pick-and-place operation runs smoothly from start to finish.

### Setup
1. **Compile the code** onto your **Arduino Mega**.
2. Connect the motors, encoders, L298N motor drivers, and buttons.
3. Plug in your **24V power supply** and the robot is ready to go!

## What’s Next? (Future Improvements)

Although this project has been **abandoned** (the robot was disassembled 😢), the code remains highly functional and could be adapted for future projects. Here are some ideas for enhancing the robot:

- **Cartesian Trajectory Planning**: Moving all 3 axes simultaneously would make the pick-and-place sequence even more fluid and efficient.
- **Class-based PID Controls**: Refactoring the PID control logic into classes would improve code organization and maintainability.
- **Speed and Acceleration Limits**: Introducing speed and acceleration limits could prevent issues like dropping items during operation.

## Why It Matters

This project demonstrates that even old hardware, like printers, can be repurposed into functional robotic systems. It’s an excellent starting point for anyone interested in robotics, automation, and sustainable practices through creative reuse of components.

The code is fast, efficient, and designed to run without delays—making it perfect for real-time control applications. Use this as a reference, customize it for your own needs, and who knows? You might take this palletizing robot to the next level.

Let the robot do the work—precisely and without interruption.

Enjoy!

## Final Demo

Now, see the Jenga Palletizing robot in action!

https://github.com/user-attachments/assets/d43d5cb7-fa35-47aa-a776-7560850deeca

Credits:

- José Vásquez: Project Lead, Software Development, Control Systems, and Mechatronics Design
- Christian Batista: Mechanical and Electrical Assembly
- Lucía Araya: Mechanical and Electrical Assembly
- Ian Cambronero: Mechanical Design and Assembly
- Carlos Cortés: Mechanical Assembly and Software Design

Course: Robotics | Mechatronics Engineering | Costa Rica Institute of Technology  | 2022
