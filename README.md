# Palletizing Robot

Welcome to the **Palletizing Robot** project! This is a **4 DOF robot** that’s got a little bit of everything: precision, automation, and repurposed hardware from an old printer (yes, really!). It uses **3 prismatic actuators** and a **rotational joint** in the end effector to perform an efficient pick-and-place operation.

## What’s Inside?

### The Mechanics
- **3 Prismatic Actuators**: These are powered by **DC motors with encoders**, each controlled by an optimized **PID controller** in **Arduino**. The cool part? **Interrupts** handle position updates without delay. So no waiting around for your robot to make its move.
- **1 Rotational Joint**: This gives the robot the ability to rotate and pick up items with precision.

**Note**: The actuators and sensors were repurposed from an old printer—don’t let your recycling pile go to waste, folks! These components can still do cool stuff when given a second chance.

### Control System
This isn’t your typical robot with a joystick or complicated software interface. The robot is controlled by a set of **physical buttons**. The magic behind it is a **finite state machine (FSM)** that governs the robot’s state, meaning you can **Start**, **Pause**, **Stop**, or **Restart** the robot with zero delays. Yep, everything happens in real time.

### Pick-and-Place: It’s Preprogrammed
The robot is set up with a **predefined pick-and-place sequence**, and here's the cool part:
- Fixed **starting position** and **target angle**.
- It calculates the **positions of 6 pallets** from these fixed parameters.
So, all you need to do is press a button, and the robot gets to work—no guessing or recalibration.

### No Delays, Just Action
The code is designed with **no delays**—everything runs based on internal logic. That means the robot is always ready to move, no blocking code in the way. The pick-and-place operation flows seamlessly from start to finish.

### Setup
1. **Compile the code** into your **Arduino Mega**.
2. Hook up the motors, encoders, L298N motor drivers, and buttons.
3. Plug in your **24V power supply** and fire it up!

## What’s Next? (Future Improvements)

Sadly, this project has been **abandoned** (the robot got disassembled 😢), but the code is still golden and could be adapted to other projects. If you want to take this robot to the next level, here are some ideas:

- **Cartesian Trajectory Planning**: Imagine the robot moving all 3 axes at the same time. That would make pick-and-place even smoother and more efficient.
- **Class-based PID Controls**: Tidy up the PID control code with classes to keep everything organized and easier to manage.
- **Speed and Acceleration Limits**: Add limits to speed and acceleration to prevent any mishaps like dropping your precious cargo.

## Why Bother?

This project shows that even old, seemingly useless hardware (like printers) can be turned into something useful and fun. It’s perfect for anyone interested in robotics, automation, and reusing components creatively.

The code is fast, efficient, and doesn't rely on delays—ideal for real-time control systems. So, take this as a reference, make it your own, and who knows? You might just take this palletizing robot to new heights.

Let the robot do the work—no delays, just precision.

Enjoy!


Credits

- José Vásquez: Project Lead, Software Development, Control Systems, and Mechatronics Design
- Christian Batista: Mechanical and Electrical Assembly
- Lucía Araya: Mechanical and Electrical Assembly
- Ian Cambronero: Mechanical Design and Assembly
- Carlos Cortés: Mechanical Assembly and Software Design
