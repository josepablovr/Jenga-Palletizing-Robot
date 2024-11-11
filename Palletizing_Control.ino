#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#include <Servo.h>

Servo servo; // Declare a Servo object to control the servo motor

// Constants for piece dimensions
const int DistanceBetweenPieces = 28; // Distance between pieces (in mm)
const int PieceHeight = 17;           // Height of each piece (in mm)

// Coordinates for the robot's end effector position
int posX = 230; // X position of the end effector
int posY = 210; // Y position of the end effector
int posZ = -38; // Z position of the end effector (negative for downward)
int Orientation = 0; // Orientation angle of the end effector

// Button definitions for controlling the robot
#define STOP 44        
#define START 45        
#define CALIBRATION 47  
#define RESET 46       

// Motor and encoder pins for the Y-axis
#define ENCA_Y 21        // Encoder A for Y-axis
#define ENCB_Y 22        // Encoder B for Y-axis
#define PWM_Y 5          // PWM pin for Y-axis motor control
#define IN2_Y 26         // Motor direction control pin 2 for Y-axis
#define IN1_Y 24         // Motor direction control pin 1 for Y-axis

// Motor and encoder pins for the X-axis
#define ENCA_X 20        // Encoder A for X-axis
#define ENCB_X 23        // Encoder B for X-axis
#define PWM_X 6          // PWM pin for X-axis motor control
#define IN2_X 30         // Motor direction control pin 2 for X-axis
#define IN1_X 28         // Motor direction control pin 1 for X-axis

// Motor and encoder pins for the Z-axis
#define ENCA_Z 19        // Encoder A for Z-axis
#define ENCB_Z 32        // Encoder B for Z-axis
#define PWM_Z 4          // PWM pin for Z-axis motor control
#define IN2_Z 25         // Motor direction control pin 2 for Z-axis
#define IN1_Z 27         // Motor direction control pin 1 for Z-axis

// Gripper motor control pins
#define PWM_G 3          // PWM pin for gripper motor control
#define IN2_G 36         // Gripper direction control pin 2
#define IN1_G 37         // Gripper direction control pin 1
int pwm_open = 90;     // PWM value to open the gripper
int pwm_close = 100;   // PWM value to close the gripper

// Control parameters for Y-axis
int minPercent_Y = 46;   // Minimum percentage for Y-axis speed
int maxPercent_Y = 52;   // Maximum percentage for Y-axis speed
int minSpeed_Y = 255 * minPercent_Y / 100; // Min speed for Y-axis based on percentage
int maxSpeed_Y = 255 * maxPercent_Y / 100; // Max speed for Y-axis based on percentage
float kp_Y = 0.09;       // Proportional gain for Y-axis PID control
float kd_Y = 0.0;        // Derivative gain for Y-axis PID control
float ki_Y = 0.05;       // Integral gain for Y-axis PID control

// Control parameters for X-axis
int minPercent_X = 32;   // Minimum percentage for X-axis speed
int maxPercent_X = 40;   // Maximum percentage for X-axis speed
int minSpeed_X = 255 * minPercent_X / 100; // Min speed for X-axis based on percentage
int maxSpeed_X = 255 * maxPercent_X / 100; // Max speed for X-axis based on percentage
float kp_X = 0.1;        // Proportional gain for X-axis PID control
float kd_X = 0.0;        // Derivative gain for X-axis PID control
float ki_X = 0.004;      // Integral gain for X-axis PID control

// Control parameters for Z-axis
int minPercent_Z = 55;   // Minimum percentage for Z-axis speed
int maxPercent_Z = 65;   // Maximum percentage for Z-axis speed
int minSpeed_Z = 255 * minPercent_Z / 100; // Min speed for Z-axis based on percentage
int maxSpeed_Z = 255 * maxPercent_Z / 100; // Max speed for Z-axis based on percentage
float kp_Z = 0.005;      // Proportional gain for Z-axis PID control
float kd_Z = 0.0;        // Derivative gain for Z-axis PID control
float ki_Z = 0.0001;     // Integral gain for Z-axis PID control

// Control parameters for Z-axis during descent
float kp_ZD = 0.012;     // Proportional gain for Z-axis descent PID control
float kd_ZD = 0.0;       // Derivative gain for Z-axis descent PID control
float ki_ZD = 0.01;      // Integral gain for Z-axis descent PID control
int minPercent_ZD = 23;  // Minimum percentage for Z-axis descent speed
int maxPercent_ZD = 27;  // Maximum percentage for Z-axis descent speed
int minSpeed_ZD = 255 * minPercent_ZD / 100; // Min speed for Z-axis descent based on percentage
int maxSpeed_ZD = 255 * maxPercent_ZD / 100; // Max speed for Z-axis descent based on percentage

// Control parameters for Z-axis rotation (ZR)
int minSpeedZR = minSpeed_Z;  // Min speed for Z-axis rotation (same as Z-axis)
int maxSpeedZR = maxSpeed_Z;  // Max speed for Z-axis rotation (same as Z-axis)
float kp_ZR = kp_Z;           // Proportional gain for Z-axis rotation PID control
float kd_ZR = kd_Z;           // Derivative gain for Z-axis rotation PID control
float ki_ZR = ki_Z;           // Integral gain for Z-axis rotation PID control

// Movement status flags for each axis
bool moveY = false;  // Flag indicating movement status for Y-axis
bool moveX = false;  // Flag indicating movement status for X-axis
bool moveZ = false;  // Flag indicating movement status for Z-axis
int step = 0;        // Step counter for controlling the movement sequence

// Volatile variables for position tracking with interrupts
volatile signed long posY_interrupt = 0; // Position variable for Y-axis, modified by interrupts
volatile signed long posX_interrupt = 0; // Position variable for X-axis, modified by interrupts
volatile signed long posZ_interrupt = 0; // Position variable for Z-axis, modified by interrupts

// Non-volatile position variables
signed long posY = 0; // Current Y-axis position
signed long posX = 0; // Current X-axis position
signed long posZ = 0; // Current Z-axis position

const int Time = 10; // Time delay for loop or control actions (in milliseconds)

// PID control variables
long prevTime = 0;       // Previous time for calculating time difference in PID
float prevError = 0;     // Previous error for calculating PID derivative
float errorIntegral = 0; // Integral of the error for PID control
unsigned long timer = 0; // Timer for control actions
signed long error = 0;   // Error term for PID control
int power = 0;           // Power output for the motors

// Desired positions for each axis
float DesiredPositionX = 0.0; // Desired X position
float DesiredPositionY = 0.0; // Desired Y position
float DesiredPositionZ = 0.0; // Desired Z position
float DesiredOrientation = 0.0; // Desired orientation (rotation angle)

// Enumeration for robot states
enum PossibleStates {STATE_ZERO, STOP, RESET, CALIBRATE, START};
PossibleStates state = STATE_ZERO; // Initial robot state

int Stage = 0; // Current step in the sequence
int Piece = 0; // Current piece number in the process

void setup() {
  // Start serial communication for debugging and status updates
  Serial.begin(9600);

  // Setup pins for Y-axis motor, encoder, and PWM
  pinMode(ENCA_Y, INPUT);
  pinMode(ENCB_Y, INPUT);
  pinMode(IN1_Y, OUTPUT);
  pinMode(IN2_Y, OUTPUT);
  pinMode(PWM_Y, OUTPUT);
  TCCR3B = TCCR3B & B11111000 | B00000011;  // Set PWM frequency to 490.20 Hz for Y-axis

  // Setup pins for X-axis motor, encoder, and PWM
  pinMode(ENCA_X, INPUT);
  pinMode(ENCB_X, INPUT);
  pinMode(IN1_X, OUTPUT);
  pinMode(IN2_X, OUTPUT);
  pinMode(PWM_X, OUTPUT);

  // Setup pins for Z-axis motor, encoder, and PWM
  pinMode(ENCA_Z, INPUT);
  pinMode(ENCB_Z, INPUT);
  pinMode(IN1_Z, OUTPUT);
  pinMode(IN2_Z, OUTPUT);
  pinMode(PWM_Z, OUTPUT);

  // Setup buttons for control
  pinMode(STOP, INPUT);
  pinMode(START, INPUT);
  pinMode(CALIBRATION, INPUT);
  pinMode(RESET, INPUT);

  // Setup pins for gripper motor
  pinMode(IN1_G, OUTPUT);
  pinMode(IN2_G, OUTPUT);
  pinMode(PWM_G, OUTPUT);

  // Attach interrupts for encoders on all axes
  attachInterrupt(digitalPinToInterrupt(ENCA_Y), readEncoderY, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_X), readEncoderX, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Z), readEncoderZ, RISING);

  // Print startup message to the serial monitor
  Serial.println("START");

  // Initialize motors to stop position
  digitalWrite(IN1_Y, LOW);
  digitalWrite(IN2_Y, LOW);
  digitalWrite(PWM_Y, LOW);
  digitalWrite(IN1_X, LOW);
  digitalWrite(IN2_X, LOW);
  digitalWrite(PWM_X, LOW);
  digitalWrite(IN1_Z, LOW);
  digitalWrite(IN2_Z, LOW);
  digitalWrite(PWM_Z, LOW);

  // Attach the servo motor to pin 2
  servo.attach(2);
}



void loop() {

  switch (state) {  // Switch to determine the current state of the robot
  
  case INITIAL_STATE: {  // Initial state: robot is stopped, waiting for user input
    // Stop all motors by setting relevant pins to LOW
    digitalWrite(IN1_Y, LOW);
    digitalWrite(IN2_Y, LOW);
    digitalWrite(PWM_Y, LOW);
    digitalWrite(IN1_X, LOW);
    digitalWrite(IN2_X, LOW);
    digitalWrite(PWM_X, LOW);
    digitalWrite(IN1_Z, LOW);
    digitalWrite(IN2_Z, LOW);
    digitalWrite(PWM_Z, LOW);

    // If the start button is pressed, move to the "Start" state
    if ( digitalRead(START) == true) {
      state = START;
    }
    break;
  }

  case START: {  // Start state: robot is beginning the pick-and-place sequence
    switch (piece) {  // Check which piece we're working with

      case (0): {
        SetpiecePosition(posX, posY, posZ, orientation, 1);  // Set initial position for piece 0
        piece = 1;
        break;
      }

      case (1): {  // Move to the next position for piece 1
        bool trajectory = MoveToSequence(targetPositionX, targetPositionY, targetPositionZ, targetOrientation, -54);
        if (trajectory == true) {
          piece = 2;
          stage = 0;
<<<<<<< HEAD:Palletizing_Control.ino
          PositionPiece(posX, posY, posZ, orientation, piece);
=======
          PositionPiece(posX, posY, posZ, orientation, part);
>>>>>>> b9c5767 (Translation of Sequence Functions):Control_Paletizador.ino
        }
        break;
      }

      case (2): {
        bool trajectory = MoveToSequence(targetPositionX, targetPositionY, targetPositionZ, targetOrientation, -54);
        if (trajectory == true) {
          piece = 3;
          stage = 0;
<<<<<<< HEAD:Palletizing_Control.ino
          PositionPiece(posX, posY, posZ, orientation, piece);
=======
          PositionPiece(posX, posY, posZ, orientation, part);
>>>>>>> b9c5767 (Translation of Sequence Functions):Control_Paletizador.ino
        }
        break;
      }

      case (3): {
        bool trajectory = MoveToSequence(targetPositionX, targetPositionY, targetPositionZ, targetOrientation, -54);
        if (trajectory == true) {
          piece = 4;
          stage = 0;
<<<<<<< HEAD:Palletizing_Control.ino
          PositionPiece(posX, posY, posZ, orientation, piece);
=======
          PositionPiece(posX, posY, posZ, orientation, part);
>>>>>>> b9c5767 (Translation of Sequence Functions):Control_Paletizador.ino
        }
        break;
      }

      case (4): {
        bool trajectory = MoveToSequence(targetPositionX, targetPositionY, targetPositionZ, targetOrientation, -54);
        if (trajectory == true) {
          piece = 5;
          stage = 0;
<<<<<<< HEAD:Palletizing_Control.ino
          PositionPiece(posX, posY, posZ, orientation, piece);
=======
          PositionPiece(posX, posY, posZ, orientation, part);
>>>>>>> b9c5767 (Translation of Sequence Functions):Control_Paletizador.ino
        }
        break;
      }

      case (5): {
        bool trajectory = MoveToSequence(targetPositionX, targetPositionY, targetPositionZ, targetOrientation, -54);
        if (trajectory == true) {
          piece = 6;
          stage = 0;
<<<<<<< HEAD:Palletizing_Control.ino
          PositionPiece(posX, posY, posZ, orientation, piece);
=======
          PositionPiece(posX, posY, posZ, orientation, part);
>>>>>>> b9c5767 (Translation of Sequence Functions):Control_Paletizador.ino
        }
        break;
      }

      case (6): {  // If piece 6 is finished, reset the cycle
        bool trajectory = MoveToSequence(targetPositionX, targetPositionY, targetPositionZ, targetOrientation, -54);
        if (trajectory == true) {
          piece = 0;
          stage = 0;
<<<<<<< HEAD:Palletizing_Control.ino
          state = STOP;  // Stop the robot once all pieces are processed
          PositionPiece(posX, posY, posZ, orientation, piece);
=======
          state = STOP;  // Stop the robot once all parts are processed
          PositionPiece(posX, posY, posZ, orientation, part);
>>>>>>> b9c5767 (Translation of Sequence Functions):Control_Paletizador.ino
        }
        break;
      }
    }

    // If the stop button is pressed, switch to the "Stop" state
    if ( digitalRead(STOP) == true) {
      state = STOP;
    }
    break;
  }

  case STOP: {  // Stop state: robot halts
    // Stop all motors and turn off the gripper
    digitalWrite(IN1_Y, LOW);
    digitalWrite(IN2_Y, LOW);
    digitalWrite(IN1_X, LOW);
    digitalWrite(IN2_X, LOW);
    digitalWrite(IN1_Z, LOW);
    digitalWrite(IN2_Z, LOW);
    StopGripper();

    // If start is pressed, move to the "Start" state
    if ( digitalRead(START) == true) {
      state = START;
    }

    // If reset is pressed, go to the "Reset" state
    if ( digitalRead(RESET) == true) {
      state = RESET;
    }

    servo.write(0);  // Reset the servo to its initial position
    break;
  }

  case RESET: {  // Reset state: reset the robot to its initial position
    stage = 0;
    // Close the gripper before resetting
    //CloseGripper();

    // If stop is pressed, switch to the stop state
    if ( digitalRead(STOP) == true) {
      state = STOP;
    }

    // Move to the home position (0, 0, 0)
    bool trajectory = MoveToCoordinates(0, 0, 0);
    if (trajectory == true) {
      state = STOP;
      stage = 0;
    }
    break;
  }
  }
}




// Function to control motor direction and speed
void setMotor(int direction, int pwmValue, int pwmPin, int in1Pin, int in2Pin) {
  analogWrite(pwmPin, pwmValue);  // Set the PWM value for speed control

  // Set the motor direction based on the input 'direction'
  if (direction == 1) {  // Move forward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  else if (direction == -1) {  // Move backward
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else {  // Stop the motor
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}


// Interrupt service routine to read the Y-axis encoder
void readEncoderY() {
  int stateB = digitalRead(ENCODER_B_Y);  // Read the state of encoder pin B
  if (stateB > 0) {
    posi_Y++;  // Increment position if signal is high
  }
  else {
    posi_Y--;  // Decrement position if signal is low
  }
}


// Interrupt service routine to read the X-axis encoder
void readEncoderX() {
  int stateB = digitalRead(ENCODER_B_X);  // Read the state of encoder pin B
  if (stateB > 0) {
    posi_X++;  // Increment position if signal is high
  }
  else {
    posi_X--;  // Decrement position if signal is low
  }
}


// Interrupt service routine to read the Z-axis encoder
void readEncoderZ() {
  int stateB = digitalRead(ENCODER_B_Z);  // Read the state of encoder pin B
  if (stateB > 0) {
    posi_Z++;  // Increment position if signal is high
  }
  else {
    posi_Z--;  // Decrement position if signal is low
  }
}

// PID control for the Y-axis
bool PIDY(int desiredDistance) {
  bool movementFinished = false;  // Flag to indicate if the movement has finished
  signed long target = 6 * desiredDistance;  // Target position, scaled by a factor of 6
  long currentTime = micros();  // Current time in microseconds
  float deltaTime = ((float)(currentTime - prevT)) / (1.0e6);  // Calculate time difference in seconds
  prevT = currentTime;  // Update previous time

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_Y = posi_Y;  // Read the current position of Y-axis in a safe way
  }

  // Calculate error (difference between current position and target)
  e = pos_Y - target;

  // Derivative of the error (rate of change)
  float de_dt = (e - eprev) / (deltaTime);

  // Integral of the error (accumulated error over time)
  eintegral = eintegral + e * deltaTime;
  
  // Saturate the integral term to avoid excessive windup
  if (eintegral > velmax_Y - velmin_Y) {
    eintegral = velmax_Y - velmin_Y;
  }
  else if (eintegral < -velmax_Y + velmin_Y) {
    eintegral = -velmax_Y + velmin_Y;
  }

  // PID constants for proportional, derivative, and integral control
  float kp = kp_Y;
  float kd = kd_Y;
  float ki = ki_Y;

  // Control signal calculation using the PID formula
  float u = kp * e + kd * de_dt + ki * eintegral;

  // Calculate power for motor control
  pwr = (int)fabs(u);
  pwr = velmin_Y + pwr;  // Ensure power is within valid range

  // Limit the power to the maximum value
  if (pwr > velmax_Y) {
    pwr = velmax_Y;
  }

  // Determine motor direction based on error
  int direction = 1;  // Default direction is forward
  if (e > 0) {
    direction = -1;  // Reverse direction if the error is positive
  }

  // If the error is small enough (i.e., movement is finished)
  if (abs(e) < 10) {
    setMotor(direction, 0, PWM_Y, IN1_Y, IN2_Y);  // Stop the motor
    e = 0;  // Reset error
    eintegral = 0;  // Reset integral
    movementFinished = true;  // Indicate movement is finished
    return movementFinished;  // Return true, movement finished
  }
  else {
    setMotor(direction, pwr, PWM_Y, IN1_Y, IN2_Y);  // Continue movement
  }

  // Save the current error for the next iteration
  eprev = e;
  return movementFinished;  // Return whether the movement finished
}



// PID control for the X-axis
bool PIDX(signed long desiredDistance_X) {
  // Calculate target position based on desired distance and scaling factor
  signed long target_X = desiredDistance_X * 15400 / (PI * 13);

  bool movementFinished = false;  // Flag to indicate if the movement has finished
  long currentTime = micros();  // Current time in microseconds
  float deltaTime = ((float)(currentTime - prevT)) / (1.0e6);  // Calculate time difference in seconds
  prevT = currentTime;  // Update previous time

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_X = posi_X;  // Read the current position of X-axis in a safe way
  }

  // Calculate error (difference between current position and target)
  e = pos_X - target_X;

  // Derivative of the error (rate of change)
  float de_dt = (e - eprev) / (deltaTime);

  // Integral of the error (accumulated error over time)
  eintegral = eintegral + e * deltaTime;

  // Saturate the integral term to avoid excessive windup
  if (eintegral > velmax_X - velmin_X) {
    eintegral = velmax_X - velmin_X;
  }
  else if (eintegral < -velmax_X + velmin_X) {
    eintegral = -velmax_X + velmin_X;
  }

  // PID constants for proportional, derivative, and integral control
  float kp = kp_X;
  float kd = kd_X;
  float ki = ki_X;

  int direction = -1;  // Default direction is reverse
  float Velmin = velmin_X;

  if (e < 0) {
    direction = 1;  // Forward direction if error is negative
  }
  else if (e > 0) {
    // Adjust PID constants and minimum speed for large errors
    kp = kp * 0.15;
    ki = ki * 0.05;
    Velmin = Velmin * 0.15;
  }

  // Control signal calculation using the PID formula
  float u = kp * e + kd * de_dt + ki * eintegral;

  // Calculate power for motor control
  pwr = (int)fabs(u);
  pwr = Velmin + pwr;  // Ensure power is within valid range

  // Limit the power to the maximum value
  if (pwr > velmax_X) {
    pwr = velmax_X;
  }

  // If the error is small enough (i.e., movement is finished)
  if (abs(e) < 100) {
    setMotor(direction, 0, PWM_X, IN1_X, IN2_X);  // Stop the motor
    e = 0;  // Reset error
    eintegral = 0;  // Reset integral
    movementFinished = true;  // Indicate movement is finished
    return movementFinished;  // Return true, movement finished
  }
  else {
    setMotor(direction, pwr, PWM_X, IN1_X, IN2_X);  // Continue movement
  }

  // Save the current error for the next iteration
  eprev = e;
  return movementFinished;  // Return whether the movement finished
}




// PID control for the Z-axis
bool PIDZ(signed long desiredDistance) {

  // Calculate target position based on desired distance and scaling factor
  signed long target_Z = -1 * desiredDistance * 15400 / (PI * 13);

  bool movementFinished = false;  // Flag to indicate if the movement has finished
  long currentTime = micros();  // Current time in microseconds
  float deltaTime = ((float)(currentTime - prevT)) / (1.0e6);  // Calculate time difference in seconds
  prevT = currentTime;  // Update previous time

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_Z = posi_Z;  // Read the current position of Z-axis in a safe way
  }

  // Calculate error (difference between current position and target)
  e = pos_Z - target_Z;

  // Derivative of the error (rate of change)
  float de_dt = (e - eprev) / (deltaTime);

  // Integral of the error (accumulated error over time)
  eintegral = eintegral + e * deltaTime;

  // Saturate the integral term to avoid excessive windup
  if (eintegral > velmax_Z - velmin_Z) {
    eintegral = velmax_Z - velmin_Z;
  }
  else if (eintegral < -velmax_Z + velmin_Z) {
    eintegral = -velmax_Z + velmin_Z;
  }

  // PID constants for proportional, derivative, and integral control
  float kp = kp_Z;
  float kd = kd_Z;
  float ki = ki_Z;

  // Control signal calculation using the PID formula
  float u = kp * e + kd * de_dt + ki * eintegral;

  // Calculate power for motor control
  pwr = (int)fabs(u);
  pwr = velmin_Z + pwr;  // Ensure power is within valid range

  // Limit the power to the maximum value
  if (pwr > velmax_Z) {
    pwr = velmax_Z;
  }

  // Set motor direction based on the error
  int direction = -1;
  if (e < 0) {
    direction = 1;  // Forward direction if error is negative
  }

  // Adjust PID constants and speed for different directions
  if (direction == -1) {
    kp_Z = kp_ZR;
    kd_Z = kd_ZR;
    ki_Z = ki_ZR;
    velmin_Z = velminZR;
    velmax_Z = velmaxZR;
  }
  else {
    kp_Z = kp_ZD;
    kd_Z = kd_ZD;
    ki_Z = ki_ZD;
    velmin_Z = velmin_ZD;
    velmax_Z = velmax_ZD;
  }

  // If the error is small enough (i.e., movement is finished)
  if (abs(e) < 100) {
    setMotor(direction, 0, PWM_Z, IN1_Z, IN2_Z);  // Stop the motor
    e = 0;  // Reset error
    eintegral = 0;  // Reset integral
    movementFinished = true;  // Indicate movement is finished
    return movementFinished;  // Return true, movement finished
  }
  else {
    setMotor(direction, pwr, PWM_Z, IN1_Z, IN2_Z);  // Continue movement
  }

  // Save the current error for the next iteration
  eprev = e;
  return movementFinished;  // Return whether the movement finished
}

// Function to open the gripper
void OpenGripper() {
  analogWrite(PWM_G, pwm_abrir);  // Open gripper
  digitalWrite(IN1_G, HIGH);      // Set motor direction
  digitalWrite(IN2_G, LOW);
}

// Function to close the gripper with a specific intensity
void CloseGripper(int intensity) {
  analogWrite(PWM_G, intensity);  // Control gripper force
  digitalWrite(IN1_G, LOW);        // Set motor direction
  digitalWrite(IN2_G, HIGH);
}

// Function to turn off the gripper
void TurnOffGripper() {
  analogWrite(PWM_G, 0);           // Stop gripper
  digitalWrite(IN1_G, LOW);        // Stop motor
  digitalWrite(IN2_G, LOW);
}

// Function to move the robot to a specific position in XYZ coordinates
bool MoveToCoordinateXYZ(int x, int y, int z) {
  bool movement_finished = false;
  switch (step) {
    case 0:  // Move in X direction
      moveX = PIDX(x);
      if (moveX) step = 1;
      break;
    case 1:  // Move in Y direction
      moveY = PIDY(y);
      if (moveY) step = 2;
      break;
    case 2:  // Move in Z direction
      moveZ = PIDZ(z);
      if (moveZ) step = 3;
      break;
    case 3:  // End movement
      movement_finished = true;
      moveX = moveY = moveZ = false;
      step = 0;
      break;
  }
  return movement_finished;
}

// Function to move the robot to a specific position in ZYX order
bool MoveToCoordinateZYX(int z, int y, int x) {
  bool movement_finished = false;
  switch (step) {
    case 0:  // Move in Z direction
      moveZ = PIDZ(z);
      if (moveZ) step = 1;
      break;
    case 1:  // Move in Y direction
      moveY = PIDY(y);
      if (moveY) step = 2;
      break;
    case 2:  // Move in X direction
      moveX = PIDX(x);
      if (moveX) step = 3;
      break;
    case 3:  // End movement
      movement_finished = true;
      moveX = moveY = moveZ = false;
      step = 0;
      break;
  }
  return movement_finished;
}

// Function to grab an object
bool GrabObject(int z) {
  bool movement_finished = false;
  switch (step) {
    case 0: {
      servo.write(0);  // Move gripper to initial position
      delay(1000);
      step = 1;
      break;
    }
    case 1: {
      moveZ = PIDZ(z);
      if (moveZ) step = 2;
      break;
    }
    case 2: {
      delay(500);  // Wait before closing gripper
      moveZ = false;
      CloseGripper(pwm_cerrar);  // Close gripper
      delay(1500);
      step = 3;
      break;
    }
    case 3: {
      moveZ = PIDZ(0);
      if (moveZ) {
        movement_finished = true;
        step = 0;
      }
      break;
    }
  }
  return movement_finished;
}

// Function to release an object
bool ReleaseObject(int z) {
  bool movement_finished = false;
  switch (step) {
    case 0: {
      moveZ = PIDZ(z);
      if (moveZ) step = 1;
      break;
    }
    case 1: {
      moveZ = false;
      delay(1000);  // Wait before opening gripper
      OpenGripper();  // Open gripper
      delay(1500);
      step = 2;
      break;
    }
    case 2: {
      moveZ = PIDZ(0);
      if (moveZ) {
        step = 0;
        movement_finished = true;
      }
      break;
    }
  }
  return movement_finished;
}

// Function to perform the complete sequence of grabbing, moving, and releasing the object
bool MoveToSequence(int x, int y, int z, int angle, int releaseHeight) {
  bool movement_finished = false;
  switch (stage) {
    case 0: {
      bool grab = GrabObject(releaseHeight);
      if (grab) {
        moveX = moveY = moveZ = false;
        stage = 1;
      }
      break;
    }
    case 1: {
      bool moveToXYZ = MoveToCoordinateXYZ(x, y, 0);
      CloseGripper(pwm_cerrar * 0.65);  // Close gripper with reduced intensity
      if (moveToXYZ) {
        moveToXYZ = false;
        moveX = moveY = moveZ = false;
        stage = 2;
      }
      break;
    }
    case 2: {
      servo.write(angle);  // Move to the desired angle
      delay(1000);
      stage = 3;
      break;
    }
    case 3: {
      bool release = ReleaseObject(z);
      if (release) {
        moveX = moveY = moveZ = false;
        stage = 4;
      }
      break;
    }
    case 4: {
      bool returnToOrigin = MoveToCoordinateXYZ(0, 0, 0);
      if (returnToOrigin) {
        moveX = moveY = moveZ = false;
        stage = 0;
        movement_finished = true;
      }
      break;
    }
  }
  return movement_finished;
}



// Function to position the piece at a specific location
void PositionPiece(int X, int Y, int Z, int Ang, int n) {
  float deltaX = DistanceBetweenPieces * cos(Ang * PI / 180);
  float deltaY = DistanceBetweenPieces * sin(Ang * PI / 180);
  float deltaZ = PieceHeight;

  float Orientation = 0;
  float posX = 0;
  float posY = 0;
  float posZ = 0;
  
  switch(n) {
    case 1:
      posX = X - deltaX;
      posY = Y + deltaY;
      posZ = Z;
      Orientation = Ang;
      break;
    case 2:
      posX = X;
      posY = Y;
      posZ = Z;
      Orientation = Ang;
      break;
    case 3:
      posX = X + deltaX;
      posY = Y - deltaY;
      posZ = Z;
      Orientation = Ang;
      break;
    case 4:
      posX = X + deltaY;
      posY = Y + deltaX;
      posZ = Z + deltaZ;
      Orientation = Ang + 95;
      break;
    case 5:
      posX = X;
      posY = Y;
      posZ = Z + deltaZ;
      Orientation = Ang + 95;
      break;
    case 6:
      posX = X - deltaY;
      posY = Y - deltaX;
      posZ = Z + deltaZ;
      Orientation = Ang + 95;
      break;
  }
  
  DesiredPositionX = -posX;
  DesiredPositionY = posY;
  DesiredPositionZ = posZ;
  DesiredPositionOrientation = Orientation;
}

// Function to generate a trajectory (linear interpolation)
float* GenerateLinearTrajectory(int posX, int posY, int Orientation, int Time, int moment) {
  float aX = posX / Time;
  float aY = posX / Time;
  float aO = Orientation / Time;

  float targetX = aX * moment;
  float targetY = aY * moment;
  float targetO = aO * moment;
  float parameters[3] = {targetX, targetY, targetO};
  return parameters;
}

// Function to generate a trajectory (cubic interpolation)
float* GenerateCubicTrajectory(int posX, int posY, int Orientation, int Time) {
  float a0X = pos_X;
  float a2X = 3 / pow(Time, 2) * (posX - pos_X);
  float a3X = 3 / pow(Time, 3) * (posX - pos_X);
  
  float aY = posX / Time;
  float aO = Orientation / Time;

  float parameters[3] = {a0X, a2X, a3X};
  return parameters;
}
