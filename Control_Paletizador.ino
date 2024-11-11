#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#include <Servo.h>

Servo servo; // Declare a Servo object to control the servo motor

// Constants for piece dimensions
const int DistanciaEntrePiezas = 28; // Distance between pieces (in mm)
const int AlturadePieza = 17;        // Height of each piece (in mm)

// Coordinates for the robot's end effector position
int posX = 230; // X position of the end effector
int posY = 210; // Y position of the end effector
int posZ = -38; // Z position of the end effector (negative for downward)
int Orientacion = 0; // Orientation angle of the end effector

// Button definitions for controlling the robot
#define PARO 44          // Stop button
#define INICIO 45        // Start button
#define CALIBRACION 47   // Calibration button
#define REINICIO 46      // Restart button

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
int pwm_abrir = 90;    // PWM value to open the gripper
int pwm_cerrar = 100;  // PWM value to close the gripper

// Control parameters for Y-axis
int pormin_Y = 46;     // Minimum percentage for Y-axis speed
int pormax_Y = 52;     // Maximum percentage for Y-axis speed
int velmin_Y = 255 * pormin_Y / 100; // Min speed for Y-axis based on percentage
int velmax_Y = 255 * pormax_Y / 100; // Max speed for Y-axis based on percentage
float kp_Y = 0.09;     // Proportional gain for Y-axis PID control
float kd_Y = 0.0;      // Derivative gain for Y-axis PID control
float ki_Y = 0.05;     // Integral gain for Y-axis PID control

// Control parameters for X-axis
int pormin_X = 32;     // Minimum percentage for X-axis speed
int pormax_X = 40;     // Maximum percentage for X-axis speed
int velmin_X = 255 * pormin_X / 100; // Min speed for X-axis based on percentage
int velmax_X = 255 * pormax_X / 100; // Max speed for X-axis based on percentage
float kp_X = 0.1;      // Proportional gain for X-axis PID control
float kd_X = 0.0;      // Derivative gain for X-axis PID control
float ki_X = 0.004;    // Integral gain for X-axis PID control

// Control parameters for Z-axis
int pormin_Z = 55;     // Minimum percentage for Z-axis speed
int pormax_Z = 65;     // Maximum percentage for Z-axis speed
int velmin_Z = 255 * pormin_Z / 100; // Min speed for Z-axis based on percentage
int velmax_Z = 255 * pormax_Z / 100; // Max speed for Z-axis based on percentage
float kp_Z = 0.005;    // Proportional gain for Z-axis PID control
float kd_Z = 0.0;      // Derivative gain for Z-axis PID control
float ki_Z = 0.0001;   // Integral gain for Z-axis PID control

// Control parameters for Z-axis during descent
float kp_ZD = 0.012;   // Proportional gain for Z-axis descent PID control
float kd_ZD = 0.0;     // Derivative gain for Z-axis descent PID control
float ki_ZD = 0.01;    // Integral gain for Z-axis descent PID control
int pormin_ZD = 23;    // Minimum percentage for Z-axis descent speed
int pormax_ZD = 27;    // Maximum percentage for Z-axis descent speed
int velmin_ZD = 255 * pormin_ZD / 100; // Min speed for Z-axis descent based on percentage
int velmax_ZD = 255 * pormax_ZD / 100; // Max speed for Z-axis descent based on percentage

// Control parameters for Z-axis rotation (ZR)
int velminZR = velmin_Z;  // Min speed for Z-axis rotation (same as Z-axis)
int velmaxZR = velmax_Z;  // Max speed for Z-axis rotation (same as Z-axis)
float kp_ZR = kp_Z;       // Proportional gain for Z-axis rotation PID control
float kd_ZR = kd_Z;       // Derivative gain for Z-axis rotation PID control
float ki_ZR = ki_Z;       // Integral gain for Z-axis rotation PID control

// Movement status flags for each axis
bool avanceY = false; // Flag indicating movement status for Y-axis
bool avanceX = false; // Flag indicating movement status for X-axis
bool avanceZ = false; // Flag indicating movement status for Z-axis
int paso = 0;         // Step counter for controlling the movement sequence

// Volatile variables for position tracking with interrupts
volatile signed long posi_Y = 0; // Position variable for Y-axis, modified by interrupts
volatile signed long posi_X = 0; // Position variable for X-axis, modified by interrupts
volatile signed long posi_Z = 0; // Position variable for Z-axis, modified by interrupts

// Non-volatile position variables
signed long pos_Y = 0; // Current Y-axis position
signed long pos_X = 0; // Current X-axis position
signed long pos_Z = 0; // Current Z-axis position

const int Tiempo = 10; // Time delay for loop or control actions (in milliseconds)

// PID control variables
long prevT = 0;       // Previous time for calculating time difference in PID
float eprev = 0;      // Previous error for calculating PID derivative
float eintegral = 0;  // Integral of the error for PID control
unsigned long Timer = 0; // Timer for control actions
signed long e = 0;    // Error term for PID control
int pwr = 0;          // Power output for the motors

// Desired positions for each axis
float PosicionDeseadaX = 0.0; // Desired X position
float PosicionDeseadaY = 0.0; // Desired Y position
float PosicionDeseadaZ = 0.0; // Desired Z position
float PosicionDeseadaO = 0.0; // Desired orientation (rotation angle)

// Enumeration for robot states
enum PosiblesEstados {ESTADOCERO, PARAR, REINICIAR, CALIBRAR, INICIAR};
PosiblesEstados estado = ESTADOCERO; // Initial robot state

int Etapa = 0; // Current step in the sequence
int Pieza = 0; // Current piece number in the process

void setup() {

  Serial.begin(9600);
  //EJE Y
  pinMode(ENCA_Y, INPUT);
  pinMode(ENCB_Y, INPUT);
  pinMode(IN1_Y, OUTPUT);
  pinMode(IN2_Y, OUTPUT);
  pinMode(PWM_Y, OUTPUT);

  TCCR3B = TCCR3B & B11111000 | B00000011;  // for PWM frequency of 490.20 Hz

  //EJE X
  pinMode(ENCA_X, INPUT);
  pinMode(ENCB_X, INPUT);
  pinMode(IN1_X, OUTPUT);
  pinMode(IN2_X, OUTPUT);
  pinMode(PWM_X, OUTPUT);

  //EJE Z
  pinMode(ENCA_Z, INPUT);
  pinMode(ENCB_Z, INPUT);
  pinMode(IN1_Z, OUTPUT);
  pinMode(IN2_Z, OUTPUT);
  pinMode(PWM_Z, OUTPUT);


  //BOTONES
  pinMode(PARO, INPUT);
  pinMode(INICIO, INPUT);
  pinMode(CALIBRACION, INPUT);
  pinMode(REINICIO, INPUT);

  //GARRA
  pinMode(IN1_G, OUTPUT);
  pinMode(IN2_G, OUTPUT);
  pinMode(PWM_G, OUTPUT);

  //INTERRUPCIONES
  attachInterrupt(digitalPinToInterrupt(ENCA_Y), readEncoderY, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_X), readEncoderX, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Z), readEncoderZ, RISING);

  Serial.println("INICIO");
  digitalWrite(IN1_Y, LOW);
  digitalWrite(IN2_Y, LOW);
  digitalWrite(PWM_Y, LOW);
  digitalWrite(IN1_X, LOW);
  digitalWrite(IN2_X, LOW);
  digitalWrite(PWM_X, LOW);
  digitalWrite(IN1_Z, LOW);
  digitalWrite(IN2_Z, LOW);
  digitalWrite(PWM_Z, LOW);
  servo.attach(2);

  
}


void loop() {
  Serial.println(posi_Y);
  switch (estado) {
    
    case ESTADOCERO: {
        digitalWrite(IN1_Y, LOW);
        digitalWrite(IN2_Y, LOW);
        digitalWrite(PWM_Y, LOW);
        digitalWrite(IN1_X, LOW);
        digitalWrite(IN2_X, LOW);
        digitalWrite(PWM_X, LOW);
        digitalWrite(IN1_Z, LOW);
        digitalWrite(IN2_Z, LOW);
        digitalWrite(PWM_Z, LOW);
        //Serial.println("INICIO");
        //servo.write(0);
        if ( digitalRead(INICIO) == true) {
          estado = INICIAR;
        }
        //CerrarGarra();
        break;
      }

//    case INICIAR: {
//        //bool avanzarcoordenada = AvanzarCoordenadaXYZ(-250, 220, -20);
//        Pieza = 0;

//        switch (Pieza) {
//          case (1): {
//              bool trayectoria = Secuencia(-250, 245, -50, 10, -50);
//              if (trayectoria == true) {
//                Pieza = 2;
//                Etapa = 0;
//              } break;
//            }
//          case (2): {
//              bool trayectoria = Secuencia(-220, 245, -50, 10, -50);
//              if (trayectoria == true) {
//                Pieza = 3;
//                Etapa = 0;
//              } break;
//            }
//          case (3): {
//              bool trayectoria = Secuencia(-200, 245, -50, 10, -50);
//              if (trayectoria == true) {
//                Pieza = 4;
//                Etapa = 0;
//              } break;
//            }
//          case (4): {
//              bool trayectoria = Secuencia(-210, 270, -35, 100, -50);
//              if (trayectoria == true) {
//                Pieza = 5;
//                Etapa = 0;
//              }
//            } break;
//
//          case (5): {
//              bool trayectoria = Secuencia(-210, 245, -35, 100, -50);
//              if (trayectoria == true) {
//                Pieza = 6;
//                Etapa = 0;
//              }
//              break;
//            }
//          case (6): {
//              bool trayectoria = Secuencia(-210, 220, -35, 100, -50);
//              if (trayectoria == true) {
//                Pieza = 0;
//                Etapa = 0;
//                estado = PARAR;
//              }
//              break;
//            }
//        }
//
//        CerrarGarra();
//        
//        if ( digitalRead(PARO) == true) {
//          estado = PARAR;
//        }
//        break;
//      }
    case INICIAR: {
        //bool avanzarcoordenada = AvanzarCoordenadaXYZ(-250, 220, -20);
       
        switch (Pieza) {
          case (0): {
            PosicionPieza(posX, posY, posZ, Orientacion, 1);
            Pieza = 1;
          }

          
          case (1): {
              bool trayectoria = Secuencia(PosicionDeseadaX, PosicionDeseadaY, PosicionDeseadaZ, PosicionDeseadaO, -54);
              if (trayectoria == true) {
                Pieza = 2;
                Etapa = 0;
                PosicionPieza(posX, posY, posZ, Orientacion, Pieza);
              } break;
            }
          case (2): {
              bool trayectoria = Secuencia(PosicionDeseadaX, PosicionDeseadaY, PosicionDeseadaZ, PosicionDeseadaO, -54);
              if (trayectoria == true) {
                Pieza = 3;
                Etapa = 0;
                PosicionPieza(posX, posY, posZ, Orientacion, Pieza);
              } break;
            }
          case (3): {
              bool trayectoria = Secuencia(PosicionDeseadaX, PosicionDeseadaY, PosicionDeseadaZ, PosicionDeseadaO, -54);
              if (trayectoria == true) {
                Pieza = 4;
                Etapa = 0;
                PosicionPieza(posX, posY, posZ, Orientacion, Pieza);
              } break;
            }
          case (4): {
              bool trayectoria = Secuencia(PosicionDeseadaX, PosicionDeseadaY, PosicionDeseadaZ, PosicionDeseadaO, -54);
              if (trayectoria == true) {
                Pieza = 5;
                Etapa = 0;
                PosicionPieza(posX, posY, posZ, Orientacion, Pieza);
              }
            } break;

          case (5): {
              bool trayectoria = Secuencia(PosicionDeseadaX, PosicionDeseadaY, PosicionDeseadaZ, PosicionDeseadaO, -54);
              if (trayectoria == true) {
                Pieza = 6;
                Etapa = 0;
                PosicionPieza(posX, posY, posZ, Orientacion, Pieza);
              }
              break;
            }
          case (6): {
              bool trayectoria = Secuencia(PosicionDeseadaX, PosicionDeseadaY, PosicionDeseadaZ, PosicionDeseadaO, -54);
              if (trayectoria == true) {
                Pieza = 0;
                Etapa = 0;
                estado = PARAR;
                PosicionPieza(posX, posY, posZ, Orientacion, Pieza);
              }
              break;
            }
        }

        
        
        if ( digitalRead(PARO) == true) {
          estado = PARAR;
        }
        break;
      }
    case PARAR: {
        digitalWrite(IN1_Y, LOW);
        digitalWrite(IN2_Y, LOW);
        digitalWrite(IN1_X, LOW);
        digitalWrite(IN2_X, LOW);
        digitalWrite(IN1_Z, LOW);
        digitalWrite(IN2_Z, LOW);
        ApagarGarra();
        if ( digitalRead(INICIO) == true) {
          estado = INICIAR;
        }
        if ( digitalRead(REINICIO) == true) {
          estado = REINICIAR;
        }
        servo.write(0);

       

        
        break;
      }
    case REINICIAR:  {
        Etapa = 0;
        //CerrarGarra();
        if ( digitalRead(PARO) == true) {
          estado = PARAR;
        }


        bool trayectoria = AvanzarCoordenadaXYZ (0, 0, 0);
        if (trayectoria == true) {
          estado = PARAR;
          Etapa = 0;
              } 
        break;
      }

  }
}










void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoderY() {
  int b = digitalRead(ENCB_Y);
  if (b > 0) {
    posi_Y++;
  }
  else {
    posi_Y--;
  }
}

void readEncoderX() {
  int b = digitalRead(ENCB_X);
  if (b > 0) {
    posi_X++;
  }
  else {
    posi_X--;
  }
}

void readEncoderZ() {
  int b = digitalRead(ENCB_Z);
  if (b > 0) {
    posi_Z++;
  }
  else {
    posi_Z--;
  }
}

bool PIDY (int distanciadeseada) {
  bool finmovimiento = false;
  signed long target = 6 * distanciadeseada;
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_Y = posi_Y;
  }
  // error
  e = pos_Y - target;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;
  if (eintegral > velmax_Y - velmin_Y) {
    eintegral = velmax_Y - velmin_Y;
  }
  else if (eintegral < -velmax_Y + velmin_Y) {
    eintegral = -velmax_Y + velmin_Y;
  }

  float kp = kp_Y;
  float kd = kd_Y;
  float ki = ki_Y;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // Acción de control
  pwr = (int)fabs(u);
  pwr = velmin_Y + pwr;

  if ( pwr > velmax_Y ) {
    pwr = velmax_Y;

  }
  // motor direction
  int dir = 1;
  if (e > 0) {
    dir = -1;
  }

  if (abs(e) < 10) {
    setMotor(dir, 0, PWM_Y, IN1_Y, IN2_Y);
    e = 0;
    eintegral = 0;
    finmovimiento = true;
    return finmovimiento;
  }
  else {
    setMotor(dir, pwr, PWM_Y, IN1_Y, IN2_Y);
  }



  // Guarda el error previo
  eprev = e;
  return finmovimiento;

}

bool PIDX (signed long distanciadeseada_X) {
  signed long target_X = distanciadeseada_X * 15400 / (PI * 13);

  bool finmovimiento = false;
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_X = posi_X;
  }
  // error
  e = pos_X - target_X;
  //Serial.println(target_X);
  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;
  if (eintegral > velmax_X - velmin_X) {
    eintegral = velmax_X - velmin_X;
  }
  else if (eintegral < -velmax_X + velmin_X) {
    eintegral = -velmax_X + velmin_X;
  }

  float kp = kp_X;
  float kd = kd_X;
  float ki = ki_X;
  int dir = -1;
  float Velmin = velmin_X;
  if (e < 0) {
    dir = 1;
  }

  
  
  else if (e > 0) {
    kp = kp*0.15;
    ki = ki*0.05;
    float Velmin = Velmin*0.15;
  }
  
  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // Acción de control
  pwr = (int)fabs(u);
  pwr = Velmin + pwr;

  if ( pwr > velmax_X ) {
    pwr = velmax_X;

  }
  // motor direction
  

  if (abs(e) < 100) {
    setMotor(dir, 0, PWM_X, IN1_X, IN2_X);
    e = 0;
    eintegral = 0;
    finmovimiento = true;
    return finmovimiento;
  }
  else {
    setMotor(dir, pwr, PWM_X, IN1_X, IN2_X);
  }



  // Guarda el error previo
  eprev = e;
  return finmovimiento;

}

bool PIDZ (signed long distanciadeseada) {

  signed long target_Z = -1 * distanciadeseada * 15400 / (PI * 13);

  bool finmovimiento = false;
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_Z = posi_Z;
  }
  // error
  e = pos_Z - target_Z;
  //Serial.println(target_Z);
  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;
  if (eintegral > velmax_Z - velmin_Z) {
    eintegral = velmax_Z - velmin_Z;
  }
  else if (eintegral < -velmax_Z + velmin_Z) {
    eintegral = -velmax_Z + velmin_Z;
  }
  float kp = kp_Z;
  float kd = kd_Z;
  float ki = ki_Z;


  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // Acción de control
  pwr = (int)fabs(u);
  pwr = velmin_Z + pwr;

  if ( pwr > velmin_Z ) {
    pwr = velmax_Z;

  }
  // motor direction
  int dir = -1;
  if (e < 0) {
    dir = 1;
  }
  if (dir == -1) {
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
  if (abs(e) < 100) {
    setMotor(dir, 0, PWM_Z, IN1_Z, IN2_Z);
    e = 0;
    eintegral = 0;
    finmovimiento = true;
    return finmovimiento;
  }
  else {
    setMotor(dir, pwr, PWM_Z, IN1_Z, IN2_Z);
  }



  // Guarda el error previo
  eprev = e;
  return finmovimiento;

}


void AbrirGarra() {
  analogWrite(PWM_G, pwm_abrir);
  digitalWrite(IN1_G, HIGH);
  digitalWrite(IN2_G, LOW);
}

void CerrarGarra(int intensidad) {
  analogWrite(PWM_G, intensidad);
  digitalWrite(IN1_G, LOW);
  digitalWrite(IN2_G, HIGH);
}
void ApagarGarra() {
  analogWrite(PWM_G, 0);
  digitalWrite(IN1_G, LOW);
  digitalWrite(IN2_G, LOW);
}



bool AvanzarCoordenadaXYZ (int x, int y, int z) {
  bool avance_terminado = false;
  switch (paso) {
    case 0: {
        avanceX = PIDX(x);
        
        if (avanceX == true) {
          paso = 1;
        }
        break;
      }
    case 1: {
        avanceY = PIDY(y);
        if (avanceY == true) {
          paso = 2;
        }
        break;
      }
    case 2: {
        avanceZ = PIDZ(z);
        if (avanceZ == true) {
          paso = 3;
        }
        break;
      }

    case 3: {
        avance_terminado = true;
        avanceX = false;
        avanceY = false;
        avanceZ = false;
        paso = 0;
        break;
      }

  }
  return avance_terminado;
}

bool AvanzarCoordenadaZYX (int z, int y, int x) {
  bool avance_terminado = false;
  switch (paso) {
    case 0: {
        avanceZ = PIDZ(z);
        if (avanceZ == true) {
          paso = 1;
        }
        break;
      }
    case 1: {
        avanceY = PIDY(y);
        if (avanceY == true) {
          paso = 2;
        }
        break;
      }
    case 2: {
        avanceX = PIDX(x);
        if (avanceX == true) {
          paso = 3;
        }
        break;
      }

    case 3: {
        avance_terminado = true;
        avanceX = false;
        avanceY = false;
        avanceZ = false;
        paso = 0;
        break;
      }

  }
  return avance_terminado;
}

bool AgarrarPieza(int z)
{
  bool avance_terminado = false;
  switch (paso) {

    case 0: {
        servo.write(0);
        delay(1000);
        paso = 1;
        break;
      }
    case 1: {
        avanceZ = PIDZ(z);
        if (avanceZ == true) {
          paso = 2;
        }
        break;
      }

    case 2: {
        delay(500);
        avanceZ = false;
        CerrarGarra(pwm_cerrar);
        delay(1500);
        paso = 3;
        break;
      }
    case 3: {
        avanceZ = PIDZ(0);
        if (avanceZ == true) {
          avance_terminado = true;
          paso = 0;
        }
        break;
      }

  }
  return avance_terminado;
}


bool SoltarPieza(int z)
{
  bool avance_terminado = false;
  switch (paso) {
    case 0: {
        avanceZ = PIDZ(z);
        if (avanceZ == true) {
          paso = 1;
        }
        break;
      }

    case 1: {
        avanceZ = false;
        delay(1000);
        AbrirGarra();
        delay(1500);
        paso = 2;

        break;

        
      }
    case 2: {        


        avanceZ = PIDZ(0);
        if (avanceZ == true) {

          paso = 0;
          avance_terminado = true;
        }
        break;
      }



  }
  return avance_terminado;
}


bool Secuencia(int x, int y, int z, int ang, int za) {
  bool avance_terminado = false;
  switch (Etapa) {
    case 0: {
        bool avanzarcoordenada = AgarrarPieza(za);
        
        
        if (avanzarcoordenada == true) {
          
          avanceX =  false;
          avanceY = false;
          avanceZ = false;
          Etapa = 1;
          avanzarcoordenada = false;
        }
        break;
      }

    case 1: {
      
        bool avanzarcoordenada = AvanzarCoordenadaXYZ(x, y, 0);
        CerrarGarra(pwm_cerrar*0.65);
        if (avanzarcoordenada == true) {
          avanzarcoordenada = false;
          avanceX =  false;
          avanceY = false;
          avanceZ = false;
          Etapa = 2;
        }
        break;
      }
    case 2: {
        servo.write(ang);
        delay(1000);
        Etapa = 3;
        break;
      }
    case 3:
      {
        bool avanzarcoordenada = SoltarPieza(z);
        if (avanzarcoordenada == true) {
          avanzarcoordenada = false;
          avanceX =  false;
          avanceY = false;
          avanceZ = false;
          Etapa = 4;
        }
        break;
      }

    case 4:
      {
        bool avanzarcoordenada = AvanzarCoordenadaXYZ(0, 0, 0);
        if (avanzarcoordenada == true) {

          avanceX =  false;
          avanceY = false;
          avanceZ = false;
          Etapa = 0;
          avance_terminado = true;
        }
        break;
      }
  }
  return avance_terminado;

}



void PosicionPieza(int X, int Y, int Z, int Ang, int n){
  float deltaX = DistanciaEntrePiezas*cos(Ang*PI/180);
  float deltaY = DistanciaEntrePiezas*sin(Ang*PI/180);
  float deltaZ = AlturadePieza;

  float Orientacion = 0;
  float posX = 0;
  float posY = 0;
  float posZ = 0;
  switch(n) {
    case 1:
    {
      posX = X - deltaX;
      posY = Y + deltaY;
      posZ = Z;
      Orientacion = Ang;
      break;
    }
    case 2:
    {
      posX = X;
      posY = Y;
      posZ = Z;
      Orientacion = Ang;
      break;
    }
    case 3:
    {
      posX = X + deltaX;
      posY = Y - deltaY;
      posZ = Z;
      Orientacion = Ang;
      break;
    }
    case 4:
    {
      posX = X + deltaY;
      posY = Y + deltaX;
      posZ = Z + deltaZ;
      Orientacion = Ang+95;
      break;
    }

    case 5:
    {
      posX = X;
      posY = Y;
      posZ = Z + deltaZ;
      Orientacion = Ang+95;
      break;
    }
    case 6:
    {
      posX = X - deltaY;
      posY = Y - deltaX;
      posZ = Z + deltaZ;
      Orientacion = Ang+95;
      break;
    }
        
  }
  
  PosicionDeseadaX = -posX;
  PosicionDeseadaY = posY;
  PosicionDeseadaZ = posZ;
  PosicionDeseadaO = Orientacion;
  
}

float* GeneracionTrayectoriaL(int posX, int posY, int Orientacion, int Tiempo, int instante) {
  float aX = posX/Tiempo;
  float aY = posX/Tiempo;
  float aO = Orientacion/Tiempo;


  float targetX = aX*instante;
  float targetY = aY*instante;
  float targetO = aO*instante;
  float parametros[3] = {targetX, targetY, targetO};
  return parametros;
}

float* GeneracionTrayectoriaP(int posX, int posY, int Orientacion, int Tiempo) {
  float a0X = pos_X;
  float a2X = 3/Tiempo^2*(posX - pos_X);
  float a3X = 3/Tiempo^3*(posX - pos_X);
  
  float aY = posX/Tiempo;
  float aO = Orientacion/Tiempo;

  float parametros[3] = {a0X, a2X, a3X};
  return parametros;
}
