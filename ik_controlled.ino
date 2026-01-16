#include <Math.h>
#include <Ramp.h>
#include <Adafruit_PWMServoDriver.h>


int SERVOMIN = 150;
int SERVOMAX = 600;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

#define SERVO1 4
#define SERVO2 5
#define SERVO3 6

#define SERVO4 0
#define SERVO5 1
#define SERVO6 2

#define SERVO7 25
#define SERVO8 29
#define SERVO9 27

#define SERVO10 23
#define SERVO11 21
#define SERVO12 22

#define SERVO13 16
#define SERVO14 17
#define SERVO15 18

#define SERVO16 12
#define SERVO17 13
#define SERVO18 14

const double J1L = 60.0;
const double J2L = 84.0;      //95.0; 
const double J3L = 185.0;     //205.0; 
const double Y_Rest =  52.0; 
const double Z_Rest = -112.0;
const double J3_LegAngle = 20;

// Joint Variables
double J11Act = 100.0; double J21Act = 100.0; double J31Act = 100.0; double J41Act = 100.0; double J51Act = 100.0; double J61Act = 100.0;
double J12Act = 100.0; double J22Act = 100.0; double J32Act = 100.0; double J42Act = 100.0; double J52Act = 100.0; double J62Act = 100.0;
double J13Act = 150.0; double J23Act = 150.0; double J33Act = 150.0; double J43Act = 150.0; double J53Act = 150.0; double J63Act = 150.0;

rampDouble J11Tar = 100.0; rampDouble J21Tar = 100.0; rampDouble J31Tar = 100.0; rampDouble J41Tar = 100.0; rampDouble J51Tar = 100.0; rampDouble J61Tar = 100.0;
rampDouble J12Tar = 100.0; rampDouble J22Tar = 100.0; rampDouble J32Tar = 100.0; rampDouble J42Tar = 100.0; rampDouble J52Tar = 100.0; rampDouble J62Tar = 100.0; 
rampDouble J13Tar = 150.0; rampDouble J23Tar = 150.0; rampDouble J33Tar = 150.0; rampDouble J43Tar = 150.0; rampDouble J53Tar = 150.0; rampDouble J63Tar = 150.0;

// Command Variables
bool started = false;
bool ended = false;
uint8_t commandStep = 0;
int commandStepB = 4;

// NEW: Current movement mode
char currentMode = 'S';  // S = Stop, F = Forward, B = Back, L = Left, R = Right


// Commands
const double lines[32][5] = {
  {1, 80.0, 80.0, 150.0, 0}, //leg t1L1 (1)
  {1, 80.0, 55.0, 150.0, 300},
  {1, 80.0, 30.0, 150.0, 300},
  {1, 80.0, 55.0, 100.0, 300},
  {1, 80.0, 80.0, 150.0, 300},
  
  {1, 95.8, 59.8,  150.0, 0},  //(2)
  {1, 77.8, 77.8, 100.0, 300},
  {1, 59.8, 95.8, 150.0, 300}, 
  {1, 77.8, 77.8, 150.0, 300},
  {1, 95.8, 59.8,  150.0, 300},                          
  
  {1, 30.0, 80.0, 150.0, 0},//leg t1L1 (3)
  {1, 55.0, 80.0, 150.0, 300},
  {1, 80.0, 80.0, 150.0, 300}, 
  {1, 55.0, 80.0, 100.0, 300},
  {1, 30.0, 80.0, 150.0, 300},
  
  {1, 80.0, 80.0, 150.0, 0}, //leg t1L1 (4)
  {1, 80.0, 55.0, 100.0, 300},
  {1, 80.0, 30.0, 150.0, 300},
  {1, 80.0, 55.0, 150.0, 300},
  {1, 80.0, 80.0, 150.0, 300},
  
  {1, 95.8, 59.8, 150.0, 0}, //leg t1L3 (5)
  {1, 77.8, 77.8, 150.0, 300},
  {1, 59.8, 95.8, 150.0, 300}, 
  {1, 77.8, 77.8, 100.0, 300}, 
  {1, 95.8, 59.8, 150.0, 300},                         
  
  {1, 30.0, 80.0, 150.0, 0},//leg t1L1 (6)
  {1, 55.0, 80.0, 100.0, 300},
  {1, 80.0, 80.0, 150.0, 300},
  {1, 55.0, 80.0, 150.0, 300},
  {1, 30.0, 80.0, 150.0, 300},
};



void moveServo(uint8_t servoNum, int angle) {
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  if (servoNum < 16) {
    pwm1.setPWM(servoNum, 0, pulseLength);
  } else {
    pwm2.setPWM(servoNum - 16, 0, pulseLength);
  }
}

void moveForward(){
  // Check if command finished
  if (started == false || J11Tar.isFinished() == true){
    commandStep++;
    if (commandStep > 4) {
      commandStep = 0;  // Loop back to start
    }

    double x1 = lines[commandStep][1];
    double y1 = lines[commandStep][2];
    double z1 = lines[commandStep][3];

    double x2 = lines[commandStep+5][1];
    double y2 = lines[commandStep+5][2];
    double z2 = lines[commandStep+5][3];

    double x3 = lines[commandStep+10][1];
    double y3 = lines[commandStep+10][2];
    double z3 = lines[commandStep+10][3];

    double x4 = lines[commandStep+15][1];
    double y4 = lines[commandStep+15][2];
    double z4 = lines[commandStep+15][3];

    double x5 = lines[commandStep+20][1];
    double y5 = lines[commandStep+20][2];
    double z5 = lines[commandStep+20][3];

    double x6 = lines[commandStep+25][1];
    double y6 = lines[commandStep+25][2];
    double z6 = lines[commandStep+25][3];

    uint16_t duration = lines[commandStep][4] * 2;
    
    J11Tar.go(x1, duration);
    J12Tar.go(y1, duration);
    J13Tar.go(z1, duration);

    J21Tar.go(x2, duration);
    J22Tar.go(y2, duration);
    J23Tar.go(z2, duration);

    J31Tar.go(x3, duration);
    J32Tar.go(y3, duration);
    J33Tar.go(z3, duration);

    J41Tar.go(x4, duration);
    J42Tar.go(y4, duration);
    J43Tar.go(z4, duration);

    J51Tar.go(x5, duration);
    J52Tar.go(y5, duration);
    J53Tar.go(z5, duration);

    J61Tar.go(x6, duration);
    J62Tar.go(y6, duration);
    J63Tar.go(z6, duration);

    started = true;
  }
}

void moveBack(){
  if (started == false || J11Tar.isFinished() == true){
    commandStep++;
    if (commandStep > 4) {
      commandStep = 0;  // Loop back to end
    }

    double x1 = lines[commandStep+15][1];
    double y1 = lines[commandStep+15][2];
    double z1 = lines[commandStep+15][3];

    double x2 = lines[commandStep+20][1];
    double y2 = lines[commandStep+20][2];
    double z2 = lines[commandStep+20][3];

    double x3 = lines[commandStep+25][1];
    double y3 = lines[commandStep+25][2];
    double z3 = lines[commandStep+25][3];

    double x4 = lines[commandStep][1];
    double y4 = lines[commandStep][2];
    double z4 = lines[commandStep][3];

    double x5 = lines[commandStep+5][1];
    double y5 = lines[commandStep+5][2];
    double z5 = lines[commandStep+5][3];

    double x6 = lines[commandStep+10][1];
    double y6 = lines[commandStep+10][2];
    double z6 = lines[commandStep+10][3];

    uint16_t duration = lines[commandStep][4] * 2;
    
    J11Tar.go(x1, duration);
    J12Tar.go(y1, duration);
    J13Tar.go(z1, duration);

    J21Tar.go(x2, duration);
    J22Tar.go(y2, duration);
    J23Tar.go(z2, duration);

    J31Tar.go(x3, duration);
    J32Tar.go(y3, duration);
    J33Tar.go(z3, duration);

    J41Tar.go(x4, duration);
    J42Tar.go(y4, duration);
    J43Tar.go(z4, duration);

    J51Tar.go(x5, duration);
    J52Tar.go(y5, duration);
    J53Tar.go(z5, duration);

    J61Tar.go(x6, duration);
    J62Tar.go(y6, duration);
    J63Tar.go(z6, duration);

    started = true;
  }
}

void moveLeft(){
  if (started == false || J11Tar.isFinished() == true){
    commandStep++;
    if (commandStep > 4) {
      commandStep = 0;
    }

    double x1 = lines[commandStep][1];
    double y1 = lines[commandStep][2];
    double z1 = lines[commandStep][3];

    double x2 = lines[commandStep+5][1];
    double y2 = lines[commandStep+5][2];
    double z2 = lines[commandStep+5][3];

    double x3 = lines[commandStep+10][1];
    double y3 = lines[commandStep+10][2];
    double z3 = lines[commandStep+10][3];

    double x4 = lines[commandStep+5][1];
    double y4 = lines[commandStep+5][2];
    double z4 = lines[commandStep+5][3];

    double x5 = lines[commandStep+10][1];
    double y5 = lines[commandStep+10][2];
    double z5 = lines[commandStep+10][3];

    double x6 = lines[commandStep+5][1];
    double y6 = lines[commandStep+5][2];
    double z6 = lines[commandStep+5][3];

    uint16_t duration = lines[commandStep][4] * 2;
    
    J11Tar.go(x1, duration);
    J12Tar.go(y1, duration);
    J13Tar.go(z1, duration);

    J21Tar.go(x2, duration);
    J22Tar.go(y2, duration);
    J23Tar.go(z2, duration);

    J31Tar.go(x3, duration);
    J32Tar.go(y3, duration);
    J33Tar.go(z3, duration);

    J41Tar.go(x4, duration);
    J42Tar.go(y4, duration);
    J43Tar.go(z4, duration);

    J51Tar.go(x5, duration);
    J52Tar.go(y5, duration);
    J53Tar.go(z5, duration);

    J61Tar.go(x6, duration);
    J62Tar.go(y6, duration);
    J63Tar.go(z6, duration);

    started = true;
  }
}

void moveRight(){
  if (started == false || J11Tar.isFinished() == true){
    commandStep++;
    if (commandStep > 4) {
      commandStep = 0;
    }

    double x1 = lines[commandStep+15][1];
    double y1 = lines[commandStep+15][2];
    double z1 = lines[commandStep+15][3];

    double x2 = lines[commandStep+20][1];
    double y2 = lines[commandStep+20][2];
    double z2 = lines[commandStep+20][3];

    double x3 = lines[commandStep+25][1];
    double y3 = lines[commandStep+25][2];
    double z3 = lines[commandStep+25][3];

    double x4 = lines[commandStep+20][1];
    double y4 = lines[commandStep+20][2];
    double z4 = lines[commandStep+20][3];

    double x5 = lines[commandStep+25][1];
    double y5 = lines[commandStep+25][2];
    double z5 = lines[commandStep+25][3];

    double x6 = lines[commandStep+20][1];
    double y6 = lines[commandStep+20][2];
    double z6 = lines[commandStep+20][3];

    uint16_t duration = lines[commandStep][4] * 2;
    
    J11Tar.go(x1, duration);
    J12Tar.go(y1, duration);
    J13Tar.go(z1, duration);

    J21Tar.go(x2, duration);
    J22Tar.go(y2, duration);
    J23Tar.go(z2, duration);

    J31Tar.go(x3, duration);
    J32Tar.go(y3, duration);
    J33Tar.go(z3, duration);

    J41Tar.go(x4, duration);
    J42Tar.go(y4, duration);
    J43Tar.go(z4, duration);

    J51Tar.go(x5, duration);
    J52Tar.go(y5, duration);
    J53Tar.go(z5, duration);

    J61Tar.go(x6, duration);
    J62Tar.go(y6, duration);
    J63Tar.go(z6, duration);

    started = true;
  }
}

void Stop(){
  // Robot stops moving, holds current position
    double x1 = 80;
    double y1 = 80;
    double z1 = 150;

    double x2 = 95.8;
    double y2 = 59.8;
    double z2 = 150;

    double x3 = 30;
    double y3 = 80;
    double z3 = 150;

    double x4 = 80;
    double y4 = 80;
    double z4 = 150;

    double x5 = 95.8;
    double y5 = 59.8;
    double z5 = 150;

    double x6 = 30;
    double y6 = 80;
    double z6 = 150;

    uint16_t duration = lines[commandStep][4] * 2;
    
    J11Tar.go(x1, duration);
    J12Tar.go(y1, duration);
    J13Tar.go(z1, duration);

    J21Tar.go(x2, duration);
    J22Tar.go(y2, duration);
    J23Tar.go(z2, duration);

    J31Tar.go(x3, duration);
    J32Tar.go(y3, duration);
    J33Tar.go(z3, duration);

    J41Tar.go(x4, duration);
    J42Tar.go(y4, duration);
    J43Tar.go(z4, duration);

    J51Tar.go(x5, duration);
    J52Tar.go(y5, duration);
    J53Tar.go(z5, duration);

    J61Tar.go(x6, duration);
    J62Tar.go(y6, duration);
    J63Tar.go(z6, duration);

}

void setup() {
  Serial.begin(9600);

 

  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  delay(5000);

}

void loop() {
  // Update all joint positions
  J11Act = J11Tar.update();
  J12Act = J12Tar.update();
  J13Act = J13Tar.update();

  J21Act = J21Tar.update();
  J22Act = J22Tar.update();
  J23Act = J23Tar.update();

  J31Act = J31Tar.update();
  J32Act = J32Tar.update();
  J33Act = J33Tar.update();

  J41Act = J41Tar.update();
  J42Act = J42Tar.update();
  J43Act = J43Tar.update();

  J51Act = J51Tar.update();
  J52Act = J52Tar.update();
  J53Act = J53Tar.update();

  J61Act = J61Tar.update();
  J62Act = J62Tar.update();
  J63Act = J63Tar.update();

  CartesianMove(J11Act, J12Act, J13Act, J21Act, J22Act, J23Act, J31Act, J32Act, J33Act, 
                J41Act, J42Act, J43Act, J51Act, J52Act, J53Act, J61Act, J62Act, J63Act);

  // Check for new serial commands
  if (Serial.available()) { 
    String cmd = Serial.readStringUntil('\n'); 
    cmd.trim(); 
    cmd.toUpperCase(); 
    
    if (cmd.length() > 0) {
      currentMode = cmd.charAt(0);  // Get first character
      Serial.print("Mode changed to: ");
      Serial.println(currentMode);
      
      // Reset state for new command
      started = false;
      if (currentMode == 'B') {
        commandStepB = 4;
      } else {
        commandStep = 0;
      }
    }
  }

  // Execute current movement mode continuously
  switch(currentMode) {
    case 'F':
      moveForward();
      break;
    case 'B':
      moveBack();
      break;
    case 'L':
      moveLeft();
      break;
    case 'R':

      moveRight();
      break;
    case 'S':
      Stop();
      break;
  }
}

void CartesianMove(double XA, double YA, double ZA, double XB, double YB, double ZB, double XC, double YC, double ZC,
                   double XD, double YD, double ZD, double XE, double YE, double ZE, double XF, double YF, double ZF){
  // CALCULATE INVERSE KINEMATIC SOLUTION
  double J1A = atan((YA) / (XA)) * (180 / PI);
  double HA = sqrt(((YA) * (YA)) + ((XA) * (XA)));
  double LA = sqrt((ZA * ZA) + ((HA-J1L) * (HA-J1L)));
  double J3A = acos(((J3L*J3L)+(J2L*J2L)-(LA * LA)) / (2*J2L*J3L)) * (180 / PI);
  double BA = acos(((LA * LA) + (J2L*J2L) - (J3L*J3L)) / (2*J2L * LA)) * (180 / PI);
  double AA = atan(ZA / (HA-J1L)) * (180 / PI);
  double J2A = (BA - AA);

  double J1B = atan((YB) / (XB)) * (180 / PI);
  double HB = sqrt(((YB) * (YB)) + ((XB) * (XB)));
  double LB = sqrt((ZB * ZB) + ((HB-J1L) * (HB-J1L)));
  double J3B = acos(((J3L*J3L)+(J2L*J2L)-(LB * LB)) / (2*J2L*J3L)) * (180 / PI);
  double BB = acos(((LB * LB) + (J2L*J2L) - (J3L*J3L)) / (2*J2L * LB)) * (180 / PI);
  double AB = atan(ZB / (HB-J1L)) * (180 / PI);
  double J2B = (BB - AB);
  
  double J1C = atan((YC) / (XC)) * (180 / PI);
  double HC = sqrt(((YC) * (YC)) + ((XC) * (XC)));
  double LC = sqrt((ZC * ZC) + ((HC-J1L) * (HC-J1L)));
  double J3C = acos(((J3L*J3L)+(J2L*J2L)-(LC * LC)) / (2*J2L*J3L)) * (180 / PI);
  double BC = acos(((LC * LC) + (J2L*J2L) - (J3L*J3L)) / (2*J2L * LC)) * (180 / PI);
  double AC = atan(ZC / (HC-J1L)) * (180 / PI);
  double J2C = (BC - AC);

  double J1D = atan((YD) / (XD)) * (180 / PI);
  double HD = sqrt(((YD) * (YD)) + ((XD) * (XD)));
  double LD = sqrt((ZD * ZD) + ((HD-J1L) * (HD-J1L)));
  double J3D = acos(((J3L*J3L)+(J2L*J2L)-(LD * LD)) / (2*J2L*J3L)) * (180 / PI);
  double BD = acos(((LD * LD) + (J2L*J2L) - (J3L*J3L)) / (2*J2L * LD)) * (180 / PI);
  double AD = atan(ZD / (HD-J1L)) * (180 / PI);
  double J2D = (BD - AD);

  double J1E = atan((YE) / (XE)) * (180 / PI);
  double HE = sqrt(((YE) * (YE)) + ((XE) * (XE)));
  double LE = sqrt((ZE * ZE) + ((HE-J1L) * (HE-J1L)));
  double J3E = acos(((J3L*J3L)+(J2L*J2L)-(LE * LE)) / (2*J2L*J3L)) * (180 / PI);
  double BE = acos(((LE * LE) + (J2L*J2L) - (J3L*J3L)) / (2*J2L * LE)) * (180 / PI);
  double AE = atan(ZE / (HE-J1L)) * (180 / PI);
  double J2E = (BE - AE);

  double J1F = atan((YF) / (XF)) * (180 / PI);
  double HF = sqrt(((YF) * (YF)) + ((XF) * (XF)));
  double LF = sqrt((ZF * ZF) + ((HF-J1L) * (HF-J1L)));
  double J3F = acos(((J3L*J3L)+(J2L*J2L)-(LF * LF)) / (2*J2L*J3L)) * (180 / PI);
  double BF = acos(((LF * LF) + (J2L*J2L) - (J3L*J3L)) / (2*J2L * LF)) * (180 / PI);
  double AF = atan(ZF / (HF-J1L)) * (180 / PI);
  double J2F = (BF - AF);



  UpdatePosition(J1A, J2A, J3A, J1B, J2B, J3B, J1C, J2C, J3C, J1D, J2D, J3D, J1E, J2E, J3E, J1F, J2F, J3F);
}

void UpdatePosition(double J1A, double J2A, double J3A, double J1B, double J2B, double J3B, double J1C, double J2C, double J3C, 
                    double J1D, double J2D, double J3D, double J1E, double J2E, double J3E, double J1F, double J2F, double J3F){
  
  moveServo(SERVO1, J1A);
  moveServo(SERVO2, 90-J2A);
  moveServo(SERVO3, J3A);

  moveServo(SERVO4, J1B);
  moveServo(SERVO5, 90-J2B);
  moveServo(SERVO6, J3B);

  moveServo(SERVO7, J1C);
  moveServo(SERVO8, 90-J2C);
  moveServo(SERVO9, J3C);

  moveServo(SERVO10, J1D);
  moveServo(SERVO11, 90-J2D);
  moveServo(SERVO12, J3D);

  moveServo(SERVO13, J1E);
  moveServo(SERVO14, 90-J2E);
  moveServo(SERVO15, J3E);

  moveServo(SERVO16, J1F);
  moveServo(SERVO17, 90-J2F);
  moveServo(SERVO18, J3F);

  delay(40);
}