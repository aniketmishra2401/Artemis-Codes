  
#include <Math.h>
#include <Ramp.h>
#include <Adafruit_PWMServoDriver.h>

int SERVOMIN = 150;
int SERVOMAX = 600;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);




#define SERVO1 0
#define SERVO2 1
#define SERVO3 2

#define SERVO4 8
#define SERVO5 9
#define SERVO6 10

#define SERVO7 16
#define SERVO8 17
#define SERVO9 18

#define SERVO10 24
#define SERVO11 25
#define SERVO12 26

#define SERVO13 20
#define SERVO14 21
#define SERVO15 22

#define SERVO16 28
#define SERVO17 29
#define SERVO18 30


const double J2L = 95.0; 
const double J3L = 205.0; 

const double Y_Rest =  52.0;
const double Z_Rest = -112.0;

const double J3_LegAngle = 20;

// Joint Variables

double J11Act = 0.0; double J21Act = 0.0; double J31Act = 0.0; double J41Act = 0.0; double J51Act = 0.0; double J61Act = 0.0;
double J12Act = 0.0; double J22Act = 0.0; double J32Act = 0.0; double J42Act = 0.0; double J52Act = 0.0; double J62Act = 0.0;
double J13Act = 40.0; double J23Act = 40.0; double J33Act = 40.0; double J43Act = 40.0; double J53Act = 40.0; double J63Act = 40.0;

rampDouble J11Tar = 0.0; rampDouble J21Tar = 0.0; rampDouble J31Tar = 0.0; rampDouble J41Tar = 0.0; rampDouble J51Tar = 0.0; rampDouble J61Tar = 0.0;
rampDouble J12Tar = 0.0; rampDouble J22Tar = 0.0; rampDouble J32Tar = 0.0; rampDouble J42Tar = 0.0; rampDouble J52Tar = 0.0; rampDouble J62Tar = 0.0; 
rampDouble J13Tar = 40.0; rampDouble J23Tar = 40.0; rampDouble J33Tar = 40.0; rampDouble J43Tar = 40.0; rampDouble J53Tar = 40.0; rampDouble J63Tar = 40.0;

// Command Variables
bool started = false;
bool ended = false;
uint8_t commandStep = 0;

// Commands
const double lines[32][5] = {
                            {1,150.0, 160.0, 0.0,1}, //22, 0,-20
                            {1,150.0, 1.0, 0.0,400},//28/root2, 28/root2, 
                            {1,150.0, 80.0, 100.0,400},
                            {1,150.0, 160.0, 0.0,400},
                            
                            {2,210.0, 1.0, 0.0, 0},
                            {2,0.0, 210.0, 0.0, 400},
                            {2,105.0, 105.0, 100.0, 400},
                            {2,210.0, 1.0, 0.0, 400},
                            
                            
                            {3,160.0, 150.0, 0.0, 0},
                            {3,0.0, 150.0, 0.0, 400},
                            {3,80.0, 150.0, 100.0, 400},
                            {3,160.0, 150.0, 0.0, 0},

                            {4,150.0, 160.0, 0.0,1}, //22, 0,-20
                            {4,150.0, 1.0, 0.0,400},//28/root2, 28/root2, 
                            {4,150.0, 80.0, 100.0,400},
                            {4,150.0, 160.0, 0.0,400},


                            
                            
                          
                            {5,210.0, 1.0, 0.0, 0},
                            {5,0.0, 210.0, 0.0, 400},
                            {5,105.0, 105.0, 100.0, 400},
                            {5,210.0, 1.0, 0.0, 400},
                            

                            {6,160.0, 150.0, 0.0, 0},
                            {6,0.0, 150.0, 0.0, 400},
                            {6,80.0, 150.0, 100.0, 400},
                            {6,160.0, 150.0, 0.0, 0},
                            


                            
                            };

void moveServo(uint8_t servoNum, int angle) {
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  if (servoNum < 16) {
    pwm1.setPWM(servoNum, 0, pulseLength);  // First PCA9686 (0x40)
  } else {
    pwm2.setPWM(servoNum - 16, 0, pulseLength);  // Second PCA9686 (0x41)
  }
}

void setup() {
  // DEBUG
  
  Serial.begin(9600);

  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);
  // Servo instances



  

  delay(5000);

}

void loop() {
  Serial.println("yo");
  // Update position
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

  CartesianMove(J11Act, J12Act, J13Act, J21Act, J22Act, J23Act,J31Act, J32Act, J33Act, J41Act, J42Act, J43Act, J51Act, J52Act, J53Act,J61Act, J62Act, J63Act);

  if (ended == false){
    // Check if command finished
    if (started == false | J11Tar.isFinished() == true){
      commandStep++;
      if (commandStep > 3)
      {
        ended = true;
        commandStep=0;
      }
      if (commandStep < 3)
      {
        ended = false;
      }

      
      double x1 = lines[commandStep][1];
      double y1 = lines[commandStep][2];
      double z1 = lines[commandStep][3];

      double x2 = lines[commandStep+4][1];
      double y2 = lines[commandStep+4][2];
      double z2 = lines[commandStep+4][3];

      double x3 = lines[commandStep+8][1];
      double y3 = lines[commandStep+8][2];
      double z3 = lines[commandStep+8][3];

      double x4 = lines[commandStep+12][1];
      double y4 = lines[commandStep+12][2];
      double z4 = lines[commandStep+12][3];

      double x5 = lines[commandStep+16][1];
      double y5 = lines[commandStep+16][2];
      double z5 = lines[commandStep+16][3];

      double x6 = lines[commandStep+20][1];
      double y6 = lines[commandStep+20][2];
      double z6 = lines[commandStep+20][3];

      uint16_t duration = lines[commandStep][4] * 2;
      int leg = lines[commandStep][0];
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
      //Serial.println(J11Act);
      

      started = true;
    }
  }
  
}  






void CartesianMove(double XA, double YA, double ZA, double XB, double YB, double ZB, double XC, double YC, double ZC,double XD, double YD, double ZD,double XE, double YE, double ZE,double XF, double YF, double ZF  ){
// OFFSET TO REST POSITION





// CALCULATE INVERSE KINEMATIC SOLUTION
double J1A = atan(YA / XA) * (180 / PI);
double HA = sqrt((YA * YA) + (XA * XA));
double LA = sqrt((ZA * ZA) + ((HA-7) * (HA-7)));
double J3A = acos(   (51050-(LA * LA))   /   (38950)   ) * (180 / PI);
double BA = acos(   ((LA * LA) - 33000)   /   (190 * LA)   ) * (180 / PI);
double AA = atan(ZA / (HA-7)) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
double J2A = (BA - AA);

double J1B = atan(YB / XB) * (180 / PI);
double HB = sqrt((YB * YB) + (XB * XB));
double LB = sqrt((ZB * ZB) + ((HB-7) * (HB-7)));
double J3B = acos(   (51050-(LB * LB))   /   (38950)   ) * (180 / PI);
double BB = acos(   ((LB * LB) - 33000)   /   (190 * LB)   ) * (180 / PI);
double AB = atan(ZB / (HB-7)) * (180 / PI); // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
double J2B = (BB - AB);

double J1C = atan(YC / XC) * (180 / PI);
double HC = sqrt((YC * YC) + (XC * XC));
double LC = sqrt((ZC * ZC) + ((HC-7) * (HC-7)));
double J3C = acos(   (51050-(LC * LC))   /   (38950)   ) * (180 / PI);
double BC = acos(   ((LC * LC) - 33000)   /   (190 * LC)   ) * (180 / PI);
double AC = atan(ZC / (HC-7)) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
double J2C = (BC - AC);

double J1D = atan(YD / XD) * (180 / PI);
double HD = sqrt((YD * YD) + (XD * XD));
double LD = sqrt((HD * HD) + ((ZD-7) * (ZD-7)));
double J3D = acos(   (51050-(LD * LD))   /   (38950)   ) * (180 / PI);
double BD = acos(   ((LD * LD) - 33000)   /   (190 * LD)   ) * (180 / PI);
double AD = atan(HD / (ZD-7)) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
double J2D = (BD - AD);

double J1E = atan(YE / XE) * (180 / PI);
double HE = sqrt((YE * YE) + (XE * XE));
double LE = sqrt((HE * HE) + ((ZE-7) * (ZE-7)));
double J3E = acos(   (51050-(LE * LE))   /   (38950)   ) * (180 / PI);
double BE = acos(   ((LE * LE) - 33000)   /   (190 * LE)   ) * (180 / PI);
double AE = atan(HE / (ZE-7)) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
double J2E = (BE - AE);

double J1F = atan(YF / XF) * (180 / PI);
double HF = sqrt((YF * YF) + (XF * XF));
double LF = sqrt((HF * HF) + ((ZF-7) * (ZF-7)));
double J3F = acos(   (51050-(LF * LF))   /   (38950)   ) * (180 / PI);
double BF = acos(   ((LF * LF) - 33000)   /   (190 * LF)   ) * (180 / PI);
double AF = atan(HF / (ZF-7)) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
double J2F = (BF - AF);

UpdatePosition(J1A, J2A, J3A, J1B, J2B, J3B, J1C, J2C, J3C, J1D, J2D, J3D, J1E, J2E, J3E, J1F, J2F, J3F );


}

void UpdatePosition(double J1A, double J2A, double J3A, double J1B, double J2B, double J3B, double J1C, double J2C, double J3C, double J1D, double J2D, double J3D,double J1E,double J2E,double J3E,double J1F,double J2F,double J3F){
  

  moveServo(SERVO1 ,75+J1A);
  moveServo(SERVO2, 90+J2A);
  moveServo(SERVO3, 180-J3A );

  
  moveServo(SERVO4 ,15+J1B);
  moveServo(SERVO5, 90+J2B);
  moveServo(SERVO6, 180-J3B );
  

  
  moveServo(SERVO7 ,75+J1C);
  moveServo(SERVO8, 90+J2C);
  moveServo(SERVO9, 180-J3C  );
  

  
  moveServo(SERVO10 ,15+J1D);
  moveServo(SERVO11, 90-J2D);
  moveServo(SERVO12, 180-J3D  );


  
  moveServo(SERVO13 ,45+J1E);
  moveServo(SERVO14, 90-J2E);
  moveServo(SERVO15, 180+J3E );
  

  
  moveServo(SERVO16 ,45+J1F);
  moveServo(SERVO17, 90-J2F);
  moveServo(SERVO18, 180+J3F );
  

  delay(40);


}