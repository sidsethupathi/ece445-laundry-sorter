#include <SPI.h>
#include "Pixy.h"
#include <LiquidCrystal.h>
#include <Servo.h>

/* Copy respective include file below */
#define LED0 P2_0
#define LED1 P2_1
#define START_BTN P4_5
#define MODE_0 P4_4
#define MODE_1 P3_5
#define IR_OUT P7_3

#define PWM_0 P1_1
#define PWM_1 P1_2
#define PWM_2 P1_3
#define PWM_3 P1_4
#define PWM_4 P1_5

#define LCD_RS P7_7
#define LCD_EN P7_6
#define LCD_D4 P7_5
#define LCD_D5 P7_4
#define LCD_D6 P5_7
#define LCD_D7 P5_6

#define LED1_ON() digitalWrite(LED1, HIGH);
#define LED1_OFF() digitalWrite(LED1, LOW);
#define LED0_ON() digitalWrite(LED0, HIGH);
#define LED0_OFF() digitalWrite(LED0, LOW);

#define JOINT_DELAY 20
#define JOINT_SKIP 1

struct Angles {
  int base;
  int shoulder;
  int elbow;
  int wrist;
};

Angles lut[23][26] = {
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-59,50,-89,-61}, {-57,46,-84,-68}, {-50,39,-77,-78}, {-50,39,-77,-78}, {-45,46,-89,-70}, {-40,48,-89,-82}, {-34,48,-88,-93}, {-27,45,-89,-95}, {-19,44,-88,-95}, {-10,44,-88,-95}, {0,43,-87,-95}, {9,44,-88,-95}, {18,44,-88,-95}, {26,45,-89,-95}, {33,48,-88,-93}, {39,48,-89,-82}, {45,46,-89,-70}, {49,41,-77,-80}, {53,42,-77,-81}, {56,49,-84,-71}, {59,53,-89,-64} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-59,50,-89,-61}, {-57,46,-84,-68}, {-50,39,-77,-78}, {-50,39,-77,-78}, {-45,46,-89,-70}, {-40,48,-89,-82}, {-34,48,-88,-93}, {-27,45,-89,-95}, {-19,44,-88,-95}, {-10,44,-88,-95}, {0,43,-87,-95}, {9,44,-88,-95}, {18,44,-88,-95}, {26,45,-89,-95}, {33,48,-88,-93}, {39,48,-89,-82}, {45,46,-89,-70}, {49,41,-77,-80}, {53,42,-77,-81}, {56,49,-84,-71}, {59,53,-89,-64} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-48,55,-94,-52}, {-45,50,-88,-58}, {-42,50,-89,-60}, {-38,50,-89,-62}, {-34,47,-86,-66}, {-29,48,-88,-66}, {-24,48,-89,-67}, {-19,45,-85,-71}, {-13,47,-89,-68}, {-7,36,-73,-82}, {0,47,-89,-69}, {6,47,-89,-69}, {12,47,-89,-68}, {18,47,-88,-69}, {24,48,-89,-67}, {29,48,-88,-66}, {33,46,-84,-68}, {38,50,-89,-62}, {41,39,-71,-75}, {45,50,-88,-58}, {48,55,-94,-52} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-48,55,-94,-52}, {-45,50,-88,-58}, {-42,50,-89,-60}, {-38,50,-89,-62}, {-34,47,-86,-66}, {-29,48,-88,-66}, {-24,48,-89,-67}, {-19,45,-85,-71}, {-13,47,-89,-68}, {-7,36,-73,-82}, {0,47,-89,-69}, {6,47,-89,-69}, {12,47,-89,-68}, {18,47,-88,-69}, {24,48,-89,-67}, {29,48,-88,-66}, {33,46,-84,-68}, {38,50,-89,-62}, {41,39,-71,-75}, {45,50,-88,-58}, {48,55,-94,-52} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-45,54,-89,-54}, {-42,55,-94,-52}, {-39,50,-88,-58}, {-35,50,-89,-59}, {-31,50,-89,-61}, {-27,48,-87,-64}, {-22,49,-89,-64}, {-17,49,-89,-65}, {-11,48,-88,-67}, {-6,48,-88,-67}, {0,48,-89,-66}, {6,48,-88,-67}, {11,48,-88,-67}, {17,49,-89,-65}, {22,49,-89,-64}, {26,48,-87,-64}, {31,50,-89,-61}, {35,50,-89,-59}, {39,50,-88,-58}, {42,55,-94,-52}, {45,54,-89,-54} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-42,51,-83,-57}, {-39,54,-88,-54}, {-36,55,-93,-52}, {-33,49,-87,-58}, {-29,50,-89,-58}, {-25,46,-82,-66}, {-20,50,-89,-61}, {-15,50,-89,-62}, {-10,49,-88,-64}, {-5,49,-88,-64}, {0,49,-88,-64}, {5,49,-88,-64}, {10,49,-88,-64}, {15,50,-89,-62}, {20,50,-89,-61}, {24,48,-86,-62}, {29,50,-89,-58}, {32,48,-84,-61}, {36,55,-93,-52}, {39,54,-88,-54}, {42,51,-83,-57} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-40,47,-77,-60}, {-37,50,-82,-57}, {-34,53,-87,-54}, {-30,55,-91,-54}, {-27,50,-88,-56}, {-23,49,-87,-58}, {-18,47,-83,-63}, {-14,50,-88,-60}, {-10,47,-84,-64}, {-5,50,-89,-60}, {0,50,-89,-60}, {5,50,-89,-60}, {9,50,-89,-59}, {14,50,-88,-60}, {18,47,-83,-63}, {23,49,-87,-58}, {27,50,-88,-56}, {30,55,-91,-54}, {34,53,-87,-54}, {37,50,-82,-57}, {40,47,-77,-60} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-38,44,-69,-63}, {-35,47,-75,-60}, {-32,49,-80,-60}, {-28,52,-84,-57}, {-25,54,-88,-54}, {-21,55,-91,-54}, {-17,55,-93,-52}, {-13,51,-89,-56}, {-9,51,-89,-56}, {-4,50,-88,-57}, {0,49,-87,-58}, {4,50,-88,-57}, {9,51,-89,-56}, {13,51,-89,-56}, {17,55,-93,-52}, {21,55,-91,-54}, {25,54,-88,-54}, {28,52,-84,-57}, {32,49,-80,-60}, {35,47,-75,-60}, {38,44,-69,-63} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-36,39,-61,-69}, {-33,42,-67,-66}, {-30,45,-72,-63}, {-27,47,-76,-60}, {-23,49,-80,-60}, {-20,51,-83,-57}, {-16,52,-86,-57}, {-12,54,-88,-54}, {-8,54,-89,-54}, {-4,55,-90,-54}, {0,55,-90,-54}, {4,55,-90,-54}, {8,54,-89,-54}, {12,54,-88,-54}, {16,52,-86,-57}, {20,51,-83,-57}, {23,49,-80,-60}, {27,47,-76,-60}, {30,45,-72,-63}, {33,42,-67,-66}, {36,39,-61,-69} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-34,34,-51,-72}, {-31,37,-58,-69}, {-28,40,-63,-66}, {-25,43,-68,-63}, {-22,45,-72,-63}, {0,0,0,0}, {-15,48,-77,-60}, {-11,49,-79,-60}, {-8,50,-81,-57}, {-4,50,-82,-57}, {0,50,-82,-57}, {4,50,-82,-57}, {8,50,-81,-57}, {11,49,-79,-60}, {15,48,-77,-60}, {0,0,0,0}, {22,45,-72,-63}, {25,43,-68,-63}, {28,40,-63,-66}, {31,37,-58,-69}, {34,34,-51,-72} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-32,28,-39,-77}, {-29,32,-47,-74}, {-27,35,-53,-72}, {-24,38,-58,-69}, {-21,40,-62,-66}, {-17,42,-66,-66}, {0,0,0,0}, {-11,44,-70,-63}, {-7,45,-72,-63}, {-4,45,-73,-63}, {0,46,-73,-63}, {4,45,-73,-63}, {7,45,-72,-63}, {11,44,-70,-63}, {0,0,0,0}, {17,42,-66,-66}, {21,40,-62,-66}, {24,38,-58,-69}, {27,35,-53,-72}, {29,32,-47,-74}, {32,28,-39,-77} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-30,18,-20,-89}, {-28,24,-32,-80}, {-25,28,-40,-77}, {-22,31,-46,-74}, {-19,34,-51,-72}, {-16,36,-55,-72}, {-13,38,-58,-69}, {-10,39,-60,-69}, {-7,40,-62,-69}, {-3,40,-63,-66}, {0,40,-63,-66}, {3,40,-63,-66}, {7,40,-62,-69}, {10,39,-60,-69}, {13,38,-58,-69}, {16,36,-55,-72}, {19,34,-51,-72}, {22,31,-46,-74}, {25,28,-40,-77}, {28,24,-32,-80}, {30,18,-20,-89} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-29,45,-87,-25}, {-27,49,-89,-28}, {-24,19,-21,-89}, {-21,23,-30,-83}, {-18,27,-37,-80}, {-16,29,-42,-77}, {-13,31,-45,-74}, {-9,32,-48,-74}, {-6,33,-50,-72}, {-3,34,-51,-72}, {0,34,-51,-72}, {3,34,-51,-72}, {6,33,-50,-72}, {9,32,-48,-74}, {13,31,-45,-74}, {16,29,-42,-77}, {18,27,-37,-80}, {21,23,-30,-83}, {24,19,-21,-89}, {26,49,-89,-28}, {29,45,-87,-25} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {-28,44,-89,-17}, {-25,39,-71,-40}, {-23,45,-88,-23}, {-20,46,-89,-24}, {-18,14,-11,-92}, {-15,19,-22,-86}, {-12,22,-28,-83}, {-9,24,-32,-83}, {-6,26,-35,-80}, {-3,26,-36,-80}, {0,27,-36,-80}, {3,26,-36,-80}, {6,26,-35,-80}, {9,24,-32,-83}, {12,22,-28,-83}, {15,19,-22,-86}, {18,14,-11,-92}, {20,46,-89,-24}, {23,45,-88,-23}, {25,39,-71,-40}, {28,44,-89,-17} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} },
{ {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} }
};

/* End include file */

/* Pixy define */
Pixy pixy;

/* LCD define */
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

/* Robot joint defines */
Servo claw;
Servo wrist;
Servo elbow;
Servo shoulder;
Servo base;

/* Globals */
int cnt = 0;
int start_loop = 0;
int loop_count = 0;
int r_y;
int r_x;
int ppc_x = 8;
int ppc_y = 8;

int pixy_x_start = 0;
int pixy_x_end = 0;
int pixy_y_start = 0;
int pixy_y_end = 0;

int robot_y_start = 12;
int robot_y_end = 19;
int robot_x_start = 9; // old 9, 13
int robot_x_end = 13;

void moveJoint(Servo& joint, int end) {
 int start = joint.read();
  if(start < end) {
   for(int angle = start; angle <= end; angle += JOINT_SKIP) {
     joint.write(angle); 
     delay(JOINT_DELAY);
   }
 } else {
   for(int angle = start; angle >= end; angle -= JOINT_SKIP) {
     joint.write(angle); 
     delay(JOINT_DELAY);
   }
 }
}

void printPixyBlocksToSerial(uint16_t blocks) {
  char buf[32];
  sprintf(buf, "Detected %d:\n", blocks);
  Serial1.print(buf);
  for(int j = 0; j < blocks; j++) {
    sprintf(buf, " block %d: ", j);
    Serial1.print(buf);
    pixy.blocks[j].print(); 
  }
}

int getMode() {
   if(digitalRead(MODE_1) == HIGH)
     return 1;
   else if(digitalRead(MODE_0) == HIGH)
     return 0;
   else
     return -1;
}

void setup()
{
 /* Set output pinmodes */
 pinMode(LED0, OUTPUT);
 pinMode(LED1, OUTPUT);
 
 /* Set input pinmodes */
 pinMode(START_BTN, INPUT);
 pinMode(MODE_0, INPUT);
 pinMode(MODE_1, INPUT);
 pinMode(IR_OUT, INPUT);
 
 /* Begin Serial connection */
 Serial1.begin(9600);
 Serial1.println("Serial connection started");
 
 /* Print to LCD screen */
 lcd.begin(16, 2); 
 lcd.print("ECE 445");
 
 /* Init pixy */
 pixy.init();
 
 /* Attach robot joints to corresponding PWMs */
claw.attach(PWM_4);
wrist.attach(PWM_3);
elbow.attach(PWM_2);
shoulder.attach(PWM_1);
base.attach(PWM_0);

moveJoint(claw, 80);
moveJoint(wrist, 180);
moveJoint(elbow, 90);
moveJoint(shoulder, 90);
moveJoint(base, 90);
 
delay(1000);
}


/* Control flow

1. Pixy looks for objects. 

*/
int incomingByte = 0;   // for incoming serial data
int count = 0;

int to_x = 0;
int to_y = 0;
int started = 0;

void loop() {
  /*
  for(int j = 15; j <= 22; j++) {
    moveHome();
    moveRobot(6, j);
  }  
  
  /*
  moveHome();
  openClaw();
  moveRobot(6, 14);
  closeClaw();
  moveHome();
  openClaw();
  moveRobot(6, 12);
  closeClaw();
  moveHome();
  openClaw();
  moveRobot(6, 10);
  */
 
  
  
  while(digitalRead(START_BTN) == LOW && started == 0) {
   calibrate_routine(); 
  }
  started = 1;
  
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  blocks = pixy.getBlocks();
  
  for (j=0; j<blocks; j++)
      {         
        convertPixyCoordToRobotCoord(pixy.blocks[j].x, pixy.blocks[j].y);
        if(r_x >= 6 && r_x <= 22 && r_y >= 8 && r_y <= 23 && pixy.blocks[j].signature != 7) {
          if(r_x > 19)
            r_x = 19;
          
          if(r_x > 13)
            r_x++;
          if(r_x < 9)
            r_x--;
          
          sprintf(buf, "  block %d: ", j);
          Serial1.print(buf); 
          sprintf(buf, "Moving to (x,y): (%d, %d)\n", r_x, r_y);
          Serial1.print(buf); 

          printDetection(pixy.blocks[j].signature);
          printCoordinates(r_x, r_y);
          
          
          moveRobot(r_x, r_y);
          closeClaw();
          moveHome();
          moveToBin(pixy.blocks[j].signature);
          openClaw();
          moveHome();
          
          
          delay(1000);
          


        } 
        
      }
 
      
}

int calibrated = 0;
void calibrate_routine() {
  static int i = 0;
  int calibrate_count = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  blocks = pixy.getBlocks();
  
  if (blocks)
  {
    i++;
    calibrate_count = 0;
    
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      for (j=0; j<blocks; j++)
      {
        if(pixy.blocks[j].signature == 7) {
         calibrate_count++; 
        }
      }
      
      if(calibrate_count == 2) {
        Serial1.print("Calibrating!\n");
        digitalWrite(P2_0, HIGH);
        calibratePixy(blocks);
        calibrated = 1;
        delay(2000);
        digitalWrite(P2_0, LOW);
      }
    }
  }
}

void printDetection(int sig) {  
  lcd.begin(16, 2); 
  if(sig == 1) {
    lcd.print("white"); 
   } else if(sig == 2) {
    lcd.print("red"); 
  } else if(sig == 3) {
    lcd.print("blue"); 
  } else if(sig == 4) {
    lcd.print("green"); 
  } else {
    lcd.print("unknown"); 
  }  
}

void printCoordinates(int x, int y) {
 char buf[4];
 sprintf(buf, "%d", x);
 lcd.setCursor(0, 1);
 lcd.print(buf);
 
 sprintf(buf, "%d", y);
 lcd.setCursor(4, 1);
 lcd.print(buf);
}
void test_routine() {
  //moveHome
}

void full_routine() {
  uint16_t blocks;
  lcd.setCursor(0, 1);
  
  blocks = pixy.getBlocks(); // number of blocks the pixy has seen
  if(blocks) {
    for(int i = 0; i < blocks; i++) {
     sort(pixy.blocks[i].signature, pixy.blocks[i].x, pixy.blocks[i].y); 
    }
  }
}

void sort(int sig, int p_x, int p_y) {
 convertPixyCoordToRobotCoord(p_x, p_y);
 openClaw();
 moveRobot(r_x, r_y);
 closeClaw();
 moveToBin(sig);
 openClaw(); 
}

void convertPixyCoordToRobotCoord(int p_x, int p_y) {  
  r_y = ((p_x - pixy_x_start) / ppc_y) + robot_y_start;
  r_x = (((200 - p_y) - pixy_y_start) / ppc_x) + robot_x_start; // need to flip since r_x and p_y oppose each other
}

void moveRobot(int r_x, int r_y) {
  if(r_x == 8)
    r_x = 9;
  
  if(r_x == 7 || r_x == 5)
    r_x = 6;
    
  Serial1.println(r_x);
  Serial1.println(r_y);

  Angles angles = lut[r_x][r_y];   
  
  Serial1.println(angles.wrist);
  Serial1.println(angles.elbow);
  Serial1.println(angles.shoulder);
  Serial1.println(angles.base);  
  
  int c_wrist = 180 + angles.wrist;
  int c_elbow = 90 + angles.elbow;
  int c_base = -1 * (angles.base - 90);
  int c_shoulder = 180 - angles.shoulder;
  
  moveJoint(wrist, c_wrist);
  moveJoint(elbow, c_elbow);
  moveJoint(base, c_base);
  moveJoint(shoulder, c_shoulder);
  
  delay(500);
  
}

void moveHome() {
  moveJoint(shoulder, 90);
  moveJoint(elbow, 90);
  moveJoint(wrist, 180);
  moveJoint(base, 90);
  delay(500);
}

void openClaw() {
  moveJoint(claw, 80);
  delay(1000);
}

void closeClaw() {
  moveJoint(claw, 145);
  delay(1000);
}

void moveToBin(int sig) {
  
  if(sig == 1) {
    moveJoint(wrist, 180);
    moveJoint(base, 40);
    moveJoint(shoulder, 40);
    moveJoint(elbow, 160);
  } else if(sig == 2) {
    moveJoint(wrist, 150);
    moveJoint(base, 150);
    moveJoint(shoulder, 140);
    moveJoint(elbow, 70);
  } else if (sig == 3) {
    moveJoint(wrist, 180);
    moveJoint(base, 90);
    moveJoint(shoulder, 170);
    moveJoint(elbow, 90);
  }
  
  
}

void calibratePixy(int blocks) {  
  int tmps[4];
  int c = 0;
  int j = 0;
  
  for(j = 0; j < blocks; j++) {
    if(pixy.blocks[j].signature == 7) {
      if(c < 3) {
         tmps[c] = pixy.blocks[j].x;
         c++;
         tmps[c] = pixy.blocks[j].y;
         c++;
      }
    }
  }
  
  pixy_x_start = min(tmps[0], tmps[2]);
  pixy_x_end = max(tmps[0], tmps[2]);
  
  pixy_y_start = min(tmps[1], tmps[3]);
  pixy_y_end = max(tmps[1], tmps[3]);
  
  ppc_x = (pixy_y_end - pixy_y_start) / (robot_x_end - robot_x_start);
  ppc_y = (pixy_x_end - pixy_x_start) / (robot_y_end - robot_y_start);
  
  ppc_x = ppc_y / 1;
  
  char buf[32]; 
  sprintf(buf, "ppc_x: %d, ppc_y: %d\n", ppc_x, ppc_y);
  Serial1.print(buf);
}
