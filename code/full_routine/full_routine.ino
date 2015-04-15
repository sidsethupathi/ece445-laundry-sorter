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

#define Serial Serial1

#define LED1_ON() digitalWrite(LED1, HIGH);
#define LED1_OFF() digitalWrite(LED1, LOW);
#define LED0_ON() digitalWrite(LED0, HIGH);
#define LED0_OFF() digitalWrite(LED0, LOW);

#define JOINT_DELAY 50
#define JOINT_SKIP 1


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

void moveJoint(Servo& joint, int end) {
 int start = joint.read(); 
 for(int angle = start; angle < end; angle += JOINT_SKIP) {
   joint.write(angle); 
   delay(JOINT_DELAY);
 }
}

void printPixyBlocksToSerial(uint16_t blocks) {
  char buf[32];
  sprintf(buf, "Detected %d:\n", blocks);
  Serial.print(buf);
  for(int j = 0; j < blocks; j++) {
    sprintf(buf, " block %d: ", j);
    Serial.print(buf);
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
 Serial.begin(9600);
 Serial.println("Serial connection started");
 
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
}


/* Control flow

1. Pixy looks for objects. 

*/
void loop() {
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
 int r_x, r_y;
 convertPixyCoordToRobotCoord(p_x, p_y, r_x, r_y);
 openClaw();
 moveRobot(r_x, r_y);
 closeClaw();
 moveToBin(sig);
 openClaw(); 
}

void convertPixyCoordToRobotCoord(int p_x, int p_y, int& r_x, int& r_y) {
   
}

void moveRobot(int r_x, int r_y) {
  
}

void openClaw() {
  
}

void closeClaw() {

}

void moveToBin(int sig) {
  
}


/*
void loop() {
  uint16_t blocks;
  lcd.setCursor(0, 1);

  blocks = pixy.getBlocks();
  
  if(blocks)
    printPixyBlocksToSerial(blocks);
   
  if(blocks || digitalRead(START_BTN) == HIGH) {
    start_loop = 1; 
  }
  
  if(start_loop) {
    if(getMode()) {
      LED1_ON();
      moveJoint(base, 80); 
    } else {
      LED0_ON();
      moveJoint(base, 100); 
    }
    
    lcd.print(loop_count++);

    moveJoint(elbow, 120);
    moveJoint(elbow, 90);
  }
}
*/
