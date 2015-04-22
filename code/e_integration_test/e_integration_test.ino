//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

/* 
   06.04.2014 v0.1.3 John Leimon
     + Now using pixy.init() to initialize Pixy in setup().
*/

#include <SPI.h>  
#include "Pixy.h"
#include <LiquidCrystal.h>
#include <Servo.h>

#define LED0 P2_0
#define LED1 P2_1
#define START_BTN P4_5
#define MODE_0 P4_4
#define MODE_1 P3_5
#define IR_OUT P7_3

#define JOINT_DELAY 30
#define JOINT_SKIP 1

Pixy pixy;
LiquidCrystal lcd(P7_7, P7_6, P7_5, P7_4, P5_7, P5_6);

Servo claw;
Servo wrist;
Servo elbow;
Servo shoulder;
Servo base;

int PWM_0 = P1_1;
int PWM_1 = P1_2;
int PWM_2 = P1_3;
int PWM_3 = P1_4;
int PWM_4 = P1_5;
int angle = 5;
int count = 0;

int start_loop = 0;

int z = 0;

void setup()
{

  Serial1.begin(9600);
  Serial1.print("Starting...\n");
  lcd.begin(16, 2);
  // Print a message to the LCD.
  
  lcd.print("ECE445");
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(START_BTN, INPUT);
  pinMode(MODE_0, INPUT);
  pinMode(MODE_1, INPUT);
  pinMode(IR_OUT, INPUT);
  
  //pixy.init();
  
  claw.attach(PWM_4);
  wrist.attach(PWM_3);
  elbow.attach(PWM_2);
  shoulder.attach(PWM_1);
  base.attach(PWM_0);

  
  claw.write(80);
  wrist.write(90);
  elbow.write(90);
  shoulder.write(90);
  base.write(90);
  delay(2000);
}

void loop()
{ 
  z++;
  lcd.setCursor(0, 1);
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  int sensorVal = 0; 

  if(z % 1000 == 0) {
    sensorVal = analogRead(IR_OUT);
    Serial1.println(sensorVal);
  }
  
  blocks = 0;
  //blocks = pixy.getBlocks();
  if(blocks) {
    sprintf(buf, "Detected %d:\n", blocks);
    Serial1.print(buf);
    for (j=0; j<blocks; j++)
    {
      sprintf(buf, "  block %d: ", j);
      Serial1.print(buf); 
      pixy.blocks[j].print();
    }
  }
  
  if(blocks || digitalRead(START_BTN) == HIGH) {
    start_loop = 1;
  }  
  
  if (start_loop)
  {
    if(digitalRead(MODE_1) == HIGH) {
      digitalWrite(LED1, HIGH);
      base.write(80);
  
    } else if(digitalRead(MODE_0) == HIGH) {
      digitalWrite(LED0, HIGH);
      base.write(100);
    }
    count++;
    lcd.print(count);

    
    for(angle = 90; angle< 120; angle+=1)     
    {                                
      elbow.write(angle);            
      delay(100);                    
    }
    for(angle = 120; angle> 90; angle-=1)     
    {                                
      elbow.write(angle);            
      delay(100);                    
    }
    
    digitalWrite(LED0, LOW);
    digitalWrite(LED1, LOW);
    start_loop = 0;

  }
  
}
