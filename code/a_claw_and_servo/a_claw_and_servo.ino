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
#include <Servo.h>

#define JOINT_DELAY 30
#define JOINT_SKIP 1

Servo claw;
Servo wrist;
Servo elbow;
Servo shoulder;
Servo base;

Pixy pixy;


int start_loop = 0;

int z = 0;
int angle = 0;

struct Angles {
  int base;
  int shoulder;
  int elbow;
  int wrist;
};

Angles lutt[9][30] = {
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  },
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  },
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  },
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  },
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  },
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  },
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  },
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  },
{ {0, 59, -68, -80}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}  }
};

Angles lut[2][2] = {
 { {0, 59, -68, -80}, {0, 59, -68, -80} },
 { {0, 59, -68, -80}, {0, 59, -68, -80} }
};





void moveJoint(Servo& joint, int end) {
 int start = joint.read();
  if(start < end) {
   for(int angle = start; angle < end; angle += JOINT_SKIP) {
     joint.write(angle); 
     delay(JOINT_DELAY);
   }
 } else {
   for(int angle = start; angle > end; angle -= JOINT_SKIP) {
     joint.write(angle); 
     delay(JOINT_DELAY);
   }
 }
}

void moveToXY(int x, int y) {
  Serial.println(x);
  Serial.println(y);

  Angles angles = lutt[0][0];
    
  //angles = {0, 59, -68, -80};
   
  
  Serial.println(-1 * angles.wrist);
  
  Serial.println(90 + angles.elbow);
  
  Serial.println(180 - angles.shoulder);
  
  Serial.println(-1 * (angles.base - 90));  

  moveJoint(wrist, -1 * angles.wrist);
  moveJoint(elbow, 90 + angles.elbow);
  moveJoint(shoulder, 180 - angles.shoulder);
  moveJoint(base, -1 * (angles.base - 90));
}

void moveHome() {
  moveJoint(wrist, 90);
  moveJoint(elbow, 90);
  moveJoint(shoulder, 90);
  moveJoint(base, 90);
}

void setup()
{    
  
  Serial.begin(9600);
  Serial.print("Starting...\n");
  
  pixy.init();

  claw.attach(3);
  wrist.attach(10);
  elbow.attach(9);
  shoulder.attach(6);
  base.attach(5);

  
  claw.write(80);
  wrist.write(90);
  elbow.write(90);
  shoulder.write(90);
  base.write(90);
  delay(2000);
  
}

void loop()
{ 
  uint16_t blocks = 1;
  if(blocks) {
    
     
    /*
    moveToXY(15, 15);
    delay(5000);
    moveHome();
    delay(1000);
    */
    
    
    moveJoint(elbow, 120);
    moveJoint(elbow, 90);
    
    moveToXY(15, 15);
    
    
    /*
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
    */
  } 
}

