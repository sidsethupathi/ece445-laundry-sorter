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

Servo claw;
Servo wrist;
Servo elbow;
Servo shoulder;
Servo base;

Pixy pixy;


int start_loop = 0;

int z = 0;
int angle = 0;

void setup()
{    
  
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
  uint16_t blocks;
  blocks = pixy.getBlocks();
  if(blocks) {
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
  } 
}
