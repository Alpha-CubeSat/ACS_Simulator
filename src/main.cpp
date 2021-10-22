#include <Plantv50.h>
#include <StarshotACS0.h>
#include "Arduino.h"

static Plantv50ModelClass plantObj;
static StarshotACS0ModelClass starshotObj;

void setup(){  
  plantObj.initialize();
  starshotObj.initialize();
}

void loop(){
  starshotObj.step();

  Serial.println("Angular Velocity p");
  Serial.println(starshotObj.rtU.w[0]);
  Serial.println("Angular Velocity q");
  Serial.println(starshotObj.rtU.w[1]);
  Serial.println("Angular Velocity r");
  Serial.println(starshotObj.rtU.w[2]);

  plantObj.rtU.current[0] = starshotObj.rtY.detumble[0];
  plantObj.rtU.current[1] = starshotObj.rtY.detumble[1];
  plantObj.rtU.current[2] = starshotObj.rtY.detumble[2];

  plantObj.step();

  starshotObj.rtU.w[0] = plantObj.rtY.angularvelocity[0];
  starshotObj.rtU.w[1] = plantObj.rtY.angularvelocity[1];
  starshotObj.rtU.w[2] = plantObj.rtY.angularvelocity[2];
  starshotObj.rtU.Bfield_body[0] = plantObj.rtY.magneticfield[0];
  starshotObj.rtU.Bfield_body[1] = plantObj.rtY.magneticfield[1];
  starshotObj.rtU.Bfield_body[2] = plantObj.rtY.magneticfield[2];
}

