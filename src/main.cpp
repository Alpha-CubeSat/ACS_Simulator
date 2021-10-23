#include <Plantv50.h>
#include <StarshotACS0.h>
#include "Arduino.h"

static Plantv50ModelClass plantObj;
static StarshotACS0ModelClass starshotObj;
int start_time = 0;
int imu_delay = 400;
int program_start = 0;

void setup(){  
  plantObj.initialize();
  starshotObj.initialize();
  delay(3000); 
  start_time = millis();
  program_start = millis();
}

void loop(){


  starshotObj.step();

  plantObj.rtU.current[0] = starshotObj.rtY.detumble[0];
  plantObj.rtU.current[1] = starshotObj.rtY.detumble[1];
  plantObj.rtU.current[2] = starshotObj.rtY.detumble[2];

  Serial.print(millis() - program_start);
  Serial.print(",");
  Serial.print(starshotObj.rtU.w[0]);
  Serial.print(",");
  Serial.print(starshotObj.rtU.w[1]);
  Serial.print(",");
  Serial.print(starshotObj.rtU.w[2]);
  Serial.print(",");
  Serial.print(plantObj.rtU.current[0]);
  Serial.print(",");
  Serial.print(plantObj.rtU.current[1]);
  Serial.print(",");
  Serial.println(plantObj.rtU.current[2]);


  while(millis()-start_time <= imu_delay){
    plantObj.step();
  }
  start_time = millis();
  
  starshotObj.rtU.w[0] = plantObj.rtY.angularvelocity[0];
  starshotObj.rtU.w[1] = plantObj.rtY.angularvelocity[1];
  starshotObj.rtU.w[2] = plantObj.rtY.angularvelocity[2];
  starshotObj.rtU.Bfield_body[0] = plantObj.rtY.magneticfield[0];
  starshotObj.rtU.Bfield_body[1] = plantObj.rtY.magneticfield[1];
  starshotObj.rtU.Bfield_body[2] = plantObj.rtY.magneticfield[2];
}

