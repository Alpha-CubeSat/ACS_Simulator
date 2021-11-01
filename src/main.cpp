#include <Plantv50.h>
#include <StarshotACS.h>
#include "Arduino.h"

static Plantv50ModelClass plantObj;
static StarshotACSModelClass starshotObj;

int iteration = 0;

//ms
int imu_delay = 500;
float plantsim_step_size = 1;

void setup()
{
  //sqrt(x^2 + y^2 + z^2) < 5 degrees
  plantObj.initialize(0.6, -0.5, 0.7);
  starshotObj.initialize();
  delay(10000);
}

void loop()
{
  starshotObj.step();

  plantObj.rtU.current[0] = starshotObj.rtY.detumble[0];
  plantObj.rtU.current[1] = starshotObj.rtY.detumble[1];
  plantObj.rtU.current[2] = starshotObj.rtY.detumble[2];

  if (iteration % 1 == 0)
  {
    Serial.print(iteration * imu_delay);
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
  }

  for (int i = 0; i < imu_delay / plantsim_step_size; i++)
  {
    plantObj.step(plantsim_step_size / 1000);
  }

  starshotObj.rtU.w[0] = plantObj.rtY.angularvelocity[0];
  starshotObj.rtU.w[1] = plantObj.rtY.angularvelocity[1];
  starshotObj.rtU.w[2] = plantObj.rtY.angularvelocity[2];
  starshotObj.rtU.Bfield_body[0] = plantObj.rtY.magneticfield[0];
  starshotObj.rtU.Bfield_body[1] = plantObj.rtY.magneticfield[1];
  starshotObj.rtU.Bfield_body[2] = plantObj.rtY.magneticfield[2];

  iteration++;
}
