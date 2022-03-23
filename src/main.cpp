#include <Plantv50.h>
#include <StarshotACS.h>
#include "Arduino.h"
#include <typeinfo>
#include <math.h>

static Plantv50ModelClass plantObj;
static StarshotACSModelClass starshotObj;

int iteration = 0;

//test

// ms
int imu_delay = 500;
float plantsim_step_size = 1;

bool detumbling = false;

float alpha_angle = 45;

float degrees_to_radians(float degrees)
{
  return degrees * (M_PI / 180);
}

float get_quat0(float degrees)
{
  float radians = degrees_to_radians(degrees);
  return sin(radians / 2);
}

float get_quat3(float degrees)
{
  float radians = degrees_to_radians(degrees);
  return cos(radians / 2);
}

void setup()
{
  // sqrt(x^2 + y^2 + z^2) < 5 degrees
  if (detumbling)
  {
    plantObj.initialize(0.6, -0.5, 0.7, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle));
  }
  else
  {
    plantObj.initialize(0.0, 0.0, 1.0, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle));
  }

  starshotObj.initialize();
  delay(10000);
}

void loop()
{

  if (detumbling)
  {
    plantObj.rtU.current[0] = starshotObj.rtY.detumble[0];
    plantObj.rtU.current[1] = starshotObj.rtY.detumble[1];
    plantObj.rtU.current[2] = starshotObj.rtY.detumble[2];
  }
  else
  {
    plantObj.rtU.current[0] = starshotObj.rtY.point[0];
    plantObj.rtU.current[1] = starshotObj.rtY.point[1];
    plantObj.rtU.current[2] = starshotObj.rtY.point[2];
  }

  if (iteration % 1 == 0)
  {

    Serial.print(iteration * imu_delay);
    Serial.print(",");

    if (detumbling)
    {
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
    else
    {
      Serial.println(starshotObj.pointing_error);
    }
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

  starshotObj.step();
}
