#include <Plantv50.h>
#include <StarshotACS.h>
#include "Arduino.h"
#include <typeinfo>
#include <math.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#include "DataLogging.hpp"

static Plantv50ModelClass plantObj;
static StarshotACSModelClass starshotObj;

static Plantv50ModelClass plantObj2;
static StarshotACSModelClass starshotObj2;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(21, 20);

int iteration = 0;

// ms
int imu_delay = 200;
// s
float time_step = 0.2;
// Step size from simulik is in milliseconds
float plantsim_step_size = 1;
bool detumbling = false;
float alpha_angle = 45;
// kane damper constants
double kane_damper_c = 0.00001;
double kane_Id = 0.196;
// magnetorqueer hardware constants
double ampfactor = 13.5;
double csarea = 4E-5;
double num_loops = 500;
double max_current = 0.25;
// Altitude of orbit in km
float altitude = 400;
// desired angular velocities below
double wdx = 0;
double wdy = 0;
double wdz = 1;

// power budget//

// in percent
#define DUTY_CYCLE 0.25
// in mil seconds
int CYCLE_PERIOD = 9 * 60 * 1000;

///////////////

float degrees_to_radians(float degrees)
{
  return degrees * (M_PI / 180);
}
// Orbital inclination set to ISS orbit
float inclination = degrees_to_radians(51.6F);
//
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
    // plantObj.initialize(0.06, -0.05, 0.07, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude,inclination,csarea,num_loops,ampfactor);
    plantObj.initialize(0.06, -0.05, 0.07, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude, inclination, csarea, num_loops, ampfactor);
  }
  else
  {
    // plantObj.initialize(0.0, 0.0, 1.0, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle),altitude, inclination,csarea,num_loops,ampfactor);
    plantObj.initialize(0.03598, -0.013903, 1.0565, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude, inclination, csarea, num_loops, ampfactor);
    plantObj2.initialize(0.03598, -0.013903, 1.0565, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude, inclination, csarea, num_loops, ampfactor);
  }

  // starshotObj.initialize(kane_damper_c, kane_Id, ampfactor, csarea, num_loops, wdx, wdy, wdz);
  starshotObj.initialize(0.2, kane_damper_c, kane_Id, ampfactor, max_current, csarea, num_loops, wdx, wdy, wdz);
  starshotObj2.initialize(0.2, kane_damper_c, kane_Id, ampfactor, max_current, csarea, num_loops, wdx, wdy, wdz);

  // if (!imu.begin())
  // {
  //   while (1){
  //     Serial.println("wrong");
  //   };
  // }

  DataLogSetup();
  // Serial.println("Found LSM9DS1 9DOF");
  // Serial.println("Setting up imu9DS1 9DOF");
  // imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  // imu.setupMag(imu.LSM9DS1_MAGGAIN_8GAUSS);
  // imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  delay(1000);
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
    if (((iteration * imu_delay) % CYCLE_PERIOD) < DUTY_CYCLE * CYCLE_PERIOD)
    {
      plantObj.rtU.current[0] = starshotObj.rtY.point[0];
      plantObj.rtU.current[1] = starshotObj.rtY.point[1];
      plantObj.rtU.current[2] = starshotObj.rtY.point[2];

      plantObj2.rtU.current[0] = starshotObj2.rtY.point[0];
      plantObj2.rtU.current[1] = starshotObj2.rtY.point[1];
      plantObj2.rtU.current[2] = starshotObj2.rtY.point[2];
    }
    else
    {
      // Serial.print("zeroplt, ");
      plantObj.rtU.current[0] = starshotObj.rtY.point[0];
      plantObj.rtU.current[1] = starshotObj.rtY.point[1];
      plantObj.rtU.current[2] = starshotObj.rtY.point[2];

      plantObj2.rtU.current[0] = 0.0;
      plantObj2.rtU.current[1] = 0.0;
      plantObj2.rtU.current[2] = 0.0;


    }
  }

  //Serial.print(iteration * imu_delay);
  //Serial.print(",");

  if (detumbling)
  {
    Serial.print(plantObj.rtY.angularvelocity[0]);
    Serial.print(",");
    Serial.print(plantObj.rtY.angularvelocity[1]);
    Serial.print(",");
    Serial.print(plantObj.rtY.angularvelocity[2]);
    Serial.print(",");
    Serial.print(starshotObj.rtY.detumble[0]);
    // Serial.print(starshotObj.rtY.detumble[0]);
    Serial.print(",");
    // Serial.print(starshotObj.rtY.detumble[1]);
    Serial.print(starshotObj.rtY.detumble[1]);
    Serial.print(",");
    // Serial.println(starshotObj.rtY.detumble[2]);
    Serial.println(starshotObj.rtY.detumble[2]);
  }
  else
  {
    if (((iteration * imu_delay) % CYCLE_PERIOD) < DUTY_CYCLE * CYCLE_PERIOD)
    {
      //Serial.print(starshotObj.rtY.point[2] * 1000.0);

      double ACSData[4] = {starshotObj.rtY.point[2] * 1000.0, starshotObj.pointing_error, starshotObj2.rtY.point[2] * 1000.0, starshotObj2.pointing_error};
      DataLog(ACSData, 4);
    }
    else
    {
      double ACSData[4] = {starshotObj.rtY.point[2] * 1000.0, starshotObj.pointing_error, 0, starshotObj2.pointing_error};
      DataLog(ACSData, 4);
      // Serial.print(starshotObj.rtY.point[2] * 1000.0);
      // Serial.print(0.0);
    }

    // Serial.print(",");
    // Serial.println(starshotObj.pointing_error);
  }

  for (int i = 0; i < imu_delay / plantsim_step_size; i++)
  {
    plantObj.step(plantsim_step_size / 1000);
    plantObj2.step(plantsim_step_size / 1000);
  }

  iteration++;
  starshotObj.rtU.w[0] = plantObj.rtY.angularvelocity[0];
  starshotObj.rtU.w[1] = plantObj.rtY.angularvelocity[1];
  starshotObj.rtU.w[2] = plantObj.rtY.angularvelocity[2];
  starshotObj.rtU.magneticfield[0] = plantObj.rtY.magneticfield[0];
  starshotObj.rtU.magneticfield[1] = plantObj.rtY.magneticfield[1];
  starshotObj.rtU.magneticfield[2] = plantObj.rtY.magneticfield[2];
  starshotObj.step();

  starshotObj2.rtU.w[0] = plantObj2.rtY.angularvelocity[0];
  starshotObj2.rtU.w[1] = plantObj2.rtY.angularvelocity[1];
  starshotObj2.rtU.w[2] = plantObj2.rtY.angularvelocity[2];
  starshotObj2.rtU.magneticfield[0] = plantObj2.rtY.magneticfield[0];
  starshotObj2.rtU.magneticfield[1] = plantObj2.rtY.magneticfield[1];
  starshotObj2.rtU.magneticfield[2] = plantObj2.rtY.magneticfield[2];
  starshotObj2.step();


}