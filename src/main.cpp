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
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(21,20);



int iteration = 0;

// ms
int imu_delay = 200;
//s
float time_step = 0.2;
//Step size from simulik is in milliseconds
float plantsim_step_size = 1;
bool detumbling = false;
float alpha_angle = 45;
//kane damper constants
double kane_damper_c = 0.00001;
double kane_Id = 0.196;
//magnetorqueer hardware constants
double ampfactor = 13.5;
double csarea = 4E-5;
double num_loops = 500;
double max_current = 0.25;
//Altitude of orbit in km
float altitude = 400;
//desired angular velocities below
double wdx = 0;
double wdy = 0;
double wdz = 1;
//
float degrees_to_radians(float degrees)
{
  return degrees * (M_PI / 180);
}
//Orbital inclination set to ISS orbit
//float inclination = degrees_to_radians(51.6F);
//float inclination = degrees_to_radians(10.0F);
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

// float x_new = 0;
// float x_old = 0;
// float x_oldLP = 0;
// float x_newLP = 0;


// float y_new = 0;
// float y_old = 0;
// float y_oldLP = 0;
// float y_newLP = 0;

// float z_new = 0;
// float z_old = 0;
// float z_oldLP = 0;
// float z_newLP = 0;

// float wx_new = 0;
// float wx_old = 0;
// float wx_oldLP = 0;
// float wx_newLP = 0;

// float wy_new = 0;
// float wy_old = 0;
// float wy_oldLP = 0;
// float wy_newLP = 0;

// float wz_new = 0;
// float wz_old = 0;
// float wz_oldLP = 0;
// float wz_newLP = 0;

// float a = 0.43708596;
// float b[] = {0.28145702, 0.28145702};

// float LP_filter(float old_LPval, float new_val, float old_val, float a, float b[])
// {
//   float new_LPval = a * old_LPval + b[0] * new_val + b[1] * old_val;
//   return new_LPval;
// }

void setup()
{
  //sqrt(x^2 + y^2 + z^2) < 5 degrees
  if (detumbling)
  {
    //plantObj.initialize(0.06, -0.05, 0.07, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude,inclination,csarea,num_loops,ampfactor);
    plantObj.initialize(0.06,-0.05,0.07,get_quat0(alpha_angle),0.0,0.0,get_quat3(alpha_angle),altitude,inclination,csarea,num_loops,ampfactor);
  }
  else
  {
    //plantObj.initialize(0.0, 0.0, 1.0, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle),altitude, inclination,csarea,num_loops,ampfactor);
    plantObj.initialize(0.03598, -0.013903, 1.0565, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude, inclination, csarea, num_loops, ampfactor);
  }

  //starshotObj.initialize(kane_damper_c, kane_Id, ampfactor, csarea, num_loops, wdx, wdy, wdz);
  starshotObj.initialize(0.2, kane_damper_c, kane_Id,ampfactor,max_current,csarea,num_loops,wdx,wdy,wdz);


  // if (!imu.begin())
  // {
  //   while (1){
  //     Serial.println("wrong");
  //   };
  // }
  //DataLogSetup();
  // Serial.println("Found LSM9DS1 9DOF");
  // Serial.println("Setting up imu9DS1 9DOF");
  // imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  // imu.setupMag(imu.LSM9DS1_MAGGAIN_8GAUSS);
  // imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  delay(1000);
}

// void loop()

// {
//   // sensors_event_t accel, mag, gyro, temp;
//   // imu.getEvent(&accel, &mag, &gyro, &temp);
//   // float IMUData[6] = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};
//   // DataLog(IMUData,6);
//   //Serial.printf(" % f, % f, % f, % f, % f, % f \n ", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);


//   // x_new = mag.magnetic.x;
//   // x_newLP = LP_filter(x_oldLP, x_new, x_old, a, b);

//   // y_new = mag.magnetic.y;
//   // y_newLP = LP_filter(y_oldLP, y_new, y_old, a, b);

//   // z_new = mag.magnetic.z;
//   // z_newLP = LP_filter(z_oldLP, z_new, z_old, a, b);
//   // ///
//   // wx_new = gyro.gyro.x;
//   // wx_newLP = LP_filter(wx_oldLP, wx_new, wx_old, a, b);

//   // wy_new = gyro.gyro.y;
//   // wy_newLP = LP_filter(wy_oldLP, wy_new, wy_old, a, b);

//   // wz_new = gyro.gyro.z;
//   // wz_newLP = LP_filter(wz_oldLP, wz_new, wz_old, a, b);

//   // starshotObj.rtU.w[0] = wx_newLP;
//   // starshotObj.rtU.w[1] = wy_newLP;
//   // starshotObj.rtU.w[2] = wz_newLP;

//   // starshotObj.rtU.magneticfield[0] = (x_newLP) / 1000000.0;
//   // starshotObj.rtU.magneticfield[1] = (y_newLP) / 1000000.0;
//   // starshotObj.rtU.magneticfield[2] = (z_newLP) / 1000000.0;

//   // // starshotObj.rtU.magneticfield[0] = x_new / 1000000.0;
//   // // starshotObj.rtU.magneticfield[1] = y_new / 1000000.0;
//   // // starshotObj.rtU.magneticfield[2] = z_new / 1000000.0;

//   // // starshotObj.rtU.magneticfield[0] =13 / 1000000.0;
//   // // starshotObj.rtU.magneticfield[1] = -37 / 1000000.0;
//   // // starshotObj.rtU.magneticfield[2] = 24 / 1000000.0;

//   // starshotObj.step();

//   // // Serial.printf("MAG: x: %f uT, y: %f uT, z: %f uT \n", x_input, y_input, z_input);
//   // // // Serial.printf("x: %f uT, y: %f uT, z: %f uT \n", x_new, y_new, z_new);
//   // // Serial.printf("Current: x: %f mA, y: %f mA, z: %f mA \n", starshotObj.rtY.point[0] * 1000.0, starshotObj.rtY.point[1] * 1000.0, starshotObj.rtY.point[2] * 1000.0);
//   // Serial.printf("%f,%f,%f,%f,%f,%f,%f\n",wx_newLP,wy_newLP,wz_newLP,x_newLP, y_newLP, z_newLP, starshotObj.rtY.point[2] * 1000.0);
//   // // Serial.printf("%f,%f,%f,%f,%f,%f \n", 13.0, -37.0, 24.0, starshotObj.rtY.point[0] * 1000.0, starshotObj.rtY.point[1] * 1000.0, starshotObj.rtY.point[2] * 1000.0);

//   // x_old = x_new;
//   // x_oldLP = x_newLP;

//   // y_old = y_new;
//   // y_oldLP = y_newLP;

//   // z_old = z_new;
//   // z_oldLP = z_newLP;

//   // wx_old = wx_new;
//   // wx_oldLP = wx_newLP;

//   // wy_old = y_new;
//   // wy_oldLP = wy_newLP;

//   // wz_old = wz_new;
//   // wz_oldLP = wz_newLP;


//   if (detumbling)
//   {
//     plantObj.rtU.current[0] = starshotObj.rtY.detumble[0];
//     plantObj.rtU.current[1] = starshotObj.rtY.detumble[1];
//     plantObj.rtU.current[2] = starshotObj.rtY.detumble[2];
//   }
//   else
//   {
//     plantObj.rtU.current[0] = starshotObj.rtY.point[0] ;
//     plantObj.rtU.current[1] = starshotObj.rtY.point[1] ;
//     plantObj.rtU.current[2] = starshotObj.rtY.point[2] ;
//   }

//   if (iteration % 10 == 0)
//   {

//     Serial.print(iteration * imu_delay);
//     Serial.print(",");

//     if (detumbling)
//     {
//       // Serial.print(plantObj.rtY.angularvelocity[0]);
//       // Serial.print(",");
//       // Serial.print(plantObj.rtY.angularvelocity[1]);
//       // Serial.print(",");
//       // Serial.print(plantObj.rtY.angularvelocity[2]);
//       // Serial.print(",");
      
//       // Serial.print(plantObj.rtY.magneticfield[0]*100000.0);
//       // Serial.print(",");
//       // Serial.print(plantObj.rtY.magneticfield[1]*100000.0);
//       // Serial.print(",");
//       // Serial.println(plantObj.rtY.magneticfield[2]*100000.0);

//       Serial.print(starshotObj.rtY.detumble[0]*1000.0);
//       Serial.print(",");
//       Serial.print(starshotObj.rtY.detumble[1]*1000.0);
//       Serial.print(",");
//       Serial.println(starshotObj.rtY.detumble[2]*1000.0);
//     }
//     else
//     {
//       // Serial.print(starshotObj.pointing_error);
//       // Serial.print(",");
//       Serial.println(starshotObj.rtY.point[2]*1000.0);
//       // Serial.print(",");
//       // Serial.print(plantObj.rtY.magneticfield[0] * 100000.0);
//       // Serial.print(",");
//       // Serial.print(plantObj.rtY.magneticfield[1] * 100000.0);
//       // Serial.print(",");
//       // Serial.println(plantObj.rtY.magneticfield[2] * 100000.0);
//     }
//  }

//   for (int i = 0; i < imu_delay / plantsim_step_size; i++)
//   {
//     plantObj.step(plantsim_step_size / 1000);
//   }

//   starshotObj.rtU.w[0] = plantObj.rtY.angularvelocity[0];
//   starshotObj.rtU.w[1] = plantObj.rtY.angularvelocity[1];
//   starshotObj.rtU.w[2] = plantObj.rtY.angularvelocity[2];

//   starshotObj.rtU.magneticfield[0] = plantObj.rtY.magneticfield[0];
//   starshotObj.rtU.magneticfield[1] = plantObj.rtY.magneticfield[1];
//   starshotObj.rtU.magneticfield[2] = plantObj.rtY.magneticfield[2];

//   starshotObj.step();

//   iteration++;
//   delay(5);
// }

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
      // Serial.print(plantObj.rtY.angularvelocity[0]);
      //  Serial.print(",");
      //  Serial.print(plantObj.rtY.angularvelocity[1]);
      //  Serial.print(",");
      //  Serial.print(plantObj.rtY.angularvelocity[2]);
      //  Serial.print(",");
      //  Serial.print(starshotObj.rtY.detumble[0]);
      //  //Serial.print(starshotObj.rtY.detumble[0]);
      //  Serial.print(",");
      //  //Serial.print(starshotObj.rtY.detumble[1]);
      //  Serial.print(starshotObj.rtY.detumble[1]);
      //  Serial.print(",");
      //  //Serial.println(starshotObj.rtY.detumble[2]);
      //  Serial.println(starshotObj.rtY.detumble[2]);
    }
    else
    {

      Serial.print(starshotObj.rtY.point[2] * 1000.0);
      Serial.print(",");
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
  starshotObj.rtU.magneticfield[0] = plantObj.rtY.magneticfield[0];
  starshotObj.rtU.magneticfield[1] = plantObj.rtY.magneticfield[1];
  starshotObj.rtU.magneticfield[2] = plantObj.rtY.magneticfield[2];

  iteration++;

  starshotObj.step();
}