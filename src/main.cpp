#include <iostream>
#include <fstream>
#include <thread>
#include "../lib/Plantv50_ert_rtw/Plantv50.h"
#include "../lib/StarshotACS_ert_rtw/StarshotACS.h"

static Plantv50ModelClass plantObj;
// static StarshotACSModelClass starshotObj;

static Plantv50ModelClass plantObj0;
static StarshotACSModelClass starshotObj0;

static Plantv50ModelClass plantObj1;
static StarshotACSModelClass starshotObj1;

static Plantv50ModelClass plantObj2;
static StarshotACSModelClass starshotObj2;

#define RUN_TIME_HR 100 // 24*14 - 2 weeks
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
// double kane_damper_c = 0.00001;
double kane_damper_c = 0.004;
// double kane_Id = 0.196;
double kane_Id = 0.0021;
// magnetorqueer hardware constants
double ampfactor = 1;
//double csarea = 0.07979645340118074;
double csarea = 4E-5;
double num_loops = 500;
// starshot.magnetorq.m_max_x/(starshot.magnetorq.A*starshot.magnetorq.n);
//double max_current = 0.02 / (csarea * num_loops);

double max_current = 10.0/1000.0;
//  Altitude of orbit in km
float altitude = 400;
// desired angular velocities below
double wdx = 0;
double wdy = 0;
double wdz = 1;

// power budget//
double simple_current = 10.0;
// in percent
#define DUTY_CYCLE1 0.50
#define DUTY_CYCLE2 0.75
// in mil seconds
int CYCLE_PERIOD = 9 * 60 * 1000;

///////////////

float degrees_to_radians(float degrees)
{
  return degrees * (M_PI / 180);
}
// Orbital inclination set to ISS orbit
float inclination = degrees_to_radians(10.0F);
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

struct Vector3D
{
  double x;
  double y;
  double z;
};

Vector3D normalizeVector(const Vector3D &v)
{
  double length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  Vector3D normalized;
  normalized.x = v.x / length;
  normalized.y = v.y / length;
  normalized.z = v.z / length;
  return normalized;
}

double dotProduct(const Vector3D &v1, const Vector3D &v2)
{
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

double pointingError(const Vector3D &magnetic_field)
{
  // Vector3D pointing_vector = normalizeVector(ToDirectionVector(ToEulerAngles(normalizeQ(pointing_quaternion))));
  Vector3D desired_vector = normalizeVector(magnetic_field);
  Vector3D body_vector = {0, 0, 1};

  double dot = dotProduct(body_vector, desired_vector);

  // Clamp the dot product value between -1.0 and 1.0
  if (dot > 1.0)
  {
    dot = 1.0;
  }
  else if (dot < -1.0)
  {
    dot = -1.0;
  }
  double angle = std::acos(dot);

  return angle;
}

int main()
{
  std::ofstream outfile;
  outfile.open("output/new-maxi10mA-10mA-simple-100-50-75-9min-336hr.txt");
  if (!outfile.is_open())
  { // check if file opened successfully
    return -1;
  }

  plantObj.initialize(0.03598, -0.013903, 1.0565, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude, inclination, csarea, num_loops, ampfactor);
  plantObj0.initialize(0.03598, -0.013903, 1.0565, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude, inclination, csarea, num_loops, ampfactor);
  plantObj1.initialize(0.03598, -0.013903, 1.0565, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude, inclination, csarea, num_loops, ampfactor);
  plantObj2.initialize(0.03598, -0.013903, 1.0565, get_quat0(alpha_angle), 0.0, 0.0, get_quat3(alpha_angle), altitude, inclination, csarea, num_loops, ampfactor);

  // starshotObj.initialize(0.2, kane_damper_c, kane_Id, ampfactor, max_current, csarea, num_loops, wdx, wdy, wdz);
  starshotObj0.initialize(0.2, kane_damper_c, kane_Id, ampfactor, max_current, csarea, num_loops, wdx, wdy, wdz);
  starshotObj1.initialize(0.2, kane_damper_c, kane_Id, ampfactor, max_current, csarea, num_loops, wdx, wdy, wdz);
  starshotObj2.initialize(0.2, kane_damper_c, kane_Id, ampfactor, max_current, csarea, num_loops, wdx, wdy, wdz);

  std::cout << "TOTAL SIMULATION TIME: " << RUN_TIME_HR << " hours"
            << "\n ";

  while ((iteration * imu_delay / 3600000.0) < RUN_TIME_HR)
  {
    /////////////////////////////PROGRESS BAR///////////////////////////////////
    float progress = (iteration * imu_delay / 3600000.0) / RUN_TIME_HR;
    int barWidth = 70;
    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i)
    {
      if (i < pos)
        std::cout << "=";
      else if (i == pos)
        std::cout << ">";
      else
        std::cout << " ";
    }

    std::cout << "] " << std::fixed << std::setprecision(1) << (progress * 100.0) << " %\r";
    std::cout.flush();
    ////////////////////////////UPDATE PLANT////////////////////////////////////
    plantObj.rtU.current[0] = 0.0;
    plantObj.rtU.current[1] = 0.0;
    plantObj.rtU.current[2] = simple_current / 1000.0;
    // simple mode <---

    plantObj0.rtU.current[0] = starshotObj0.rtY.point[0];
    plantObj0.rtU.current[1] = starshotObj0.rtY.point[1];
    plantObj0.rtU.current[2] = starshotObj0.rtY.point[2];

    if (((iteration * imu_delay) % CYCLE_PERIOD) < DUTY_CYCLE1 * CYCLE_PERIOD)
    {
      plantObj1.rtU.current[0] = starshotObj1.rtY.point[0];
      plantObj1.rtU.current[1] = starshotObj1.rtY.point[1];
      plantObj1.rtU.current[2] = starshotObj1.rtY.point[2];
    }
    else
    {
      plantObj1.rtU.current[0] = 0.0;
      plantObj1.rtU.current[1] = 0.0;
      plantObj1.rtU.current[2] = 0.0;
    }

    if (((iteration * imu_delay) % CYCLE_PERIOD) < DUTY_CYCLE2 * CYCLE_PERIOD)
    {
      plantObj2.rtU.current[0] = starshotObj2.rtY.point[0];
      plantObj2.rtU.current[1] = starshotObj2.rtY.point[1];
      plantObj2.rtU.current[2] = starshotObj2.rtY.point[2];
    }
    else
    {
      plantObj2.rtU.current[0] = 0.0;
      plantObj2.rtU.current[1] = 0.0;
      plantObj2.rtU.current[2] = 0.0;
    }

    for (int i = 0; i < imu_delay / plantsim_step_size; i++)
    {
      plantObj.step(plantsim_step_size / 1000);
      plantObj0.step(plantsim_step_size / 1000);
      plantObj1.step(plantsim_step_size / 1000);
      plantObj2.step(plantsim_step_size / 1000);
    }
    ///////////////////////////////////////////////////////////////////////////////

    Vector3D mag_field = {plantObj.rtY.magneticfield[0],
                          plantObj.rtY.magneticfield[1],
                          plantObj.rtY.magneticfield[2]};
    Vector3D mag_field0 = {plantObj0.rtY.magneticfield[0],
                           plantObj0.rtY.magneticfield[1],
                           plantObj0.rtY.magneticfield[2]};
    Vector3D mag_field1 = {plantObj1.rtY.magneticfield[0],
                           plantObj1.rtY.magneticfield[1],
                           plantObj1.rtY.magneticfield[2]};
    Vector3D mag_field2 = {plantObj2.rtY.magneticfield[0],
                           plantObj2.rtY.magneticfield[1],
                           plantObj2.rtY.magneticfield[2]};

    double error = pointingError(mag_field);
    double error0 = pointingError(mag_field0);
    double error1 = pointingError(mag_field1);
    double error2 = pointingError(mag_field2);

    if (((iteration * imu_delay) % CYCLE_PERIOD) < DUTY_CYCLE1 * CYCLE_PERIOD)
    {
      if (iteration % 100 == 0)
      {

        outfile << simple_current << ", " << error << " ," << starshotObj0.rtY.point[2] * 1000.0 << ", " << error0 << ", " << starshotObj1.rtY.point[2] * 1000.0 << " ," << error1 << ", " << starshotObj2.rtY.point[2] * 1000.0 << " ," << error2 << '\n';
        outfile.flush();
      }
    }
    else if (((iteration * imu_delay) % CYCLE_PERIOD) < DUTY_CYCLE2 * CYCLE_PERIOD)
    {
      if (iteration % 100 == 0)
      {
        outfile << simple_current << ", " << error << " ," << starshotObj0.rtY.point[2] * 1000.0 << ", " << error0 << ", " << 0.0 << " ," << error1 << ", " << starshotObj2.rtY.point[2] * 1000.0 << " ," << error2 << '\n';

        outfile.flush();
      }
    }
    else
    {
      if (iteration % 100 == 0)
      {
        outfile << simple_current << ", " << error << " ," << starshotObj0.rtY.point[2] * 1000.0 << ", " << error0 << ", " << 0.0 << " ," << error1 << ", " << 0.0 << " ," << error2 << '\n';
        outfile.flush();
      }
    }
    ///////////////////////////////////////////////////////////////////////////////
    iteration++;

    // simple mode no need
    starshotObj0.rtU.w[0] = plantObj0.rtY.angularvelocity[0];
    starshotObj0.rtU.w[1] = plantObj0.rtY.angularvelocity[1];
    starshotObj0.rtU.w[2] = plantObj0.rtY.angularvelocity[2];
    starshotObj0.rtU.magneticfield[0] = plantObj0.rtY.magneticfield[0];
    starshotObj0.rtU.magneticfield[1] = plantObj0.rtY.magneticfield[1];
    starshotObj0.rtU.magneticfield[2] = plantObj0.rtY.magneticfield[2];
    starshotObj0.step();

    starshotObj1.rtU.w[0] = plantObj1.rtY.angularvelocity[0];
    starshotObj1.rtU.w[1] = plantObj1.rtY.angularvelocity[1];
    starshotObj1.rtU.w[2] = plantObj1.rtY.angularvelocity[2];
    starshotObj1.rtU.magneticfield[0] = plantObj1.rtY.magneticfield[0];
    starshotObj1.rtU.magneticfield[1] = plantObj1.rtY.magneticfield[1];
    starshotObj1.rtU.magneticfield[2] = plantObj1.rtY.magneticfield[2];
    starshotObj1.step();

    starshotObj2.rtU.w[0] = plantObj2.rtY.angularvelocity[0];
    starshotObj2.rtU.w[1] = plantObj2.rtY.angularvelocity[1];
    starshotObj2.rtU.w[2] = plantObj2.rtY.angularvelocity[2];
    starshotObj2.rtU.magneticfield[0] = plantObj2.rtY.magneticfield[0];
    starshotObj2.rtU.magneticfield[1] = plantObj2.rtY.magneticfield[1];
    starshotObj2.rtU.magneticfield[2] = plantObj2.rtY.magneticfield[2];
    starshotObj2.step();

    // wait for some time
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     std::cout << "Time: " << (iteration * imu_delay / 3600000.0) << std::endl;
  }
  outfile.close();
  std::cout << "\n"
            << "done."
            << "\n";
  return 0;
}
