#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "../lib/Plant_ert_rtw/Plant.h"
#include "../lib/StarshotACS_ert_rtw/StarshotACS.h"
#include <random>

// static Plant plantObj;
// static StarshotACS starshotObj;

int iteration = 0;
double imu_delay = 0.20; // sec
double RUN_TIME_HR = 100;

// Monte Carlo
int num_mc_iterations = 1000;
double percent_avg = 0.2; // percentage of iterations to consider for average pointing error

double Kp_min = 1e-10; // minimum value for Kp
double Kp_max = 1e-1;  // maximum value for Kp

double Kd_min = 1e-10; // minimum value for Kd
double Kd_max = 1e-1;  // maximum value for Kd

std::default_random_engine generator;
std::uniform_real_distribution<double> distribution_Kp_log(std::log10(Kp_min), std::log10(Kp_max));
std::uniform_real_distribution<double> distribution_Kd_log(std::log10(Kd_min), std::log10(Kd_max));
// Duty cycle
// 10 min on 30min off
double period = 40;
double duty_perc = 0.25;

/// PLANT PARAMETERS///
double altitude_input = 400;
double I_input[9] = {0.00195761450869, -5.836632382E-5, 2.27638093E-6,
                     -5.836632382E-5, 0.00196346658902, 8.8920475E-7, 2.27638093E-6, 8.8920475E-7,
                     0.00204697265884};

double inclination_input = 0.90058989402907408; // 51.6 deg in rad
double m_input = 1.3;                           // kg
double q0_input[4] = {0.5, 0.5, -0.18301270189221924, 0.6830127018922193};

double wx_input = 0.0;
double wy_input = 0.0;
double wz_input = 1.0;

/// STARSHOT PARAMETERS///
double A_input = 4.0E-5;
double Id_input = 0.0021;
double Kd_input = 0.0007935279615795299;
double Kp_input = 5.2506307629097953E-10;
double c_input = 0.004;
double i_max_input = 0.25;
double k_input = 13.5;
double n_input = 500.0;
double step_size_input = imu_delay; // sec

// given period(mins), duty_perc[0,1], and the current_time(mins)
// output if the current time is in the active range
bool duty_cyc(double current_time, double period, double duty_perc)
{
  // Calculate the time at which the signal becomes inactive
  double active_time = period * duty_perc;

  // Calculate the time within the current period 90, 20
  double time_in_current_period = fmod(current_time, period);

  // Check if the time is within the active range
  return time_in_current_period < active_time;
}

int main()

{

  std::ofstream outfile;

  // open output file
  outfile.open("output/test.txt", std::ios::app);
  if (!outfile.is_open())
  { // check if file opened successfully
    return -1;
  }
  ////////

  std::cout << "EACH SIMULATION TIME: " << RUN_TIME_HR << " hours"
            << "\n";
  std::cout << "Duty Period: " << period << " mins"
            << ", On for: " << duty_perc * 100 << "\%"
            << "\n";
  std::cout << "Monte Carlo Iterations: " << num_mc_iterations << "\n";

  for (int mc_iteration = 0; mc_iteration < num_mc_iterations; mc_iteration++)
  {
    iteration = 0;
    double pt_error_sum = 0.0;

    // Generate random numbers
    double Kp_rand_log = distribution_Kp_log(generator);
    double Kd_rand_log = distribution_Kd_log(generator);

    // Convert back to original scale
    double Kp_rand = std::pow(10, Kp_rand_log);
    double Kd_rand = std::pow(10, Kd_rand_log);

    // default sanity check  (why not 0? Testing if there is states leftover from last mc iteration)
    // checked so np
    if (mc_iteration == 0)
    {
      Kp_rand = Kp_input;
      Kd_rand = Kd_input;
    }

    ///////////////
    std::cout << mc_iteration + 1 << "/" << num_mc_iterations;
    std::cout << " | Kp: " << Kp_rand << " | Kd: " << Kd_rand;
    ///////////////
    Plant plantObj;
    StarshotACS starshotObj;

    plantObj.initialize(altitude_input, I_input, inclination_input, m_input, q0_input, wx_input, wy_input, wz_input);
    starshotObj.initialize(step_size_input, A_input, Id_input, Kd_rand, Kp_rand, c_input, i_max_input, k_input, n_input);

    while ((iteration * imu_delay / 3600.0) < RUN_TIME_HR)
    {
      // step 0.2s
      for (int i = 0; i < (int)(imu_delay / 0.001); i++)
      {
        if (duty_cyc(iteration * imu_delay / 60.0, period, duty_perc))
        {
          plantObj.rtU.current[0] = starshotObj.rtY.point[0];
          plantObj.rtU.current[1] = starshotObj.rtY.point[1];
          plantObj.rtU.current[2] = starshotObj.rtY.point[2];
        }
        else
        {
          plantObj.rtU.current[0] = 0.0;
          plantObj.rtU.current[1] = 0.0;
          plantObj.rtU.current[2] = 0.0;
        }

        plantObj.step();
      }

      if (duty_cyc(iteration * imu_delay / 60.0, period, duty_perc))
      {
        starshotObj.rtU.w[0] = plantObj.rtY.angularvelocity[0];
        starshotObj.rtU.w[1] = plantObj.rtY.angularvelocity[1];
        starshotObj.rtU.w[2] = plantObj.rtY.angularvelocity[2];
        starshotObj.rtU.Bfield_body[0] = plantObj.rtY.magneticfield[0];
        starshotObj.rtU.Bfield_body[1] = plantObj.rtY.magneticfield[1];
        starshotObj.rtU.Bfield_body[2] = plantObj.rtY.magneticfield[2];
        starshotObj.step();
      }

      // in the last percent_avg
      if ((iteration * imu_delay / 3600.0) > (1 - percent_avg) * RUN_TIME_HR)
      {
        pt_error_sum += starshotObj.rtY.pt_error;
      }
      iteration++;
    }

    double pt_avg = pt_error_sum / (percent_avg * RUN_TIME_HR * 3600 / imu_delay);
    std::cout << " | Last " << percent_avg * 100 << "\% pt_error averge: " << pt_avg << "\n";
    // write to file
    outfile << pt_avg << " ," << Kp_rand << " ," << Kd_rand << std::endl;
    ////////////////////////////////
  }

  outfile.close();
  return 0;
}
