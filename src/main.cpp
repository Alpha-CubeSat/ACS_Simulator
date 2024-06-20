#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "../lib/ACS_libs/Plant_ert_rtw/Plant.h"
#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"
#include <random>

#define FILE_NAME "output/mc_detumble_wz1_3openI_2hr.txt"

double imu_delay = 0.10; // sec (step size)
double RUN_TIME_HR = 2;

// Monte Carlo
int num_mc_iterations = 300;
double percent_avg = 0.05; // percentage of iterations to consider for average

double Kp_min = 1e-10; // minimum value for Kp
double Kp_max = 1e-1;  // maximum value for Kp

double Kd_min = 1e-10; // minimum value for Kd
double Kd_max = 1e-1;  // maximum value for Kd

double c_min = 1e-8; // minimum value for c
double c_max = 1e-3; // maximum value for c

double Id_min = 1e-2; // minimum value for Id
double Id_max = 1e2;  // maximum value for Id

// Duty cycle
// period*duty_perc min on
double period = 40;
double duty_perc = 1;

/// PLANT PARAMETERS///
double altitude_input = 400;

// old one
//  double I_input[9] = {0.0021342, -0.0000596453, 0.0000182133,
//                       -0.0000596453, 0.00210682, 0.0000194886,
//                       0.0000182133, 0.0000194886, 0.00244726};

// double I_input[9] = {0.002052, -0.000039, 0.000001,
//                      -0.000039, 0.002057, 0.000044,
//                      0.000001, 0.000044, 0.002406};

// 3 steel wights+ wire accounted for correctly 80-20
// double I_input[9] = {0.002102, -0.000049, 0.000011,
//                      -0.000049, 0.002084, 0.000019,
//                      0.000011, 0.000019, 0.002437};

// door open
double I_input[9] = {0.002253, -0.000048, -0.000031,
                     -0.000048, 0.002225, 0.000023,
                     -0.000031, 0.000023, 0.002387};

double inclination_input = 0.90058989402907408; // 51.6 deg in rad
double m_input = 1.3;                           // kg
double q0_input[4] = {0.5, 0.5, -0.18301270189221924, 0.6830127018922193};

// double wx_input = 0.02;
// double wy_input = -0.04;
// double wz_input = 0.01;

double wx_input = 0.5;
double wy_input = 3.14159265358979;
double wz_input = 4.0;

/// STARSHOT PARAMETERS///
double A_input = 4.0E-5;
double Id_input = 0.196;
double Kd_input = 0.0007935279615795299;
double Kp_input = 5.2506307629097953E-10;
double c_input = 1.0E-5;
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
  outfile.open(FILE_NAME, std::ios::app);
  if (!outfile.is_open())
  { // check if file opened successfully
    return -1;
  }

  std::cout << "EACH SIMULATION TIME: " << RUN_TIME_HR << " hours"
            << "\n";
  std::cout << "Duty Period: " << period << " mins"
            << ", On for: " << duty_perc * 100 << "\%"
            << "\n";
  std::cout << "Monte Carlo Iterations: " << num_mc_iterations << "\n";

  for (int mc_iteration = 0; mc_iteration < num_mc_iterations; mc_iteration++)
  {

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution_Kp_log(std::log10(Kp_min), std::log10(Kp_max));
    std::uniform_real_distribution<double> distribution_Kd_log(std::log10(Kd_min), std::log10(Kd_max));
    std::uniform_real_distribution<double> distribution_c_log(std::log10(c_min), std::log10(c_max));
    std::uniform_real_distribution<double> distribution_Id_log(std::log10(Id_min), std::log10(Id_max));

    int iteration = 0;
    double pt_error_sum = 0.0;
    double w_x_sum = 0.0;
    double w_y_sum = 0.0;
    double w_z_sum = 0.0;

    // Generate random numbers
    double Kp_rand_log = distribution_Kp_log(generator);
    double Kd_rand_log = distribution_Kd_log(generator);

    double c_rand_log = distribution_c_log(generator);
    double Id_rand_log = distribution_Id_log(generator);

    // Convert back to original scale
    double Kp_rand = std::pow(10, Kp_rand_log);
    double Kd_rand = std::pow(10, Kd_rand_log);
    double c_rand = std::pow(10, c_rand_log);
    double Id_rand = std::pow(10, Id_rand_log);

    // default sanity check  (why not 0? Testing if there is states leftover from last mc iteration)
    // checked so np
    if (mc_iteration == 0)
    {
      Kp_rand = Kp_input;
      Kd_rand = Kd_input;
      c_rand = c_input;
      Id_rand = Id_input;
    }

    Plant plantObj;
    StarshotACS starshotObj;

    plantObj.initialize(0.001, altitude_input, I_input, inclination_input, m_input, q0_input, wx_input, wy_input, wz_input);
    starshotObj.initialize(step_size_input, A_input, Id_rand, Kd_rand, Kp_rand, c_rand, i_max_input, k_input, n_input);

    while ((iteration * imu_delay / 3600.0) < RUN_TIME_HR)
    {
      for (int i = 0; i < (int)(imu_delay / 0.001); i++)
      {
        if (duty_cyc(iteration * imu_delay / 60.0, period, duty_perc))
        {
          plantObj.rtU.current[0] = starshotObj.rtY.detumble[0] * 0.88; // scaled by 0.88 to account for hardware loss
          plantObj.rtU.current[1] = starshotObj.rtY.detumble[1] * 0.88;
          plantObj.rtU.current[2] = starshotObj.rtY.detumble[2] * 0.88;
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
        w_x_sum += plantObj.rtY.angularvelocity[0];
        w_y_sum += plantObj.rtY.angularvelocity[1];
        w_z_sum += plantObj.rtY.angularvelocity[2];
      }
      iteration++;
    }

    double pt_avg = pt_error_sum / (percent_avg * RUN_TIME_HR * 3600 / imu_delay);
    double w_x_avg = w_x_sum / (percent_avg * RUN_TIME_HR * 3600 / imu_delay);
    double w_y_avg = w_y_sum / (percent_avg * RUN_TIME_HR * 3600 / imu_delay);
    double w_z_avg = w_z_sum / (percent_avg * RUN_TIME_HR * 3600 / imu_delay);

    // print
    std::cout << mc_iteration + 1 << "/" << num_mc_iterations;
    // std::cout << " | Kp: " << Kp_rand << " | Kd: " << Kd_rand;
    std::cout << " | c: " << c_rand << " | Id: " << Id_rand;
    // std::cout << " | Last " << percent_avg * 100 << "\% pt_error averge: " << pt_avg << "\n";
    std::cout << " | Last " << percent_avg * 100 << "\% wx average: " << w_x_avg << " wy average: " << w_y_avg << " wz average: " << w_z_avg << "\n";

    // write
    // outfile << pt_avg << " ," << Kp_rand << " ," << Kd_rand << ", " << duty_perc << std::endl;
    outfile << w_x_avg << " ," << w_y_avg << " ," << w_z_avg << " ," << c_rand << " ," << Id_rand << std::endl;
  }

  outfile.close();
  return 0;
}
