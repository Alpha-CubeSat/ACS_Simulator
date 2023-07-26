#include <iostream>
#include <fstream>
#include <thread>
#include "../lib/Plant_ert_rtw/Plant.h"
#include "../lib/StarshotACS_ert_rtw/StarshotACS.h"

static Plant plantObj;
static StarshotACS starshotObj;


int iteration = 0;
double imu_delay = 0.2;
double RUN_TIME_HR = 100;

/// PLANT PARAMETERS///
double altitude_input = 400;
double I_input[9] = {0.00195761450869, -5.836632382E-5, 2.27638093E-6,
                     -5.836632382E-5, 0.00196346658902, 8.8920475E-7, 2.27638093E-6, 8.8920475E-7,
                     0.00204697265884};

double inclination_input = 0.90058989402907408; // 51.6 deg in rad
double m_input = 1.3;
double q0_input[4] = {0.49999999999999994, 0.5, -0.18301270189221924,
                      0.6830127018922193};

double wx_input = 0.0;
double wy_input = 0.0;
double wz_input = 1.0;

/// STARSHOT PARAMETERS///
double A_input = 4.0E-5;
double Id_input = 0.0021;
double Kd_input = 0.0007935279615795299;
double Kp_input = 5.2506307629097953E-10;
double c_input = 0.004;
double i_max_input = 0.250;
double k_input = 13.5;
double n_input = 500.0;

int main()
{
  std::ofstream outfile;
  outfile.open("output/test.txt");
  if (!outfile.is_open())
  { // check if file opened successfully
    return -1;
  }

  plantObj.initialize(altitude_input, I_input, inclination_input, m_input, q0_input, wx_input, wy_input, wz_input);
  starshotObj.initialize(A_input, Id_input, Kd_input, Kp_input, c_input, i_max_input, k_input, n_input);

  // plantObj.initialize();
  // starshotObj.initialize();

  std::cout << "TOTAL SIMULATION TIME: " << RUN_TIME_HR << " hours"
            << "\n ";

  while ((iteration * imu_delay / 3600.0) < RUN_TIME_HR)
  {
    // /////////////////////////////PROGRESS BAR///////////////////////////////////
    // float progress = (iteration * imu_delay / 3600000.0) / RUN_TIME_HR;
    // int barWidth = 70;
    // std::cout << "[";
    // int pos = barWidth * progress;
    // for (int i = 0; i < barWidth; ++i)
    // {
    //   if (i < pos)
    //     std::cout << "=";
    //   else if (i == pos)
    //     std::cout << ">";
    //   else
    //     std::cout << " ";
    // }

    // std::cout << "] " << std::fixed << std::setprecision(1) << (progress * 100.0) << (iteration * imu_delay)<< " % \r";
    // std::cout.flush();
    ////////////////////////////UPDATE PLANT////////////////////////////////////
    // plantObj.rtU.current[0] = starshotObj.rtY.point[0];
    // plantObj.rtU.current[1] = starshotObj.rtY.point[1];
    // plantObj.rtU.current[2] = starshotObj.rtY.point[2];

    plantObj.rtU.current[0] =0.0;
    plantObj.rtU.current[1] = 0.0;
    plantObj.rtU.current[2] = 0.25;

    // step 0.2s
    for (int i = 0; i < (int)(imu_delay / 0.001); i++)
    {
      plantObj.step();
    }
    ///////////////////////////////////////////////////////////////////////////////

    // starshotObj.rtU.w[0] = plantObj.rtY.angularvelocity[0];
    // starshotObj.rtU.w[1] = plantObj.rtY.angularvelocity[1];
    // starshotObj.rtU.w[2] = plantObj.rtY.angularvelocity[2];
    // starshotObj.rtU.Bfield_body[0] = plantObj.rtY.magneticfield[0];
    // starshotObj.rtU.Bfield_body[1] = plantObj.rtY.magneticfield[1];
    // starshotObj.rtU.Bfield_body[2] = plantObj.rtY.magneticfield[2];

    // starshotObj.step();
    // write to file
    if ((int)(iteration * imu_delay) % 400==0)
    {
      std::cout << "time(s): " << (iteration * imu_delay) << " , pointing error(deg): " << starshotObj.rtY.pt_error << " , z-axis current(mA): " << starshotObj.rtY.point[2] * 1000.0 << ", w_x: " << plantObj.rtY.angularvelocity[0] << ", w_y: " << plantObj.rtY.angularvelocity[1] << ", w_z: " << plantObj.rtY.angularvelocity[2] << "\n";
      outfile << (iteration * imu_delay) << " ," << starshotObj.rtY.pt_error << " ," << starshotObj.rtY.point[2] * 1000.0 << '\n';
    }

    iteration++;
  }


  outfile.close();
  std::cout << "\n"
            << "done."
            << "\n";
  return 0;
}
