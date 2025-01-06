#pragma once
#ifndef ARDUINO_EIGEN_H
#define ARDUINO_EIGEN_H

#ifndef EMBEDDED_BUILD
#include <Arduino.h>
#else
#include "../NativeMocks/MockArduino.h"
#endif

#include "ArduinoEigen/ArduinoEigenCommon.h"

// guarantee that the Eigen code that you are #including is licensed under the MPL2
#define EIGEN_MPL2_ONLY

#include "ArduinoEigen/Eigen/Eigen"
#include "ArduinoEigen/ArduinoEigenExtension.h"

#endif  // ARDUINO_EIGEN_H
