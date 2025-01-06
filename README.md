# ACS_Simulator

Simulation of ACS on different platforms

## Branches

`main`: Runs on Flight-Hardware-Like Hardwares, usually means Teensy3.5, Adafruit_LSM9DS1, H-Bridge, mag-torq.

`pcsim`: Runs on computers--really fast!

`monte-carlo`: Feature Branch, using the Monte-Carlo Method, runs a bunch of simulations and finds good parameters.


# Adjustments made to main for the incorporation of EKF include:

 - State prediction based on system dynamics
 - Measurement correction using calibration data
 - Offset tracking and compensation
 - Voltage-dependent calibration
 - Magnetic field corrections

 The newly incorporated EKF along with the unit tests for these changes required some adjustments be made to `platformio.ini`, the previous state has been preserved in the commented section above the current configuration.