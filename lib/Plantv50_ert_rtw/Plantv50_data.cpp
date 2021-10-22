//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Plantv50_data.cpp
//
// Code generated for Simulink model 'Plantv50'.
//
// Model version                  : 9.1
// Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
// C/C++ source code generated on : Tue Sep 28 14:41:31 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "Plantv50.h"

// Constant parameters (default storage)
const ConstPPlant rtConstPPlant = {
  // Expression: starshot.IC.massproperties.I
  //  Referenced by: '<S3>/Constant'

  { 0.00195761450869, -5.836632382E-5, 2.27638093E-6, -5.836632382E-5,
    0.00196346658902, 8.8920475E-7, 2.27638093E-6, 8.8920475E-7,
    0.00204697265884 },

  // Expression: starshot.IC.massproperties.Iinv
  //  Referenced by: '<S3>/Constant1'

  { 511.27961796087834, 15.198640692914713, -0.57518202338311786,
    15.198640692914713, 509.7551984404119, -0.23833959751184652,
    -0.57518202338311786, -0.23833959751184652, 488.52705332800241 },

  // Expression: eye(3,3)
  //  Referenced by: '<S7>/Matrix Gain'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }
};

//
// File trailer for generated code.
//
// [EOF]
//
