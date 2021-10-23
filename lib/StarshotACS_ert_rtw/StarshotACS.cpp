//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: StarshotACS.cpp
//
// Code generated for Simulink model 'StarshotACS'.
//
// Model version                  : 1.77
// Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
// C/C++ source code generated on : Fri Oct 22 23:47:34 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "StarshotACS.h"

// Model step function
void StarshotACSModelClass::step()
{
  real_T rtb_Gain[3];
  real_T rtb_TrigonometricFunction5;
  real_T rtb_Product7;
  real_T rtb_TSamp[3];
  real_T rtb_VectorConcatenate[9];
  real_T rtb_Saturation3;
  real_T tmp;
  real_T rtb_VectorConcatenate_0[9];
  int32_T i;
  real_T rtb_Product1;
  real_T rtb_Gain8_idx_1;
  real_T rtb_Gain8_idx_2;
  real_T rtb_Gain8_idx_0;
  real_T rtb_Product1_idx_0;
  real_T rtb_Product1_idx_1;
  int32_T tmp_0;

  // Outputs for Atomic SubSystem: '<Root>/StarshotACS'
  // Gain: '<S2>/Kane damping' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   Gain: '<S2>/Gain 2'

  rtb_Gain8_idx_0 = 0.004 * -rtDW.DiscreteTimeIntegrator_DSTATE[0];

  // Gain: '<S2>/Gain 2' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'

  rtb_Product1_idx_0 = -rtDW.DiscreteTimeIntegrator_DSTATE[0];

  // Gain: '<S2>/Kane damping' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   Gain: '<S2>/Gain 2'

  rtb_Gain8_idx_1 = 0.004 * -rtDW.DiscreteTimeIntegrator_DSTATE[1];

  // Gain: '<S2>/Gain 2' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'

  rtb_Product1_idx_1 = -rtDW.DiscreteTimeIntegrator_DSTATE[1];
  rtb_Product1 = -rtDW.DiscreteTimeIntegrator_DSTATE[2];

  // Gain: '<S2>/Kane damping' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   Gain: '<S2>/Gain 2'

  rtb_Gain8_idx_2 = 0.004 * -rtDW.DiscreteTimeIntegrator_DSTATE[2];

  // S-Function (sdsp2norm2): '<S2>/Normalization' incorporates:
  //   DotProduct: '<S8>/Dot Product6'
  //   Inport: '<Root>/Bfield_body'

  rtb_Product7 = (rtU.Bfield_body[0] * rtU.Bfield_body[0] + rtU.Bfield_body[1] *
                  rtU.Bfield_body[1]) + rtU.Bfield_body[2] * rtU.Bfield_body[2];
  rtb_Saturation3 = 1.0 / (rtb_Product7 + 1.0E-10);
  rtb_Gain[0] = rtU.Bfield_body[0] * rtb_Saturation3;

  // SampleTimeMath: '<S5>/TSamp' incorporates:
  //   Inport: '<Root>/angularvelocity'
  //
  //  About '<S5>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp[0] = rtU.w[0] * 100.0;

  // S-Function (sdsp2norm2): '<S2>/Normalization' incorporates:
  //   Inport: '<Root>/Bfield_body'

  rtb_Gain[1] = rtU.Bfield_body[1] * rtb_Saturation3;

  // SampleTimeMath: '<S5>/TSamp' incorporates:
  //   Inport: '<Root>/angularvelocity'
  //
  //  About '<S5>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp[1] = rtU.w[1] * 100.0;

  // S-Function (sdsp2norm2): '<S2>/Normalization' incorporates:
  //   Inport: '<Root>/Bfield_body'

  rtb_Gain[2] = rtU.Bfield_body[2] * rtb_Saturation3;

  // SampleTimeMath: '<S5>/TSamp' incorporates:
  //   Inport: '<Root>/angularvelocity'
  //
  //  About '<S5>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp[2] = rtU.w[2] * 100.0;

  // Product: '<S4>/Product2'
  rtb_Saturation3 = rtb_Gain[0];

  // Product: '<S4>/Product4'
  rtb_TrigonometricFunction5 = rtb_Gain[1];

  // Product: '<S4>/Product5'
  tmp = rtb_Gain[0];

  // Gain: '<S2>/Gain' incorporates:
  //   Product: '<S4>/Product'
  //   Product: '<S4>/Product1'
  //   Product: '<S4>/Product2'
  //   Product: '<S4>/Product3'
  //   Sum: '<S4>/Sum'
  //   Sum: '<S4>/Sum1'

  rtb_Gain[0] = (rtb_Gain8_idx_1 * rtb_Gain[2] - rtb_Gain8_idx_2 * rtb_Gain[1]) *
    46.300583387350684;
  rtb_Gain[1] = (rtb_Gain8_idx_2 * rtb_Saturation3 - rtb_Gain8_idx_0 * rtb_Gain
                 [2]) * 46.300583387350684;

  // SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn1' incorporates:
  //   Constant: '<S6>/Constant3'
  //   Gain: '<S6>/Gain'
  //   Inport: '<Root>/angularvelocity'

  rtb_VectorConcatenate[0] = 0.0;
  rtb_VectorConcatenate[1] = rtU.w[2];
  rtb_VectorConcatenate[2] = -rtU.w[1];

  // SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn2' incorporates:
  //   Constant: '<S6>/Constant3'
  //   Gain: '<S6>/Gain1'
  //   Inport: '<Root>/angularvelocity'

  rtb_VectorConcatenate[3] = -rtU.w[2];
  rtb_VectorConcatenate[4] = 0.0;
  rtb_VectorConcatenate[5] = rtU.w[0];

  // SignalConversion: '<S6>/ConcatBufferAtVector ConcatenateIn3' incorporates:
  //   Constant: '<S6>/Constant3'
  //   Gain: '<S6>/Gain2'
  //   Inport: '<Root>/angularvelocity'

  rtb_VectorConcatenate[6] = rtU.w[1];
  rtb_VectorConcatenate[7] = -rtU.w[0];
  rtb_VectorConcatenate[8] = 0.0;

  // Saturate: '<S2>/Saturation3'
  rtb_Gain8_idx_2 = rtb_Gain[0];

  // Saturate: '<S2>/Saturation4'
  rtb_Saturation3 = rtb_Gain[1];

  // Saturate: '<S2>/Saturation5' incorporates:
  //   Gain: '<S2>/Gain'
  //   Product: '<S4>/Product4'
  //   Product: '<S4>/Product5'
  //   Sum: '<S4>/Sum2'

  rtb_Gain8_idx_0 = (rtb_Gain8_idx_0 * rtb_TrigonometricFunction5 -
                     rtb_Gain8_idx_1 * tmp) * 46.300583387350684;

  // Sqrt: '<S8>/Sqrt4' incorporates:
  //   Sqrt: '<S14>/Sqrt4'

  rtb_Product7 = std::sqrt(rtb_Product7);

  // DotProduct: '<S9>/Dot Product6'
  rtb_Gain8_idx_1 = 0.0;
  for (i = 0; i < 3; i++) {
    // Product: '<S3>/Product6' incorporates:
    //   Inport: '<Root>/Bfield_body'
    //   Product: '<S3>/Product4'
    //   Sqrt: '<S8>/Sqrt4'

    rtb_TrigonometricFunction5 = ((rtConstB.VectorConcatenate[i + 3] *
      rtU.Bfield_body[1] + rtConstB.VectorConcatenate[i] * rtU.Bfield_body[0]) +
      rtConstB.VectorConcatenate[i + 6] * rtU.Bfield_body[2]) / rtb_Product7;

    // DotProduct: '<S9>/Dot Product6'
    rtb_Gain8_idx_1 += rtb_TrigonometricFunction5 * rtb_TrigonometricFunction5;

    // Product: '<S3>/Product6' incorporates:
    //   Inport: '<Root>/Bfield_body'
    //   Inport: '<Root>/angularvelocity'
    //   Product: '<S11>/Product3'
    //   Product: '<S3>/Product4'

    rtb_Gain[i] = rtU.w[i] * rtU.Bfield_body[i];
  }

  // Sqrt: '<S9>/Sqrt4' incorporates:
  //   DotProduct: '<S9>/Dot Product6'

  rtb_Gain8_idx_1 = std::sqrt(rtb_Gain8_idx_1);

  // Trigonometry: '<S3>/Trigonometric Function5'
  if (rtb_Gain8_idx_1 > 1.0) {
    rtb_Gain8_idx_1 = 1.0;
  } else {
    if (rtb_Gain8_idx_1 < -1.0) {
      rtb_Gain8_idx_1 = -1.0;
    }
  }

  rtb_TrigonometricFunction5 = std::asin(rtb_Gain8_idx_1);

  // End of Trigonometry: '<S3>/Trigonometric Function5'

  // SampleTimeMath: '<S7>/TSamp'
  //
  //  About '<S7>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Gain8_idx_1 = rtb_TrigonometricFunction5 * 100.0;

  // Switch: '<S11>/Switch' incorporates:
  //   Constant: '<S11>/Constant3'
  //   Constant: '<S11>/Constant4'
  //   DotProduct: '<S13>/Dot Product6'
  //   Inport: '<Root>/angularvelocity'
  //   Product: '<S11>/Divide6'
  //   Sqrt: '<S13>/Sqrt4'
  //   Sum: '<S11>/Add'

  if (1.0 / std::sqrt((rtU.w[0] * rtU.w[0] + rtU.w[1] * rtU.w[1]) + rtU.w[2] *
                      rtU.w[2]) * ((rtb_Gain[0] + rtb_Gain[1]) + rtb_Gain[2]) /
      rtb_Product7 > 0.0) {
    i = 1;
  } else {
    i = -1;
  }

  // End of Switch: '<S11>/Switch'

  // Switch: '<S12>/Switch1' incorporates:
  //   Constant: '<S12>/Constant10'
  //   Constant: '<S12>/Constant9'
  //   Inport: '<Root>/angularvelocity'

  if (rtU.w[2] >= 0.0) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  // End of Switch: '<S12>/Switch1'

  // Product: '<S3>/Product7' incorporates:
  //   Gain: '<S3>/Gain10'
  //   Gain: '<S3>/Gain11'
  //   Product: '<S3>/Product8'
  //   Sqrt: '<S8>/Sqrt4'
  //   Sum: '<S3>/Sum7'
  //   Sum: '<S7>/Diff'
  //   UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtb_Product7 = ((rtb_Gain8_idx_1 - rtDW.UD_DSTATE_k) * 1.5890498768155689E-6 +
                  5.02469927340896E-10 * rtb_TrigonometricFunction5) *
    static_cast<real_T>((tmp_0 * i)) / rtb_Product7;

  // Sum: '<S2>/Sum10' incorporates:
  //   Constant: '<S2>/Identity matrix'

  for (i = 0; i < 9; i++) {
    rtb_VectorConcatenate_0[i] = rtb_VectorConcatenate[i] +
      rtConstPStar.Identitymatrix_Value[i];
  }

  // End of Sum: '<S2>/Sum10'
  for (i = 0; i < 3; i++) {
    // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' incorporates:
    //   Gain: '<S2>/Gain 8'
    //   Gain: '<S2>/Gain 9'
    //   Gain: '<S2>/Id inverse'
    //   Product: '<S2>/Product1'
    //   Sum: '<S2>/Sum8'
    //   Sum: '<S5>/Diff'
    //   UnitDelay: '<S5>/UD'
    //
    //  Block description for '<S5>/Diff':
    //
    //   Add in CPU
    //
    //  Block description for '<S5>/UD':
    //
    //   Store in Global RAM

    rtDW.DiscreteTimeIntegrator_DSTATE[i] += ((0.0 - (rtb_TSamp[i] -
      rtDW.UD_DSTATE[i])) - ((476.1904761904762 * -rtb_Product1_idx_0 * 0.004 *
      rtb_VectorConcatenate_0[i] + 476.1904761904762 * -rtb_Product1_idx_1 *
      0.004 * rtb_VectorConcatenate_0[i + 3]) + 476.1904761904762 *
      -rtb_Product1 * 0.004 * rtb_VectorConcatenate_0[i + 6])) * 0.01;

    // Update for UnitDelay: '<S5>/UD' incorporates:
    //   Product: '<S2>/Product1'
    //   Sum: '<S5>/Diff'
    //
    //  Block description for '<S5>/UD':
    //
    //   Store in Global RAM
    //
    //  Block description for '<S5>/Diff':
    //
    //   Add in CPU

    rtDW.UD_DSTATE[i] = rtb_TSamp[i];
  }

  // Update for UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_k = rtb_Gain8_idx_1;

  // Saturate: '<S2>/Saturation3'
  if (rtb_Gain8_idx_2 > 0.19909250856560792) {
    // Outport: '<Root>/detumble'
    rtY.detumble[0] = 0.19909250856560792;
  } else if (rtb_Gain8_idx_2 < -0.19909250856560792) {
    // Outport: '<Root>/detumble'
    rtY.detumble[0] = -0.19909250856560792;
  } else {
    // Outport: '<Root>/detumble'
    rtY.detumble[0] = rtb_Gain8_idx_2;
  }

  // Saturate: '<S2>/Saturation4'
  if (rtb_Saturation3 > 0.19909250856560792) {
    // Outport: '<Root>/detumble'
    rtY.detumble[1] = 0.19909250856560792;
  } else if (rtb_Saturation3 < -0.19909250856560792) {
    // Outport: '<Root>/detumble'
    rtY.detumble[1] = -0.19909250856560792;
  } else {
    // Outport: '<Root>/detumble'
    rtY.detumble[1] = rtb_Saturation3;
  }

  // Saturate: '<S2>/Saturation5'
  if (rtb_Gain8_idx_0 > 0.19909250856560792) {
    // Outport: '<Root>/detumble'
    rtY.detumble[2] = 0.19909250856560792;
  } else if (rtb_Gain8_idx_0 < -0.19909250856560792) {
    // Outport: '<Root>/detumble'
    rtY.detumble[2] = -0.19909250856560792;
  } else {
    // Outport: '<Root>/detumble'
    rtY.detumble[2] = rtb_Gain8_idx_0;
  }

  // Outport: '<Root>/point' incorporates:
  //   Saturate: '<S3>/Saturation3'
  //   Saturate: '<S3>/Saturation4'

  rtY.point[0] = 0.0;
  rtY.point[1] = 0.0;

  // Saturate: '<S3>/Saturation5' incorporates:
  //   Gain: '<S3>/Gain'

  rtb_Gain8_idx_2 = 46.300583387350684 * rtb_Product7;
  if (rtb_Gain8_idx_2 > 0.19909250856560792) {
    // Outport: '<Root>/point'
    rtY.point[2] = 0.19909250856560792;
  } else if (rtb_Gain8_idx_2 < -0.19909250856560792) {
    // Outport: '<Root>/point'
    rtY.point[2] = -0.19909250856560792;
  } else {
    // Outport: '<Root>/point'
    rtY.point[2] = rtb_Gain8_idx_2;
  }

  // End of Saturate: '<S3>/Saturation5'
  // End of Outputs for SubSystem: '<Root>/StarshotACS'
}

// Model initialize function
void StarshotACSModelClass::initialize()
{
  // SystemInitialize for Atomic SubSystem: '<Root>/StarshotACS'
  // InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_DSTATE[0] = 0.0;
  rtDW.DiscreteTimeIntegrator_DSTATE[1] = 0.0;
  rtDW.DiscreteTimeIntegrator_DSTATE[2] = 1.0;

  // End of SystemInitialize for SubSystem: '<Root>/StarshotACS'
}

// Constructor
StarshotACSModelClass::StarshotACSModelClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
StarshotACSModelClass::~StarshotACSModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL * StarshotACSModelClass::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
