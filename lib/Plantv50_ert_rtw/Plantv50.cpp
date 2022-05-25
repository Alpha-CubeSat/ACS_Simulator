//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Plantv50.cpp
//
// Code generated for Simulink model 'Plantv50'.
//
// Model version                  : 10.27
// Simulink Coder version         : 9.6 (R2021b) 14-May-2021
// C/C++ source code generated on : Fri Apr 29 03:44:30 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "Plantv50.h"
#define NumBitsPerChar                 8U

extern real_T rt_atan2d_snf(real_T u0, real_T u1);

//===========*
//  Constants *
// ===========
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

//
//  UNUSED_PARAMETER(x)
//    Used to specify that a function parameter (argument) is required but not
//    accessed by the function body.

#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      // do nothing
#else

//
//  This is the semi-ANSI standard way of indicating that an
//  unused function parameter is required.

#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern "C" {
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}
  extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetNaN(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T nan = 0.0;
    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF = { { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

extern "C" {
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  static void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    boolean_T result = (boolean_T) 0;
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.fltVal = value;
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}
  extern "C"
{
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T inf = 0.0;
    if (bitsPerReal == 32U) {
      inf = rtGetInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
    }

    return inf;
  }

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T minf = 0.0;
    if (bitsPerReal == 32U) {
      minf = rtGetMinusInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
    }

    return minf;
  }

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}

//
// Output and update for atomic system:
//    '<S1>/qtoQ'
//    '<S1>/qtoQ1'
//
void Plantv50ModelClass::qtoQ(const real_T rtu_q[4], real_T rty_Q[9])
{
  real_T rtu_q_0[9];
  real_T tmp[9];
  real_T rtb_Sum1;
  real_T rtu_q_1;
  int32_T i;

  // Product: '<S7>/qTq' incorporates:
  //   Math: '<S7>/T2'

  rtu_q_1 = 0.0;
  for (i = 0; i < 3; i++) {
    rtu_q_1 += rtu_q[i] * rtu_q[i];

    // Product: '<S7>/qqT' incorporates:
    //   Math: '<S7>/T1'
    //   Math: '<S7>/T2'

    rtu_q_0[3 * i] = rtu_q[0] * rtu_q[i];
    rtu_q_0[3 * i + 1] = rtu_q[1] * rtu_q[i];
    rtu_q_0[3 * i + 2] = rtu_q[2] * rtu_q[i];
  }

  // Sum: '<S7>/Sum1' incorporates:
  //   Product: '<S7>/Product1'
  //   Product: '<S7>/qTq'

  rtb_Sum1 = rtu_q[3] * rtu_q[3] - rtu_q_1;

  // Reshape: '<S13>/3x3' incorporates:
  //   Constant: '<S13>/diag 0 '
  //   Gain: '<S13>/Gain'
  //   Gain: '<S13>/Gain1'
  //   Gain: '<S13>/Gain2'

  tmp[0] = 0.0;
  tmp[1] = rtu_q[2];
  tmp[2] = -rtu_q[1];
  tmp[3] = -rtu_q[2];
  tmp[4] = 0.0;
  tmp[5] = rtu_q[0];
  tmp[6] = rtu_q[1];
  tmp[7] = -rtu_q[0];
  tmp[8] = 0.0;

  // Product: '<S7>/Product'
  rtu_q_1 = rtu_q[3];

  // Sum: '<S7>/Sum8' incorporates:
  //   Gain: '<S7>/Gain1'
  //   Gain: '<S7>/Gain2'
  //   Gain: '<S7>/Matrix Gain'
  //   Product: '<S7>/Product'

  for (i = 0; i < 9; i++) {
    rty_Q[i] = (2.0 * rtu_q_0[i] - tmp[i] * rtu_q_1 * 2.0) + rtConstPPlant.pooled5[i]
      * rtb_Sum1;
  }

  // End of Sum: '<S7>/Sum8'
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T u0_0;
    int32_T u1_0;
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

// Model step function
void Plantv50ModelClass::step(float step_size)
{
  real_T rtb_Sum8[9];
  real_T rtb_Elementproduct[6];
  real_T rtb_Product[4];
  real_T rtb_Product_j[3];
  real_T rtb_Sum[3];
  real_T rtb_DiscreteTimeIntegrator1;
  real_T rtb_DiscreteTimeIntegrator2;
  real_T rtb_DiscreteTimeIntegrator3;
  real_T rtb_DiscreteTimeIntegrator4;
  real_T rtb_DiscreteTimeIntegrator5;
  real_T rtb_DiscreteTimeIntegrator6;
  real_T rtb_Sum6;
  real_T rtb_TmpSignalConversionAtMatr_0;
  real_T rtb_TmpSignalConversionAtMatr_1;
  real_T rtb_qd3;
  int32_T i;

  // Outputs for Atomic SubSystem: '<Root>/Plantv5'
  // Gain: '<S1>/Gain' incorporates:
  //   Inport: '<Root>/current'

   rtb_Product_j[0] = Area * loops * MagtorqAmpFac * rtU.current[0];
  rtb_Product_j[1] = Area * loops * MagtorqAmpFac * rtU.current[1];
  rtb_Product_j[2] = Area * loops * MagtorqAmpFac * rtU.current[2];


  // DotProduct: '<S11>/Dot Product' incorporates:
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator1'

  rtb_TmpSignalConversionAtMatr_1 = ((rtDW.DiscreteTimeIntegrator1_DSTATE[0] *
    rtDW.DiscreteTimeIntegrator1_DSTATE[0] +
    rtDW.DiscreteTimeIntegrator1_DSTATE[1] *
    rtDW.DiscreteTimeIntegrator1_DSTATE[1]) +
    rtDW.DiscreteTimeIntegrator1_DSTATE[2] *
    rtDW.DiscreteTimeIntegrator1_DSTATE[2]) +
    rtDW.DiscreteTimeIntegrator1_DSTATE[3] *
    rtDW.DiscreteTimeIntegrator1_DSTATE[3];

  // Math: '<S11>/Math Function' incorporates:
  //   DotProduct: '<S11>/Dot Product'
  //
  //  About '<S11>/Math Function':
  //   Operator: sqrt

  if (rtb_TmpSignalConversionAtMatr_1 < 0.0) {
    rtb_TmpSignalConversionAtMatr_1 = -std::sqrt(std::abs
      (rtb_TmpSignalConversionAtMatr_1));
  } else {
    rtb_TmpSignalConversionAtMatr_1 = std::sqrt(rtb_TmpSignalConversionAtMatr_1);
  }

  // End of Math: '<S11>/Math Function'

  // Product: '<S11>/Product' incorporates:
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator1'

  rtb_Product[0] = rtDW.DiscreteTimeIntegrator1_DSTATE[0] /
    rtb_TmpSignalConversionAtMatr_1;
  rtb_Product[1] = rtDW.DiscreteTimeIntegrator1_DSTATE[1] /
    rtb_TmpSignalConversionAtMatr_1;
  rtb_Product[2] = rtDW.DiscreteTimeIntegrator1_DSTATE[2] /
    rtb_TmpSignalConversionAtMatr_1;
  rtb_Product[3] = rtDW.DiscreteTimeIntegrator1_DSTATE[3] /
    rtb_TmpSignalConversionAtMatr_1;

  // Outputs for Atomic SubSystem: '<S1>/qtoQ1'
  qtoQ(rtb_Product, rtb_Sum8);

  // End of Outputs for SubSystem: '<S1>/qtoQ1'

  // Outputs for Atomic SubSystem: '<S1>/Tranlational Dynamics'
  // DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
  rtb_DiscreteTimeIntegrator1 = rtDW.DiscreteTimeIntegrator1_DSTA_ls;

  // DiscreteIntegrator: '<S6>/Discrete-Time Integrator2'
  rtb_DiscreteTimeIntegrator2 = rtDW.DiscreteTimeIntegrator2_DSTAT_e;

  // DiscreteIntegrator: '<S6>/Discrete-Time Integrator3'
  rtb_DiscreteTimeIntegrator3 = rtDW.DiscreteTimeIntegrator3_DSTATE;

  // DiscreteIntegrator: '<S6>/Discrete-Time Integrator4'
  rtb_DiscreteTimeIntegrator4 = rtDW.DiscreteTimeIntegrator4_DSTATE;

  // DiscreteIntegrator: '<S6>/Discrete-Time Integrator5'
  rtb_DiscreteTimeIntegrator5 = rtDW.DiscreteTimeIntegrator5_DSTATE;

  // DiscreteIntegrator: '<S6>/Discrete-Time Integrator6'
  rtb_DiscreteTimeIntegrator6 = rtDW.DiscreteTimeIntegrator6_DSTATE;

  // Sqrt: '<S6>/Sqrt' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator4'
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator5'
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator6'
  //   DotProduct: '<S6>/Dot Product'
  //   DotProduct: '<S6>/Dot Product1'
  //   DotProduct: '<S6>/Dot Product2'
  //   Sum: '<S6>/Sum'

  rtb_TmpSignalConversionAtMatr_1 = std::sqrt
    ((rtDW.DiscreteTimeIntegrator4_DSTATE * rtDW.DiscreteTimeIntegrator4_DSTATE
      + rtDW.DiscreteTimeIntegrator5_DSTATE *
      rtDW.DiscreteTimeIntegrator5_DSTATE) + rtDW.DiscreteTimeIntegrator6_DSTATE
     * rtDW.DiscreteTimeIntegrator6_DSTATE);

  // Product: '<S6>/Divide' incorporates:
  //   Math: '<S6>/Math Function'

  rtb_Sum6 = 1.0 / (rtb_TmpSignalConversionAtMatr_1 *
                    rtb_TmpSignalConversionAtMatr_1) * -3.342008836E+14;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator5'
  //   DotProduct: '<S6>/Dot Product4'
  //   Product: '<S6>/Divide1'

  rtDW.DiscreteTimeIntegrator1_DSTA_ls += 1.0 / rtb_TmpSignalConversionAtMatr_1 *
    rtDW.DiscreteTimeIntegrator5_DSTATE * rtb_Sum6 * 0.001;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator2' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator4'
  //   DotProduct: '<S6>/Dot Product5'
  //   Product: '<S6>/Divide1'

  rtDW.DiscreteTimeIntegrator2_DSTAT_e += 1.0 / rtb_TmpSignalConversionAtMatr_1 *
    rtDW.DiscreteTimeIntegrator4_DSTATE * rtb_Sum6 * 0.001;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator3' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator6'
  //   DotProduct: '<S6>/Dot Product3'
  //   Product: '<S6>/Divide1'

  rtDW.DiscreteTimeIntegrator3_DSTATE += 1.0 / rtb_TmpSignalConversionAtMatr_1 *
    rtDW.DiscreteTimeIntegrator6_DSTATE * rtb_Sum6 * 0.001;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator4'
  rtDW.DiscreteTimeIntegrator4_DSTATE += 0.001 * rtb_DiscreteTimeIntegrator2;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator5'
  rtDW.DiscreteTimeIntegrator5_DSTATE += 0.001 * rtb_DiscreteTimeIntegrator1;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator6'
  rtDW.DiscreteTimeIntegrator6_DSTATE += 0.001 * rtb_DiscreteTimeIntegrator3;

  // End of Outputs for SubSystem: '<S1>/Tranlational Dynamics'

  // Outputs for Atomic SubSystem: '<S1>/Magnetic Field Model'
  // Product: '<S4>/Divide2' incorporates:
  //   Constant: '<S4>/Constant1'
  //   Math: '<S4>/Math Function5'
  //   Math: '<S4>/Math Function6'
  //   Math: '<S4>/Math Function7'
  //   Sqrt: '<S4>/Sqrt3'
  //   Sum: '<S4>/Sum4'

  rtb_Sum6 = 6.378E+6 / std::sqrt((rtb_DiscreteTimeIntegrator4 *
    rtb_DiscreteTimeIntegrator4 + rtb_DiscreteTimeIntegrator5 *
    rtb_DiscreteTimeIntegrator5) + rtb_DiscreteTimeIntegrator6 *
    rtb_DiscreteTimeIntegrator6);

  // Gain: '<S4>/Gain4' incorporates:
  //   Math: '<S4>/Math Function8'
  //   Product: '<S4>/Product'

  rtb_Sum6 = rtb_Sum6 * rtb_Sum6 * rtb_Sum6 * -3.12E-5;

  // Sum: '<S4>/Sum5' incorporates:
  //   Constant: '<S4>/Constant8'
  //   Math: '<S4>/Math Function3'
  //   Math: '<S4>/Math Function4'
  //   Sqrt: '<S4>/Sqrt2'
  //   Sum: '<S4>/Sum3'
  //   Trigonometry: '<S4>/Trigonometric Function3'

  rtb_DiscreteTimeIntegrator3 = 1.5707963267948966 - rt_atan2d_snf
    (rtb_DiscreteTimeIntegrator6, std::sqrt(rtb_DiscreteTimeIntegrator4 *
      rtb_DiscreteTimeIntegrator4 + rtb_DiscreteTimeIntegrator5 *
      rtb_DiscreteTimeIntegrator5));

  // Product: '<S4>/Divide3' incorporates:
  //   Gain: '<S4>/Gain5'
  //   Trigonometry: '<S4>/Trigonometric Function2'

  rtb_TmpSignalConversionAtMatr_1 = 2.0 * rtb_Sum6 * std::cos
    (rtb_DiscreteTimeIntegrator3);

  // Product: '<S4>/Divide4' incorporates:
  //   Trigonometry: '<S4>/Trigonometric Function4'

  rtb_DiscreteTimeIntegrator1 = rtb_Sum6 * std::sin(rtb_DiscreteTimeIntegrator3);

  // Outputs for Atomic SubSystem: '<S4>/Dipole->ECI'
  // Trigonometry: '<S10>/Trigonometric Function8' incorporates:
  //   Gain: '<S4>/Gain6'
  //   Trigonometry: '<S10>/Trigonometric Function5'

  rtb_DiscreteTimeIntegrator2 = std::cos(-rtb_DiscreteTimeIntegrator3);

  // Trigonometry: '<S10>/Trigonometric Function7' incorporates:
  //   Gain: '<S4>/Gain6'
  //   Trigonometry: '<S10>/Trigonometric Function6'

  rtb_qd3 = std::sin(-rtb_DiscreteTimeIntegrator3);

  // Sum: '<S10>/Sum6' incorporates:
  //   Product: '<S10>/Product3'
  //   Product: '<S10>/Product4'
  //   Trigonometry: '<S10>/Trigonometric Function7'
  //   Trigonometry: '<S10>/Trigonometric Function8'

  rtb_Sum6 = rtb_DiscreteTimeIntegrator2 * rtb_DiscreteTimeIntegrator1 + rtb_qd3
    * rtb_TmpSignalConversionAtMatr_1;

  // Trigonometry: '<S10>/Trigonometric Function'
  rtb_DiscreteTimeIntegrator3 = rt_atan2d_snf(rtb_DiscreteTimeIntegrator5,
    rtb_DiscreteTimeIntegrator4);

  // SignalConversion generated from: '<S1>/Matrix Multiply' incorporates:
  //   Product: '<S10>/Divide'
  //   Product: '<S10>/Divide1'
  //   Product: '<S10>/Product1'
  //   Product: '<S10>/Product2'
  //   Sum: '<S10>/Sum5'
  //   Trigonometry: '<S10>/Trigonometric Function1'
  //   Trigonometry: '<S10>/Trigonometric Function2'

  rtb_TmpSignalConversionAtMatr_0 = rtb_Sum6 * std::cos
    (rtb_DiscreteTimeIntegrator3);
  rtb_DiscreteTimeIntegrator3 = rtb_Sum6 * std::sin(rtb_DiscreteTimeIntegrator3);
  rtb_TmpSignalConversionAtMatr_1 = rtb_DiscreteTimeIntegrator2 *
    rtb_TmpSignalConversionAtMatr_1 - rtb_qd3 * rtb_DiscreteTimeIntegrator1;

  // End of Outputs for SubSystem: '<S4>/Dipole->ECI'
  // End of Outputs for SubSystem: '<S1>/Magnetic Field Model'

  // Product: '<S1>/Matrix Multiply1' incorporates:
  //   Sum: '<S7>/Sum8'

  for (i = 0; i < 3; i++) {
    rtb_Sum[i] = (rtb_Sum8[i + 3] * rtb_DiscreteTimeIntegrator3 + rtb_Sum8[i] *
                  rtb_TmpSignalConversionAtMatr_0) + rtb_Sum8[i + 6] *
      rtb_TmpSignalConversionAtMatr_1;
  }

  // End of Product: '<S1>/Matrix Multiply1'

  // Product: '<S2>/Element product'
  rtb_Elementproduct[0] = rtb_Product_j[1] * rtb_Sum[2];
  rtb_Elementproduct[1] = rtb_Sum[0] * rtb_Product_j[2];
  rtb_Elementproduct[2] = rtb_Product_j[0] * rtb_Sum[1];
  rtb_Elementproduct[3] = rtb_Sum[1] * rtb_Product_j[2];
  rtb_Elementproduct[4] = rtb_Product_j[0] * rtb_Sum[2];
  rtb_Elementproduct[5] = rtb_Sum[0] * rtb_Product_j[1];

  // Outport: '<Root>/angular velocity' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

  rtY.w[0] = rtDW.DiscreteTimeIntegrator_DSTATE;
  rtY.w[1] = rtDW.DiscreteTimeIntegrator1_DSTAT_l;
  rtY.w[2] = rtDW.DiscreteTimeIntegrator2_DSTATE;

  // Outputs for Atomic SubSystem: '<S1>/Dynamics'
  for (i = 0; i < 3; i++) {
    // Sum: '<S2>/Add3'
    rtb_Sum[i] = rtb_Elementproduct[i] - rtb_Elementproduct[i + 3];

    // Product: '<S3>/Product' incorporates:
    //   Constant: '<S3>/Constant'
    //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
    //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

    rtb_Product_j[i] = (rtConstPPlant.Constant_Value[i + 3] *
                        rtDW.DiscreteTimeIntegrator1_DSTAT_l +
                        rtConstPPlant.Constant_Value[i] *
                        rtDW.DiscreteTimeIntegrator_DSTATE) +
      rtConstPPlant.Constant_Value[i + 6] * rtDW.DiscreteTimeIntegrator2_DSTATE;
  }

  // Sum: '<S3>/Sum' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  //   Product: '<S9>/Element product'
  //   Sum: '<S9>/Add3'

  rtb_DiscreteTimeIntegrator1 = rtb_Sum[0] -
    (rtDW.DiscreteTimeIntegrator1_DSTAT_l * rtb_Product_j[2] -
     rtDW.DiscreteTimeIntegrator2_DSTATE * rtb_Product_j[1]);
  rtb_DiscreteTimeIntegrator2 = rtb_Sum[1] -
    (rtDW.DiscreteTimeIntegrator2_DSTATE * rtb_Product_j[0] -
     rtDW.DiscreteTimeIntegrator_DSTATE * rtb_Product_j[2]);
  rtb_qd3 = rtb_Sum[2] - (rtDW.DiscreteTimeIntegrator_DSTATE * rtb_Product_j[1]
    - rtDW.DiscreteTimeIntegrator1_DSTAT_l * rtb_Product_j[0]);

  // Product: '<S3>/Product1' incorporates:
  //   Constant: '<S3>/Constant1'

  for (i = 0; i < 3; i++) {
    rtb_Product_j[i] = (rtConstPPlant.Constant1_Value[i + 3] *
                        rtb_DiscreteTimeIntegrator2 + rtConstPPlant.Constant1_Value[i]
                        * rtb_DiscreteTimeIntegrator1) +
      rtConstPPlant.Constant1_Value[i + 6] * rtb_qd3;
  }

  // End of Product: '<S3>/Product1'
  // End of Outputs for SubSystem: '<S1>/Dynamics'

  // Outputs for Atomic SubSystem: '<S1>/qtoQ'
  qtoQ(rtb_Product, rtb_Sum8);

  // End of Outputs for SubSystem: '<S1>/qtoQ'

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  //   Fcn: '<S12>/qd1'
  //   Fcn: '<S12>/qd2'
  //   Fcn: '<S12>/qd3'
  //   Fcn: '<S12>/qd4'

  rtb_DiscreteTimeIntegrator1 = rtDW.DiscreteTimeIntegrator1_DSTATE[0];
  rtb_DiscreteTimeIntegrator2 = rtDW.DiscreteTimeIntegrator1_DSTATE[1];
  rtb_qd3 = rtDW.DiscreteTimeIntegrator1_DSTATE[2];
  rtb_Sum6 = rtDW.DiscreteTimeIntegrator1_DSTATE[3];
  rtDW.DiscreteTimeIntegrator1_DSTATE[0] = ((rtDW.DiscreteTimeIntegrator_DSTATE *
    rtb_Product[3] - rtDW.DiscreteTimeIntegrator1_DSTAT_l * rtb_Product[2]) +
    rtDW.DiscreteTimeIntegrator2_DSTATE * rtb_Product[1]) / 2.0 * 0.001 +
    rtb_DiscreteTimeIntegrator1;
  rtDW.DiscreteTimeIntegrator1_DSTATE[1] = ((rtDW.DiscreteTimeIntegrator_DSTATE *
    rtb_Product[2] + rtDW.DiscreteTimeIntegrator1_DSTAT_l * rtb_Product[3]) -
    rtDW.DiscreteTimeIntegrator2_DSTATE * rtb_Product[0]) / 2.0 * 0.001 +
    rtb_DiscreteTimeIntegrator2;
  rtDW.DiscreteTimeIntegrator1_DSTATE[2] = ((-rtb_Product[1] *
    rtDW.DiscreteTimeIntegrator_DSTATE + rtDW.DiscreteTimeIntegrator1_DSTAT_l *
    rtb_Product[0]) + rtDW.DiscreteTimeIntegrator2_DSTATE * rtb_Product[3]) /
    2.0 * 0.001 + rtb_qd3;
  rtDW.DiscreteTimeIntegrator1_DSTATE[3] = ((-rtb_Product[0] *
    rtDW.DiscreteTimeIntegrator_DSTATE - rtDW.DiscreteTimeIntegrator1_DSTAT_l *
    rtb_Product[1]) - rtDW.DiscreteTimeIntegrator2_DSTATE * rtb_Product[2]) /
    2.0 * 0.001 + rtb_Sum6;

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.001 * rtb_Product_j[0];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  rtDW.DiscreteTimeIntegrator1_DSTAT_l += 0.001 * rtb_Product_j[1];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.001 * rtb_Product_j[2];

  // End of Outputs for SubSystem: '<Root>/Plantv5'

  // Outport: '<Root>/magnetic field' incorporates:
  //   Product: '<S1>/Matrix Multiply'
  //   Sum: '<S7>/Sum8'

  for (i = 0; i < 3; i++) {
    // Outputs for Atomic SubSystem: '<Root>/Plantv5'
    rtY.magneticfield[i] = 0.0;
    rtY.magneticfield[i] += rtb_Sum8[i] * rtb_TmpSignalConversionAtMatr_0;
    rtY.magneticfield[i] += rtb_Sum8[i + 3] * rtb_DiscreteTimeIntegrator3;
    rtY.magneticfield[i] += rtb_Sum8[i + 6] * rtb_TmpSignalConversionAtMatr_1;

    // End of Outputs for SubSystem: '<Root>/Plantv5'
  }

  // End of Outport: '<Root>/magnetic field'

  // Outport: '<Root>/xyzposition'
  rtY.xyzposition[0] = rtb_DiscreteTimeIntegrator4;
  rtY.xyzposition[1] = rtb_DiscreteTimeIntegrator5;
  rtY.xyzposition[2] = rtb_DiscreteTimeIntegrator6;

  // Outport: '<Root>/quaternion'
  rtY.quaternion[0] = rtb_Product[0];
  rtY.quaternion[1] = rtb_Product[1];
  rtY.quaternion[2] = rtb_Product[2];
  rtY.quaternion[3] = rtb_Product[3];
}

// Model initialize function
void Plantv50ModelClass::initialize(float DiscreteTimeIntegrator_DSTATE, float DiscreteTimeIntegrator1_DSTAT_l, float DiscreteTimeIntegrator2_DSTATE, float quat0, float quat1, float quat2, float quat3, float altitude, float Inclination,float csarea,float num_loops,float ampFactor)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // SystemInitialize for Atomic SubSystem: '<Root>/Plantv5'
  // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator1' 
  rtDW.DiscreteTimeIntegrator1_DSTATE[0] = quat0;
  rtDW.DiscreteTimeIntegrator1_DSTATE[1] = quat1;
  rtDW.DiscreteTimeIntegrator1_DSTATE[2] = quat2;
  rtDW.DiscreteTimeIntegrator1_DSTATE[3] = quat3;

  // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE = DiscreteTimeIntegrator_DSTATE;

  // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  rtDW.DiscreteTimeIntegrator1_DSTAT_l = DiscreteTimeIntegrator1_DSTAT_l;

  // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_DSTATE = DiscreteTimeIntegrator2_DSTATE;

  // SystemInitialize for Atomic SubSystem: '<S1>/Tranlational Dynamics'
  // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' 
   rtDW.DiscreteTimeIntegrator1_DSTA_ls = (sqrt(398600/(6371+altitude))*1000)/sqrt(1+pow(tan(Inclination),2));//4748.3154149357551;

  // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator3'
  //below is starshot.IC>zdot
  rtDW.DiscreteTimeIntegrator3_DSTATE = tan(Inclination)*(sqrt(398600/(6371+altitude))*1000)/sqrt(1+pow(tan(Inclination),2));//5990.8830750989018;

  // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator4'
  rtDW.DiscreteTimeIntegrator4_DSTATE = 450000.0;
  Area = csarea;
  loops = num_loops;
  MagtorqAmpFac = ampFactor;
  // End of SystemInitialize for SubSystem: '<S1>/Tranlational Dynamics'
  // End of SystemInitialize for SubSystem: '<Root>/Plantv5'
}

// Constructor
Plantv50ModelClass::Plantv50ModelClass() :
  rtU(),
  rtY(),
  rtDW(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
Plantv50ModelClass::~Plantv50ModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL * Plantv50ModelClass::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
