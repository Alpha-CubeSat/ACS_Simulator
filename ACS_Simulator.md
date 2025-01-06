# Directory Structure

```
ACS_Simulator
|__ lib
|   |__ ACS_libs
|   |   |__ Plant_ert_rtw
|   |   |   |__ Plant.cpp
|   |   |   |   |__ Plant.cpp
|   |   |   |__ Plant.h
|   |   |   |   |__ Plant.h
|   |   |   |__ Plant_data.cpp
|   |   |   |   |__ Plant_data.cpp
|   |   |   |__ rtwtypes.h
|   |   |       |__ rtwtypes.h
|   |   |__ StarshotACS_ert_rtw
|   |       |__ StarshotACS.cpp
|   |       |   |__ StarshotACS.cpp
|   |       |__ StarshotACS.h
|   |       |   |__ StarshotACS.h
|   |       |__ StarshotACS_data.cpp
|   |       |   |__ StarshotACS_data.cpp
|   |       |__ rtwtypes.h
|   |           |__ rtwtypes.h
|   |__ ArduinoEigen
|   |   |__ ArduinoEigen.h
|   |   |   |__ ArduinoEigen.h
|   |   |__ ArduinoEigenDense.h
|   |   |   |__ ArduinoEigenDense.h
|   |   |__ ArduinoEigenSparse.h
|   |   |   |__ ArduinoEigenSparse.h
|   |__ ekf
|       |__ ekf.cpp
|       |   |__ ekf.cpp
|       |__ ekf.h
|       |   |__ ekf.h
|       |__ ekf.o
|           |__ ekf.o
|__ src
|   |__ DataLogging.cpp
|   |   |__ DataLogging.cpp
|   |__ DataLogging.hpp
|   |   |__ DataLogging.hpp
|   |__ MagneticFieldFilter.hpp
|   |   |__ MagneticFieldFilter.hpp
|   |__ main.cpp
|       |__ main.cpp
```

# File Contents

```lib\ACS_libs\Plant_ert_rtw\Plant.cpp
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Plant.cpp
//
// Code generated for Simulink model 'Plant'.
//
// Model version                  : 13.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Jul 26 17:41:01 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "Plant.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include <stddef.h>
#define NumBitsPerChar                 8U

// Exported block parameters
real_T Altitude = 400.0;               // Variable: Alt
                                          //  Referenced by:
                                          //    '<S7>/Discrete-Time Integrator1'
                                          //    '<S7>/Discrete-Time Integrator3'
                                          //    '<S7>/Discrete-Time Integrator4'
                                          //  Cubesat Altitude (km)

real_T I[9] = { 0.00195761450869, -5.836632382E-5, 2.27638093E-6,
  -5.836632382E-5, 0.00196346658902, 8.8920475E-7, 2.27638093E-6, 8.8920475E-7,
  0.00204697265884 } ;                 // Variable: I
                                          //  Referenced by:
                                          //    '<S3>/I'
                                          //    '<S3>/I^-1'


real_T inclination = 0.90058989402907408;// Variable: inclination
                                            //  Referenced by:
                                            //    '<S7>/Discrete-Time Integrator1'
                                            //    '<S7>/Discrete-Time Integrator3'
                                            //  ISS inclination

real_T m = 1.3;                        // Variable: m
                                          //  Referenced by: '<S7>/mass gain'
                                          //  Cubesat Mass

real_T q0[4] = { 0.49999999999999994, 0.5, -0.18301270189221924,
  0.6830127018922193 } ;               // Variable: q0
                                          //  Referenced by: '<S6>/Discrete-Time Integrator1'
                                          //  Init Quat (Default Euler Angle = pi/6*[1 2 3])


real_T wx = 0;                       // Variable: wx
                                          //  Referenced by: '<S1>/Discrete-Time Integrator'
                                          //  init angular vel -x

real_T wy = 0;                       // Variable: wy
                                          //  Referenced by: '<S1>/Discrete-Time Integrator1'
                                          //  init angular vel -y

real_T wz =1;                       // Variable: wz
                                          //  Referenced by: '<S1>/Discrete-Time Integrator2'
                                          //  init angular vel -z

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void rt_invd3x3_snf(const real_T u[9], real_T y[9]);

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

extern "C"
{
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}

//=========*
//  Asserts *
// =========
#ifndef utAssert
#if defined(DOASSERTS)
#if !defined(PRINT_ASSERTS)
#include <assert.h>
#define utAssert(exp)                  assert(exp)
#else
#include <stdio.h>

static void _assert(char_T *statement, char_T *file, int line)
{
  printf("%s in %s on line %d\n", statement, file, line);
}

#define utAssert(_EX)                  ((_EX) ? (void)0 : _assert(#_EX, __FILE__, __LINE__))
#endif

#else
#define utAssert(exp)                                            // do nothing
#endif
#endif

extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  [[maybe_unused]] static real_T rtGetNaN(void)
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
  [[maybe_unused]] static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF = { { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

extern "C"
{
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  [[maybe_unused]] static void rt_InitInfAndNaN(size_t realSize)
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
  [[maybe_unused]] static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  [[maybe_unused]] static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  [[maybe_unused]] static boolean_T rtIsNaN(real_T value)
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
  [[maybe_unused]] static boolean_T rtIsNaNF(real32_T value)
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
  [[maybe_unused]] static real_T rtGetInf(void)
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
  [[maybe_unused]] static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  [[maybe_unused]] static real_T rtGetMinusInf(void)
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
  [[maybe_unused]] static real32_T rtGetMinusInfF(void)
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
void Plant::qtoQ(const real_T rtu_q[4], real_T rty_Q[9])
{
  real_T rtu_q_0[9];
  real_T tmp[9];
  real_T rtb_Sum1;
  real_T rtu_q_1;

  // Product: '<S8>/qTq' incorporates:
  //   Math: '<S8>/T2'

  rtu_q_1 = 0.0;
  for (int32_T i = 0; i < 3; i++) {
    // Math: '<S8>/T2'
    rtb_Sum1 = rtu_q[i];
    rtu_q_1 += rtb_Sum1 * rtb_Sum1;

    // Product: '<S8>/qqT' incorporates:
    //   Math: '<S8>/T1'
    //   Math: '<S8>/T2'

    rtu_q_0[3 * i] = rtu_q[0] * rtu_q[i];
    rtu_q_0[3 * i + 1] = rtu_q[1] * rtu_q[i];
    rtu_q_0[3 * i + 2] = rtu_q[2] * rtu_q[i];
  }

  // Sum: '<S8>/Sum1' incorporates:
  //   Product: '<S8>/Product1'
  //   Product: '<S8>/qTq'

  rtb_Sum1 = rtu_q[3] * rtu_q[3] - rtu_q_1;

  // Reshape: '<S16>/3x3' incorporates:
  //   Constant: '<S16>/diag 0 '
  //   Gain: '<S16>/Gain'
  //   Gain: '<S16>/Gain1'
  //   Gain: '<S16>/Gain2'

  tmp[0] = 0.0;
  tmp[1] = rtu_q[2];
  tmp[2] = -rtu_q[1];
  tmp[3] = -rtu_q[2];
  tmp[4] = 0.0;
  tmp[5] = rtu_q[0];
  tmp[6] = rtu_q[1];
  tmp[7] = -rtu_q[0];
  tmp[8] = 0.0;

  // Product: '<S8>/Product'
  rtu_q_1 = rtu_q[3];

  // Sum: '<S8>/Sum8' incorporates:
  //   Gain: '<S8>/Gain1'
  //   Gain: '<S8>/Gain2'
  //   Gain: '<S8>/Matrix Gain'
  //   Product: '<S8>/Product'

  for (int32_T i = 0; i < 9; i++) {
    rty_Q[i] = (2.0 * rtu_q_0[i] - tmp[i] * rtu_q_1 * 2.0) + rtConstPplant.pooled6[i]
      * rtb_Sum1;
  }

  // End of Sum: '<S8>/Sum8'
}

[[maybe_unused]] real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
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

[[maybe_unused]] void rt_invd3x3_snf(const real_T u[9], real_T y[9])
{
  real_T x[9];
  real_T absx11;
  real_T absx21;
  real_T absx31;
  int32_T p1;
  int32_T p2;
  int32_T p3;
  std::memcpy(&x[0], &u[0], 9U * sizeof(real_T));
  p1 = 1;
  p2 = 3;
  p3 = 6;
  absx11 = std::abs(u[0]);
  absx21 = std::abs(u[1]);
  absx31 = std::abs(u[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 4;
    p2 = 0;
    x[0] = u[1];
    x[1] = u[0];
    x[3] = u[4];
    x[4] = u[3];
    x[6] = u[7];
    x[7] = u[6];
  } else if (absx31 > absx11) {
    p1 = 7;
    p3 = 0;
    x[2] = x[0];
    x[0] = u[2];
    x[5] = x[3];
    x[3] = u[5];
    x[8] = x[6];
    x[6] = u[8];
  }

  absx31 = x[1] / x[0];
  x[1] = absx31;
  absx11 = x[2] / x[0];
  x[2] = absx11;
  x[4] -= absx31 * x[3];
  x[5] -= absx11 * x[3];
  x[7] -= absx31 * x[6];
  x[8] -= absx11 * x[6];
  if (std::abs(x[5]) > std::abs(x[4])) {
    int32_T itmp;
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    x[1] = absx11;
    x[2] = absx31;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }

  absx31 = x[5] / x[4];
  x[8] -= absx31 * x[7];
  absx11 = (x[1] * absx31 - x[2]) / x[8];
  absx21 = -(x[7] * absx11 + x[1]) / x[4];
  y[p1 - 1] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  y[p1] = absx21;
  y[p1 + 1] = absx11;
  absx11 = -absx31 / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  y[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  y[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}

// Model step function
void Plant::step()
{
  real_T rtb_Sum8[9];
  real_T rtb_Elementproduct[6];
  real_T rtb_Product[4];
  real_T rtb_Normalization[3];
  real_T rtb_Product_j[3];
  real_T rtb_Sum[3];
  real_T acc;
  real_T rtb_DiscreteTimeIntegrator1;
  real_T rtb_DiscreteTimeIntegrator2;
  real_T rtb_DiscreteTimeIntegrator3;
  real_T rtb_Product_n;
  real_T rtb_Product_p_tmp;
  real_T rtb_qd2;
  real_T rtb_qd3;
  real_T rtb_qd4;
  int32_T i;
  int32_T k;

  // Outputs for Atomic SubSystem: '<Root>/Plant'
  // Gain: '<S1>/Gain' incorporates:
  //   Inport: '<Root>/current'

  rtb_Product_j[0] = 0.27 * rtU.current[0];
  rtb_Product_j[1] = 0.27 * rtU.current[1];
  rtb_Product_j[2] = 0.27 * rtU.current[2];

  // Outport: '<Root>/angular velocity' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

  rtY.angularvelocity[0] = rtDW.DiscreteTimeIntegrator_DSTATE;
  rtY.angularvelocity[1] = rtDW.DiscreteTimeIntegrator1_DSTATE;
  rtY.angularvelocity[2] = rtDW.DiscreteTimeIntegrator2_DSTATE;

  // Outputs for Atomic SubSystem: '<S1>/Quaternion Integration'
  // DotProduct: '<S14>/Dot Product' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'

  acc = ((rtDW.DiscreteTimeIntegrator1_DSTAT_m[0] *
          rtDW.DiscreteTimeIntegrator1_DSTAT_m[0] +
          rtDW.DiscreteTimeIntegrator1_DSTAT_m[1] *
          rtDW.DiscreteTimeIntegrator1_DSTAT_m[1]) +
         rtDW.DiscreteTimeIntegrator1_DSTAT_m[2] *
         rtDW.DiscreteTimeIntegrator1_DSTAT_m[2]) +
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[3] *
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[3];

  // Math: '<S14>/Math Function' incorporates:
  //   DotProduct: '<S14>/Dot Product'
  //
  //  About '<S14>/Math Function':
  //   Operator: sqrt

  if (acc < 0.0) {
    rtb_Product_n = -std::sqrt(std::abs(acc));
  } else {
    rtb_Product_n = std::sqrt(acc);
  }

  // End of Math: '<S14>/Math Function'

  // Product: '<S14>/Product' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'

  rtb_Product[0] = rtDW.DiscreteTimeIntegrator1_DSTAT_m[0] / rtb_Product_n;
  rtb_Product[1] = rtDW.DiscreteTimeIntegrator1_DSTAT_m[1] / rtb_Product_n;
  rtb_Product[2] = rtDW.DiscreteTimeIntegrator1_DSTAT_m[2] / rtb_Product_n;
  rtb_Product[3] = rtDW.DiscreteTimeIntegrator1_DSTAT_m[3] / rtb_Product_n;

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  //   Fcn: '<S15>/qd1'
  //   Fcn: '<S15>/qd2'
  //   Fcn: '<S15>/qd3'
  //   Fcn: '<S15>/qd4'

  rtb_Product_n = rtDW.DiscreteTimeIntegrator1_DSTAT_m[0];
  rtb_qd2 = rtDW.DiscreteTimeIntegrator1_DSTAT_m[1];
  rtb_qd3 = rtDW.DiscreteTimeIntegrator1_DSTAT_m[2];
  acc = rtDW.DiscreteTimeIntegrator1_DSTAT_m[3];
  rtDW.DiscreteTimeIntegrator1_DSTAT_m[0] = ((rtb_Product[3] *
    rtDW.DiscreteTimeIntegrator_DSTATE - rtb_Product[2] *
    rtDW.DiscreteTimeIntegrator1_DSTATE) + rtb_Product[1] *
    rtDW.DiscreteTimeIntegrator2_DSTATE) / 2.0 * 0.001 + rtb_Product_n;
  rtDW.DiscreteTimeIntegrator1_DSTAT_m[1] = ((rtb_Product[2] *
    rtDW.DiscreteTimeIntegrator_DSTATE + rtb_Product[3] *
    rtDW.DiscreteTimeIntegrator1_DSTATE) - rtb_Product[0] *
    rtDW.DiscreteTimeIntegrator2_DSTATE) / 2.0 * 0.001 + rtb_qd2;
  rtDW.DiscreteTimeIntegrator1_DSTAT_m[2] = ((-rtb_Product[1] *
    rtDW.DiscreteTimeIntegrator_DSTATE + rtb_Product[0] *
    rtDW.DiscreteTimeIntegrator1_DSTATE) + rtb_Product[3] *
    rtDW.DiscreteTimeIntegrator2_DSTATE) / 2.0 * 0.001 + rtb_qd3;
  rtDW.DiscreteTimeIntegrator1_DSTAT_m[3] = ((-rtb_Product[0] *
    rtDW.DiscreteTimeIntegrator_DSTATE - rtb_Product[1] *
    rtDW.DiscreteTimeIntegrator1_DSTATE) - rtb_Product[2] *
    rtDW.DiscreteTimeIntegrator2_DSTATE) / 2.0 * 0.001 + acc;

  // End of Outputs for SubSystem: '<S1>/Quaternion Integration'

  // Outputs for Atomic SubSystem: '<S1>/qtoQ1'
  qtoQ(rtb_Product, rtb_Sum8);

  // End of Outputs for SubSystem: '<S1>/qtoQ1'

  // Outputs for Atomic SubSystem: '<S1>/Tranlational Dynamics'
  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'
  rtb_DiscreteTimeIntegrator1 = rtDW.DiscreteTimeIntegrator1_DSTAT_l;

  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator2'
  rtb_DiscreteTimeIntegrator2 = rtDW.DiscreteTimeIntegrator2_DSTAT_e;

  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator3'
  rtb_DiscreteTimeIntegrator3 = rtDW.DiscreteTimeIntegrator3_DSTATE;

  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator4'
  rtb_Product_n = rtDW.DiscreteTimeIntegrator4_DSTATE;

  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator5'
  rtb_qd2 = rtDW.DiscreteTimeIntegrator5_DSTATE;

  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator6'
  rtb_qd3 = rtDW.DiscreteTimeIntegrator6_DSTATE;

  // Sqrt: '<S7>/Sqrt' incorporates:
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator4'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator5'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator6'
  //   DotProduct: '<S7>/Dot Product'
  //   DotProduct: '<S7>/Dot Product1'
  //   DotProduct: '<S7>/Dot Product2'
  //   Sum: '<S7>/Sum'

  rtb_qd4 = std::sqrt((rtDW.DiscreteTimeIntegrator4_DSTATE *
                       rtDW.DiscreteTimeIntegrator4_DSTATE +
                       rtDW.DiscreteTimeIntegrator5_DSTATE *
                       rtDW.DiscreteTimeIntegrator5_DSTATE) +
                      rtDW.DiscreteTimeIntegrator6_DSTATE *
                      rtDW.DiscreteTimeIntegrator6_DSTATE);

  // Product: '<S7>/Divide' incorporates:
  //   Constant: '<S7>/Constant'
  //   Gain: '<S7>/Gain1'
  //   Gain: '<S7>/mass gain'
  //   Math: '<S7>/Math Function'

  acc = 1.0 / (rtb_qd4 * rtb_qd4) * (m * -3.983324E+14);

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator5'
  //   DotProduct: '<S7>/Dot Product4'
  //   Product: '<S7>/Divide1'

  rtDW.DiscreteTimeIntegrator1_DSTAT_l += 1.0 / rtb_qd4 *
    rtDW.DiscreteTimeIntegrator5_DSTATE * acc * 0.001;

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator2' incorporates:
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator4'
  //   DotProduct: '<S7>/Dot Product5'
  //   Product: '<S7>/Divide1'

  rtDW.DiscreteTimeIntegrator2_DSTAT_e += 1.0 / rtb_qd4 *
    rtDW.DiscreteTimeIntegrator4_DSTATE * acc * 0.001;

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator3' incorporates:
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator6'
  //   DotProduct: '<S7>/Dot Product3'
  //   Product: '<S7>/Divide1'

  rtDW.DiscreteTimeIntegrator3_DSTATE += 1.0 / rtb_qd4 *
    rtDW.DiscreteTimeIntegrator6_DSTATE * acc * 0.001;

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator4'
  rtDW.DiscreteTimeIntegrator4_DSTATE += 0.001 * rtb_DiscreteTimeIntegrator2;

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator5'
  rtDW.DiscreteTimeIntegrator5_DSTATE += 0.001 * rtb_DiscreteTimeIntegrator1;

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator6'
  rtDW.DiscreteTimeIntegrator6_DSTATE += 0.001 * rtb_DiscreteTimeIntegrator3;

  // End of Outputs for SubSystem: '<S1>/Tranlational Dynamics'

  // Outputs for Atomic SubSystem: '<S1>/Magnetic Field Model'
  // Sum: '<S4>/Sum4' incorporates:
  //   Math: '<S4>/Math Function5'
  //   Math: '<S4>/Math Function6'
  //   Sum: '<S4>/Sum3'

  rtb_DiscreteTimeIntegrator1 = rtb_Product_n * rtb_Product_n + rtb_qd2 *
    rtb_qd2;

  // Product: '<S4>/Divide2' incorporates:
  //   Constant: '<S4>/Constant1'
  //   Math: '<S4>/Math Function7'
  //   Sqrt: '<S4>/Sqrt3'
  //   Sum: '<S4>/Sum4'

  acc = 6.371E+6 / std::sqrt(rtb_qd3 * rtb_qd3 + rtb_DiscreteTimeIntegrator1);

  // Gain: '<S4>/Gain4' incorporates:
  //   Math: '<S4>/Math Function8'
  //   Product: '<S4>/Product'

  acc = acc * acc * acc * -3.12E-5;

  // Sum: '<S4>/Sum5' incorporates:
  //   Constant: '<S4>/Constant8'
  //   Sqrt: '<S4>/Sqrt2'
  //   Trigonometry: '<S4>/Trigonometric Function3'

  rtb_qd4 = 1.5707963267948966 - rt_atan2d_snf(rtb_qd3, std::sqrt
    (rtb_DiscreteTimeIntegrator1));

  // Product: '<S4>/Divide3' incorporates:
  //   Gain: '<S4>/Gain5'
  //   Trigonometry: '<S4>/Trigonometric Function2'

  rtb_DiscreteTimeIntegrator1 = 2.0 * acc * std::cos(rtb_qd4);

  // Product: '<S4>/Divide4' incorporates:
  //   Trigonometry: '<S4>/Trigonometric Function4'

  rtb_DiscreteTimeIntegrator2 = acc * std::sin(rtb_qd4);

  // Outputs for Atomic SubSystem: '<S4>/Dipole->ECI'
  // Trigonometry: '<S13>/Trigonometric Function8' incorporates:
  //   Gain: '<S4>/Gain6'
  //   Trigonometry: '<S13>/Trigonometric Function5'

  rtb_DiscreteTimeIntegrator3 = std::cos(-rtb_qd4);

  // Trigonometry: '<S13>/Trigonometric Function7' incorporates:
  //   Gain: '<S4>/Gain6'
  //   Trigonometry: '<S13>/Trigonometric Function6'

  rtb_Product_p_tmp = std::sin(-rtb_qd4);

  // Sum: '<S13>/Sum6' incorporates:
  //   Product: '<S13>/Product3'
  //   Product: '<S13>/Product4'
  //   Trigonometry: '<S13>/Trigonometric Function7'
  //   Trigonometry: '<S13>/Trigonometric Function8'

  acc = rtb_DiscreteTimeIntegrator3 * rtb_DiscreteTimeIntegrator2 +
    rtb_Product_p_tmp * rtb_DiscreteTimeIntegrator1;

  // Trigonometry: '<S13>/Trigonometric Function'
  rtb_qd4 = rt_atan2d_snf(rtb_qd2, rtb_Product_n);

  // SignalConversion generated from: '<S1>/Matrix Multiply' incorporates:
  //   Product: '<S13>/Divide'
  //   Product: '<S13>/Divide1'
  //   Product: '<S13>/Product1'
  //   Product: '<S13>/Product2'
  //   Sum: '<S13>/Sum5'
  //   Trigonometry: '<S13>/Trigonometric Function1'
  //   Trigonometry: '<S13>/Trigonometric Function2'

  rtb_Normalization[0] = acc * std::cos(rtb_qd4);
  rtb_Normalization[1] = acc * std::sin(rtb_qd4);
  rtb_Normalization[2] = rtb_DiscreteTimeIntegrator3 *
    rtb_DiscreteTimeIntegrator1 - rtb_Product_p_tmp *
    rtb_DiscreteTimeIntegrator2;

  // End of Outputs for SubSystem: '<S4>/Dipole->ECI'
  // End of Outputs for SubSystem: '<S1>/Magnetic Field Model'

  // Product: '<S1>/Matrix Multiply1' incorporates:
  //   Sum: '<S8>/Sum8'

  acc = rtb_Normalization[1];
  rtb_DiscreteTimeIntegrator1 = rtb_Normalization[0];
  rtb_DiscreteTimeIntegrator2 = rtb_Normalization[2];
  for (i = 0; i < 3; i++) {
    rtb_Sum[i] = (rtb_Sum8[i + 3] * acc + rtb_Sum8[i] *
                  rtb_DiscreteTimeIntegrator1) + rtb_Sum8[i + 6] *
      rtb_DiscreteTimeIntegrator2;
  }

  // End of Product: '<S1>/Matrix Multiply1'

  // Product: '<S2>/Element product'
  rtb_Elementproduct[0] = rtb_Product_j[1] * rtb_Sum[2];
  rtb_Elementproduct[1] = rtb_Sum[0] * rtb_Product_j[2];
  rtb_Elementproduct[2] = rtb_Product_j[0] * rtb_Sum[1];
  rtb_Elementproduct[3] = rtb_Sum[1] * rtb_Product_j[2];
  rtb_Elementproduct[4] = rtb_Product_j[0] * rtb_Sum[2];
  rtb_Elementproduct[5] = rtb_Sum[0] * rtb_Product_j[1];

  // Outputs for Atomic SubSystem: '<S1>/Dynamics'
  for (i = 0; i < 3; i++) {
    // Sum: '<S2>/Add3'
    rtb_Sum[i] = rtb_Elementproduct[i] - rtb_Elementproduct[i + 3];

    // Product: '<S3>/Product' incorporates:
    //   Constant: '<S3>/I'
    //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
    //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'

    rtb_Product_j[i] = (I[i + 3] * rtDW.DiscreteTimeIntegrator1_DSTATE + I[i] *
                        rtDW.DiscreteTimeIntegrator_DSTATE) + I[i + 6] *
      rtDW.DiscreteTimeIntegrator2_DSTATE;
  }

  // Assertion: '<S11>/Assertion' incorporates:
  //   Constant: '<S3>/I^-1'
  //   Product: '<S12>/Product'
  //   Product: '<S12>/Product1'
  //   Product: '<S12>/Product2'
  //   Product: '<S12>/Product3'
  //   Product: '<S12>/Product4'
  //   Product: '<S12>/Product5'
  //   Sum: '<S12>/Sum'

  utAssert(((((I[0] * I[4] * I[8] - I[0] * I[5] * I[7]) - I[1] * I[3] * I[8]) +
             I[2] * I[3] * I[7]) + I[1] * I[5] * I[6]) - I[2] * I[4] * I[6] !=
           0.0);

  // Product: '<S11>/Product' incorporates:
  //   Constant: '<S3>/I^-1'
  //   Sum: '<S8>/Sum8'

  rt_invd3x3_snf(I, rtb_Sum8);

  // Sum: '<S3>/Sum' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  //   Product: '<S10>/Element product'
  //   Sum: '<S10>/Add3'

  acc = rtb_Sum[0] - (rtDW.DiscreteTimeIntegrator1_DSTATE * rtb_Product_j[2] -
                      rtDW.DiscreteTimeIntegrator2_DSTATE * rtb_Product_j[1]);
  rtb_DiscreteTimeIntegrator1 = rtb_Sum[1] -
    (rtDW.DiscreteTimeIntegrator2_DSTATE * rtb_Product_j[0] -
     rtDW.DiscreteTimeIntegrator_DSTATE * rtb_Product_j[2]);
  rtb_DiscreteTimeIntegrator2 = rtb_Sum[2] - (rtDW.DiscreteTimeIntegrator_DSTATE
    * rtb_Product_j[1] - rtDW.DiscreteTimeIntegrator1_DSTATE * rtb_Product_j[0]);

  // Product: '<S3>/Product1' incorporates:
  //   Sum: '<S8>/Sum8'

  for (i = 0; i < 3; i++) {
    rtb_Product_j[i] = (rtb_Sum8[i + 3] * rtb_DiscreteTimeIntegrator1 +
                        rtb_Sum8[i] * acc) + rtb_Sum8[i + 6] *
      rtb_DiscreteTimeIntegrator2;
  }

  // End of Product: '<S3>/Product1'
  // End of Outputs for SubSystem: '<S1>/Dynamics'

  // Outputs for Atomic SubSystem: '<S1>/qtoQ'
  qtoQ(rtb_Product, rtb_Sum8);

  // End of Outputs for SubSystem: '<S1>/qtoQ'

  // S-Function (sdsp2norm2): '<S5>/Normalization'
  i = 0;
  acc = 0.0;
  for (k = 0; k < 3; k++) {
    // Outport: '<Root>/magnetic field' incorporates:
    //   Product: '<S1>/Matrix Multiply'
    //   Sum: '<S8>/Sum8'

    rtY.magneticfield[k] = (rtb_Sum8[k + 3] * rtb_Normalization[1] + rtb_Sum8[k]
      * rtb_Normalization[0]) + rtb_Sum8[k + 6] * rtb_Normalization[2];

    // S-Function (sdsp2norm2): '<S5>/Normalization'
    acc += rtb_Normalization[i] * rtb_Normalization[i];
    i++;
  }

  // S-Function (sdsp2norm2): '<S5>/Normalization'
  acc = 1.0 / (std::sqrt(acc) + 1.0E-10);
  rtb_Normalization[0] *= acc;
  rtb_Normalization[1] *= acc;
  rtb_Normalization[2] *= acc;

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.001 * rtb_Product_j[0];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  rtDW.DiscreteTimeIntegrator1_DSTATE += 0.001 * rtb_Product_j[1];

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.001 * rtb_Product_j[2];

  // End of Outputs for SubSystem: '<Root>/Plant'

  // Outport: '<Root>/xyzposition'
  rtY.xyzposition[0] = rtb_Product_n;
  rtY.xyzposition[1] = rtb_qd2;
  rtY.xyzposition[2] = rtb_qd3;

  // Outport: '<Root>/quaternion'
  rtY.quaternion[0] = rtb_Product[0];
  rtY.quaternion[1] = rtb_Product[1];
  rtY.quaternion[2] = rtb_Product[2];
  rtY.quaternion[3] = rtb_Product[3];

  // Outputs for Atomic SubSystem: '<Root>/Plant'
  // DotProduct: '<S5>/Dot Product' incorporates:
  //   Constant: '<S5>/Constant'

  acc = (rtb_Normalization[0] * 0.0 + rtb_Normalization[1] * 0.0) +
    rtb_Normalization[2];

  // Saturate: '<S5>/Saturation3' incorporates:
  //   DotProduct: '<S5>/Dot Product'

  if (acc > 1.0) {
    acc = 1.0;
  } else if (acc < -1.0) {
    acc = -1.0;
  }

  // Outport: '<Root>/pt_error' incorporates:
  //   Gain: '<S5>/Multiply'
  //   Saturate: '<S5>/Saturation3'
  //   Trigonometry: '<S5>/Acos'

  rtY.pt_error = 57.295779513082323 * std::acos(acc);

  // End of Outputs for SubSystem: '<Root>/Plant'
}

// Model initialize function
void Plant::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    real_T DiscreteTimeIntegrator1_DSTAT_l;

    // SystemInitialize for Atomic SubSystem: '<Root>/Plant'
    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE = wx;

    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
    rtDW.DiscreteTimeIntegrator1_DSTATE = wy;

    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
    rtDW.DiscreteTimeIntegrator2_DSTATE = wz;

    // SystemInitialize for Atomic SubSystem: '<S1>/Quaternion Integration'
    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[0] = q0[0];
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[1] = q0[1];
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[2] = q0[2];
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[3] = q0[3];

    // End of SystemInitialize for SubSystem: '<S1>/Quaternion Integration'

    // SystemInitialize for Atomic SubSystem: '<S1>/Tranlational Dynamics'
    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator3'

    DiscreteTimeIntegrator1_DSTAT_l = std::sqrt(398600.0 / (Altitude + 6371.0)) *
      1000.0;
    rtDW.DiscreteTimeIntegrator1_DSTAT_l = DiscreteTimeIntegrator1_DSTAT_l * std::
      cos(inclination);

    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator3'
    rtDW.DiscreteTimeIntegrator3_DSTATE = DiscreteTimeIntegrator1_DSTAT_l * std::
      tan(inclination) * std::sin(inclination);

    // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator4'
    rtDW.DiscreteTimeIntegrator4_DSTATE = Altitude * 1000.0 + 6.371E+6;

    // End of SystemInitialize for SubSystem: '<S1>/Tranlational Dynamics'
    // End of SystemInitialize for SubSystem: '<Root>/Plant'
  }
}
// Model initialize function
void Plant::initialize(double Altitude_input, double I_input[9], double inclination_input, double m_input, double q0_input[4], double wx_input, double wy_input, double wz_input)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    real_T DiscreteTimeIntegrator1_DSTAT_l;

    Altitude = Altitude_input;

    for (int i = 0; i < 9; i++)
    {
      I[i] = I_input[i];
    }

    inclination = inclination_input;

    m = m_input;

    for (int i = 0; i < 4; i++)
    {
      q0[i] = q0_input[i];
    }

    wx = wx_input;
    wy = wy_input;
    wz = wz_input;

    // SystemInitialize for Atomic SubSystem: '<Root>/Plant'
    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE = wx_input;

    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
    rtDW.DiscreteTimeIntegrator1_DSTATE = wy_input;

    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator2'
    rtDW.DiscreteTimeIntegrator2_DSTATE = wz_input;

    // SystemInitialize for Atomic SubSystem: '<S1>/Quaternion Integration'
    // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator1'
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[0] = q0_input[0];
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[1] = q0_input[1];
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[2] = q0_input[2];
    rtDW.DiscreteTimeIntegrator1_DSTAT_m[3] = q0_input[3];

    // End of SystemInitialize for SubSystem: '<S1>/Quaternion Integration'

    // SystemInitialize for Atomic SubSystem: '<S1>/Tranlational Dynamics'
    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' incorporates:
    //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator3'

    DiscreteTimeIntegrator1_DSTAT_l = std::sqrt(398600.0 / (Altitude_input + 6371.0)) *
                                      1000.0;
    rtDW.DiscreteTimeIntegrator1_DSTAT_l = DiscreteTimeIntegrator1_DSTAT_l * std::cos(inclination_input);

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator3'
    rtDW.DiscreteTimeIntegrator3_DSTATE = DiscreteTimeIntegrator1_DSTAT_l * std::tan(inclination_input) * std::sin(inclination_input);

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator4'
    rtDW.DiscreteTimeIntegrator4_DSTATE = Altitude_input * 1000.0 + 6.371E+6;

    // End of SystemInitialize for SubSystem: '<S1>/Tranlational Dynamics'
    // End of SystemInitialize for SubSystem: '<Root>/Plant'
  }
}

// Constructor
Plant::Plant() :
  rtU(),
  rtY(),
  rtDW(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
Plant::~Plant()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODELplant * Plant::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
```

```lib\ACS_libs\Plant_ert_rtw\Plant.h
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Plant.h
//
// Code generated for Simulink model 'Plant'.
//
// Model version                  : 13.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Jul 26 17:41:01 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_Plant_h_
#define RTW_HEADER_Plant_h_
#include "rtwtypes.h"
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTMplant RT_MODELplant;

// Block signals and states (default storage) for system '<Root>'
struct DWplant
{
  real_T DiscreteTimeIntegrator1_DSTAT_m[4]; // '<S6>/Discrete-Time Integrator1'
  real_T Product_DWORK4[9];                  // '<S11>/Product'
  real_T DiscreteTimeIntegrator_DSTATE;      // '<S1>/Discrete-Time Integrator'
  real_T DiscreteTimeIntegrator1_DSTATE;     // '<S1>/Discrete-Time Integrator1'
  real_T DiscreteTimeIntegrator2_DSTATE;     // '<S1>/Discrete-Time Integrator2'
  real_T DiscreteTimeIntegrator1_DSTAT_l;    // '<S7>/Discrete-Time Integrator1'
  real_T DiscreteTimeIntegrator2_DSTAT_e;    // '<S7>/Discrete-Time Integrator2'
  real_T DiscreteTimeIntegrator3_DSTATE;     // '<S7>/Discrete-Time Integrator3'
  real_T DiscreteTimeIntegrator4_DSTATE;     // '<S7>/Discrete-Time Integrator4'
  real_T DiscreteTimeIntegrator5_DSTATE;     // '<S7>/Discrete-Time Integrator5'
  real_T DiscreteTimeIntegrator6_DSTATE;     // '<S7>/Discrete-Time Integrator6'
};

// Constant parameters (default storage)
struct ConstPplant
{
  // Pooled Parameter (Expression: eye(3,3))
  //  Referenced by:
  //    '<S8>/Matrix Gain'
  //    '<S9>/Matrix Gain'

  real_T pooled6[9];
};

// External inputs (root inport signals with default storage)
struct ExtUplant
{
  real_T current[3]; // '<Root>/current'
};

// External outputs (root outports fed by signals with default storage)
struct ExtYplant
{
  real_T angularvelocity[3]; // '<Root>/angular velocity'
  real_T magneticfield[3];   // '<Root>/magnetic field'
  real_T xyzposition[3];     // '<Root>/xyzposition'
  real_T quaternion[4];      // '<Root>/quaternion'
  real_T pt_error;           // '<Root>/pt_error'
};

// Real-time Model Data Structure
struct tag_RTMplant
{
  const char_T *volatile errorStatus;
};

// Constant parameters (default storage)
extern const ConstPplant rtConstPplant;

//
//  Exported Global Parameters
//
//  Note: Exported global parameters are tunable parameters with an exported
//  global storage class designation.  Code generation will declare the memory for
//  these parameters and exports their symbols.
//

extern real_T Altitude; // Variable: Alt
                        //  Referenced by:
                        //    '<S7>/Discrete-Time Integrator1'
                        //    '<S7>/Discrete-Time Integrator3'
                        //    '<S7>/Discrete-Time Integrator4'
                        //  Cubesat Altitude (km)

extern real_T I[9]; // Variable: I
                    //  Referenced by:
                    //    '<S3>/I'
                    //    '<S3>/I^-1'

extern real_T inclination; // Variable: inclination
                           //  Referenced by:
                           //    '<S7>/Discrete-Time Integrator1'
                           //    '<S7>/Discrete-Time Integrator3'
                           //  ISS inclination

extern real_T m; // Variable: m
                 //  Referenced by: '<S7>/mass gain'
                 //  Cubesat Mass

extern real_T q0[4]; // Variable: q0
                     //  Referenced by: '<S6>/Discrete-Time Integrator1'
                     //  Init Quat (Default Euler Angle = pi/6*[1 2 3])

extern real_T wx; // Variable: wx
                  //  Referenced by: '<S1>/Discrete-Time Integrator'
                  //  init angular vel -x

extern real_T wy; // Variable: wy
                  //  Referenced by: '<S1>/Discrete-Time Integrator1'
                  //  init angular vel -y

extern real_T wz; // Variable: wz
                  //  Referenced by: '<S1>/Discrete-Time Integrator2'
                  //  init angular vel -z

extern "C"
{
  [[maybe_unused]] static real_T rtGetNaN(void);
  [[maybe_unused]] static real32_T rtGetNaNF(void);
} // extern "C"

#define NOT_USING_NONFINITE_LITERALS 1

extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  [[maybe_unused]] static void rt_InitInfAndNaN(size_t realSize);
  [[maybe_unused]] static boolean_T rtIsInf(real_T value);
  [[maybe_unused]] static boolean_T rtIsInfF(real32_T value);
  [[maybe_unused]] static boolean_T rtIsNaN(real_T value);
  [[maybe_unused]] static boolean_T rtIsNaNF(real32_T value);
  struct BigEndianIEEEDouble
  {
    struct
    {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  };

  struct LittleEndianIEEEDouble
  {
    struct
    {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  };

  struct IEEESingle
  {
    union
    {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  };
} // extern "C"

extern "C"
{
  [[maybe_unused]] static real_T rtGetInf(void);
  [[maybe_unused]] static real32_T rtGetInfF(void);
  [[maybe_unused]] static real_T rtGetMinusInf(void);
  [[maybe_unused]] static real32_T rtGetMinusInfF(void);
} // extern "C"

// Class declaration for model Plant
class Plant
{
  // public data and function members
public:
  // Real-Time Model get method
  RT_MODELplant *getRTM();

  // External inputs
  ExtUplant rtU;

  // External outputs
  ExtYplant rtY;

  // model initialize function
  void initialize();
  void initialize(double Altitude_input, double I_input[9], double inclination_input, double m_input, double q0_input[4], double wx_input, double wy_input, double wz_input);
  // model step function
  void step();

  // Constructor
  Plant();

  // Destructor
  ~Plant();

  // private data and function members
private:
  // Block states
  DWplant rtDW;

  // private member function(s) for subsystem '<S1>/qtoQ'
  static void qtoQ(const real_T rtu_q[4], real_T rty_Q[9]);

  // Real-Time Model
  RT_MODELplant rtM;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Scope' : Unused code path elimination
//  Block '<S12>/Reshape' : Reshape block reduction

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('starshotsim_dev/Plant')    - opens subsystem starshotsim_dev/Plant
//  hilite_system('starshotsim_dev/Plant/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'starshotsim_dev'
//  '<S1>'   : 'starshotsim_dev/Plant'
//  '<S2>'   : 'starshotsim_dev/Plant/Cross Product'
//  '<S3>'   : 'starshotsim_dev/Plant/Dynamics'
//  '<S4>'   : 'starshotsim_dev/Plant/Magnetic Field Model'
//  '<S5>'   : 'starshotsim_dev/Plant/Point Error'
//  '<S6>'   : 'starshotsim_dev/Plant/Quaternion Integration'
//  '<S7>'   : 'starshotsim_dev/Plant/Tranlational Dynamics'
//  '<S8>'   : 'starshotsim_dev/Plant/qtoQ'
//  '<S9>'   : 'starshotsim_dev/Plant/qtoQ1'
//  '<S10>'  : 'starshotsim_dev/Plant/Dynamics/Cross Product'
//  '<S11>'  : 'starshotsim_dev/Plant/Dynamics/Invert  3x3 Matrix'
//  '<S12>'  : 'starshotsim_dev/Plant/Dynamics/Invert  3x3 Matrix/Determinant of 3x3 Matrix'
//  '<S13>'  : 'starshotsim_dev/Plant/Magnetic Field Model/Dipole->ECI'
//  '<S14>'  : 'starshotsim_dev/Plant/Quaternion Integration/q_normalize'
//  '<S15>'  : 'starshotsim_dev/Plant/Quaternion Integration/qderiv'
//  '<S16>'  : 'starshotsim_dev/Plant/qtoQ/Subsystem'
//  '<S17>'  : 'starshotsim_dev/Plant/qtoQ1/Subsystem'

#endif // RTW_HEADER_Plant_h_

//
// File trailer for generated code.
//
// [EOF]
//
```

```lib\ACS_libs\Plant_ert_rtw\Plant_data.cpp
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Plant_data.cpp
//
// Code generated for Simulink model 'Plant'.
//
// Model version                  : 13.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Jul 26 17:41:01 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "Plant.h"

// Constant parameters (default storage)
const ConstPplant rtConstPplant = {
  // Pooled Parameter (Expression: eye(3,3))
  //  Referenced by:
  //    '<S8>/Matrix Gain'
  //    '<S9>/Matrix Gain'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 }
};

//
// File trailer for generated code.
//
// [EOF]
//
```

```lib\ACS_libs\Plant_ert_rtw\rtwtypes.h
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: rtwtypes.h
//
// Code generated for Simulink model 'Plant'.
//
// Model version                  : 13.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Jul 26 17:41:01 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//

#ifndef RTWTYPES_H
#define RTWTYPES_H

// Logical type definitions
#if (!defined(__cplusplus))
#ifndef false
#define false                          (0U)
#endif

#ifndef true
#define true                           (1U)
#endif
#endif

//=======================================================================*
//  Target hardware information
//    Device type: ARM Compatible->ARM Cortex-M
//    Number of bits:     char:   8    short:   16    int:  32
//                        long:  32    long long:  64
//                        native word size:  32
//    Byte ordering: LittleEndian
//    Signed integer division rounds to: Zero
//    Shift right on a signed integer as arithmetic shift: on
// =======================================================================

//=======================================================================*
//  Fixed width word size data types:                                     *
//    int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
//    uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
//    real32_T, real64_T           - 32 and 64 bit floating point numbers *
// =======================================================================
typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef long long int64_T;
typedef unsigned long long uint64_T;
typedef float real32_T;
typedef double real64_T;

//===========================================================================*
//  Generic type definitions: boolean_T, char_T, byte_T, int_T, uint_T,       *
//                            real_T, time_T, ulong_T, ulonglong_T.           *
// ===========================================================================
typedef double real_T;
typedef double time_T;
typedef unsigned char boolean_T;
typedef int int_T;
typedef unsigned int uint_T;
typedef unsigned long ulong_T;
typedef unsigned long long ulonglong_T;
typedef char char_T;
typedef unsigned char uchar_T;
typedef char_T byte_T;

//=======================================================================*
//  Min and Max:                                                          *
//    int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
//    uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
// =======================================================================
#define MAX_int8_T                     ((int8_T)(127))
#define MIN_int8_T                     ((int8_T)(-128))
#define MAX_uint8_T                    ((uint8_T)(255U))
#define MAX_int16_T                    ((int16_T)(32767))
#define MIN_int16_T                    ((int16_T)(-32768))
#define MAX_uint16_T                   ((uint16_T)(65535U))
#define MAX_int32_T                    ((int32_T)(2147483647))
#define MIN_int32_T                    ((int32_T)(-2147483647-1))
#define MAX_uint32_T                   ((uint32_T)(0xFFFFFFFFU))
#define MAX_int64_T                    ((int64_T)(9223372036854775807LL))
#define MIN_int64_T                    ((int64_T)(-9223372036854775807LL-1LL))
#define MAX_uint64_T                   ((uint64_T)(0xFFFFFFFFFFFFFFFFULL))

// Block D-Work pointer type
typedef void * pointer_T;

#endif                                 // RTWTYPES_H

//
// File trailer for generated code.
//
// [EOF]
//
```

```lib\ACS_libs\StarshotACS_ert_rtw\StarshotACS.cpp
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: StarshotACS.cpp
//
// Code generated for Simulink model 'StarshotACS'.
//
// Model version                  : 13.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Jul 26 17:42:42 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "StarshotACS.h"

// Exported block parameters
real_T A = 4.0E-5; // Variable: A
                   //  Referenced by:
                   //    '<S2>/Gain'
                   //    '<S2>/Saturation1'
                   //    '<S2>/Saturation2'
                   //    '<S2>/Saturation6'
                   //    '<S3>/Gain'
                   //    '<S3>/Saturation1'
                   //    '<S3>/Saturation2'
                   //    '<S3>/Saturation6'
                   //  Cross Section Area For magnetorquer

real_T Id = 0.0021; // Variable: Id
                    //  Referenced by: '<S2>/Id inverse'
                    //  Id for the Detumble

real_T Kd = 0.0007935279615795299; // Variable: Kd
                                   //  Referenced by: '<S3>/Kd Gain'
                                   //  Kd for the pointing

real_T Kp = 5.2506307629097953E-10; // Variable: Kp
                                    //  Referenced by: '<S3>/Kp Gain'
                                    //  Kp for the pointing

real_T c = 0.004; // Variable: c
                  //  Referenced by:
                  //    '<S2>/Gain 8'
                  //    '<S2>/Kane damping'
                  //  c for the Detumble

real_T i_max = 0.25; // Variable: i_max
                     //  Referenced by:
                     //    '<S2>/Saturation1'
                     //    '<S2>/Saturation2'
                     //    '<S2>/Saturation6'
                     //    '<S3>/Saturation1'
                     //    '<S3>/Saturation2'
                     //    '<S3>/Saturation6'
                     //  Max Current for each magnetorquer

real_T k = 13.5; // Variable: k
                 //  Referenced by:
                 //    '<S2>/Gain'
                 //    '<S2>/Saturation1'
                 //    '<S2>/Saturation2'
                 //    '<S2>/Saturation6'
                 //    '<S3>/Gain'
                 //    '<S3>/Saturation1'
                 //    '<S3>/Saturation2'
                 //    '<S3>/Saturation6'
                 //  Gain for magnetorquer

real_T n = 500.0; // Variable: n
                  //  Referenced by:
                  //    '<S2>/Gain'
                  //    '<S2>/Saturation1'
                  //    '<S2>/Saturation2'
                  //    '<S2>/Saturation6'
                  //    '<S3>/Gain'
                  //    '<S3>/Saturation1'
                  //    '<S3>/Saturation2'
                  //    '<S3>/Saturation6'
                  //  Wire Turn For magnetorquer
real_T step_size = 0.25;

// Model step function
void StarshotACS::step()
{
  real_T rtb_VectorConcatenate[9];
  real_T rtb_VectorConcatenate_0[9];
  real_T rtb_Normalization[3];
  real_T rtb_Product1[3];
  real_T acc;
  real_T rtb_Gain2_0;
  real_T rtb_Gain2_idx_0;
  real_T rtb_Gain2_idx_1;
  real_T rtb_Normalization_0;
  real_T rtb_Saturation2;
  real_T rtb_Saturation6;
  real_T rtb_Sqrt4;
  real_T rtb_TSamp_o;
  real_T rtb_TrigonometricFunction5;
  real_T rtb_TrigonometricFunction5_k;
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T u2_tmp;
  int32_T i;
  int32_T idx1;

  // Outputs for Atomic SubSystem: '<Root>/StarshotACS'
  // Gain: '<S2>/Gain 2' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'

  rtb_Gain2_idx_0 = -rtDW.DiscreteTimeIntegrator_DSTATE[0];

  // Gain: '<S2>/Kane damping' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   Gain: '<S2>/Gain 2'

  rtb_Product1[0] = c * -rtDW.DiscreteTimeIntegrator_DSTATE[0];

  // Gain: '<S2>/Gain 2' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'

  rtb_Gain2_idx_1 = -rtDW.DiscreteTimeIntegrator_DSTATE[1];

  // Gain: '<S2>/Kane damping' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   Gain: '<S2>/Gain 2'

  rtb_Product1[1] = c * -rtDW.DiscreteTimeIntegrator_DSTATE[1];

  // Gain: '<S2>/Gain 2' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'

  rtb_Gain2_0 = -rtDW.DiscreteTimeIntegrator_DSTATE[2];

  // Gain: '<S2>/Kane damping' incorporates:
  //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  //   Gain: '<S2>/Gain 2'

  rtb_Product1[2] = c * -rtDW.DiscreteTimeIntegrator_DSTATE[2];

  // S-Function (sdsp2norm2): '<S2>/Normalization' incorporates:
  //   DotProduct: '<S9>/Dot Product6'
  //   Inport: '<Root>/Bfield_body'

  rtb_Saturation6 = (rtU.Bfield_body[0] * rtU.Bfield_body[0] + rtU.Bfield_body[1] * rtU.Bfield_body[1]) + rtU.Bfield_body[2] *
                                                                                                              rtU.Bfield_body[2];
  acc = 1.0 / (rtb_Saturation6 + 1.0E-10);
  rtb_Normalization[0] = rtU.Bfield_body[0] * acc;
  rtb_Normalization[1] = rtU.Bfield_body[1] * acc;
  rtb_Normalization[2] = rtU.Bfield_body[2] * acc;

  // Sum: '<S5>/Sum' incorporates:
  //   Product: '<S5>/Product'
  //   Product: '<S5>/Product1'

  rtb_Sqrt4 = rtb_Product1[1] * rtb_Normalization[2] - rtb_Normalization[1] *
                                                           rtb_Product1[2];

  // Sum: '<S5>/Sum1' incorporates:
  //   Product: '<S5>/Product2'
  //   Product: '<S5>/Product3'

  rtb_Saturation2 = rtb_Normalization[0] * rtb_Product1[2] - rtb_Product1[0] *
                                                                 rtb_Normalization[2];

  // Sum: '<S5>/Sum2' incorporates:
  //   Product: '<S5>/Product4'
  //   Product: '<S5>/Product5'

  rtb_TrigonometricFunction5_k = rtb_Product1[0] * rtb_Normalization[1] -
                                 rtb_Normalization[0] * rtb_Product1[1];

  // Gain: '<S2>/Gain'
  tmp = 1.0 / (k * A * n);

  // Gain: '<S2>/Id inverse'
  tmp_0 = 1.0 / Id;

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Constant: '<S7>/Constant3'
  //   Gain: '<S7>/Gain'
  //   Inport: '<Root>/angularvelocity'

  rtb_VectorConcatenate[0] = 0.0;
  rtb_VectorConcatenate[1] = rtU.w[2];
  rtb_VectorConcatenate[2] = -rtU.w[1];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Constant: '<S7>/Constant3'
  //   Gain: '<S7>/Gain1'
  //   Inport: '<Root>/angularvelocity'

  rtb_VectorConcatenate[3] = -rtU.w[2];
  rtb_VectorConcatenate[4] = 0.0;
  rtb_VectorConcatenate[5] = rtU.w[0];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Constant: '<S7>/Constant3'
  //   Gain: '<S7>/Gain2'
  //   Inport: '<Root>/angularvelocity'

  rtb_VectorConcatenate[6] = rtU.w[1];
  rtb_VectorConcatenate[7] = -rtU.w[0];
  rtb_VectorConcatenate[8] = 0.0;

  // SampleTimeMath: '<S6>/TSamp' incorporates:
  //   Inport: '<Root>/angularvelocity'
  //
  //  About '<S6>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Product1[0] = rtU.w[0] * (1.0 / step_size);
  rtb_Product1[1] = rtU.w[1] * (1.0 / step_size);
  rtb_Product1[2] = rtU.w[2] * (1.0 / step_size);

  // Sqrt: '<S9>/Sqrt4'
  rtb_Saturation6 = std::sqrt(rtb_Saturation6);

  // DotProduct: '<S10>/Dot Product6'
  rtb_Normalization_0 = 0.0;

  // DotProduct: '<S14>/Dot Product6'
  u2_tmp = 0.0;

  // DotProduct: '<S15>/Dot Product6'
  tmp_1 = 0.0;

  // S-Function (sdsp2norm2): '<S4>/Normalization'
  idx1 = 0;
  acc = 0.0;
  for (i = 0; i < 3; i++)
  {
    // Product: '<S3>/Product6' incorporates:
    //   Concatenate: '<S11>/Vector Concatenate'
    //   Inport: '<Root>/Bfield_body'
    //   Product: '<S3>/Product4'

    rtb_TrigonometricFunction5 = ((rtConstB.VectorConcatenate[i + 3] *
                                       rtU.Bfield_body[1] +
                                   rtConstB.VectorConcatenate[i] * rtU.Bfield_body[0]) +
                                  rtConstB.VectorConcatenate[i + 6] * rtU.Bfield_body[2]) /
                                 rtb_Saturation6;

    // DotProduct: '<S10>/Dot Product6'
    rtb_Normalization_0 += rtb_TrigonometricFunction5 *
                           rtb_TrigonometricFunction5;

    // Product: '<S12>/Product3' incorporates:
    //   Inport: '<Root>/Bfield_body'
    //   Inport: '<Root>/angularvelocity'

    rtb_TrigonometricFunction5 = rtU.w[i];
    rtb_Normalization[i] = rtb_TrigonometricFunction5 * rtU.Bfield_body[i];

    // DotProduct: '<S14>/Dot Product6'
    u2_tmp += rtb_TrigonometricFunction5 * rtb_TrigonometricFunction5;

    // DotProduct: '<S15>/Dot Product6' incorporates:
    //   Inport: '<Root>/Bfield_body'

    tmp_1 += rtU.Bfield_body[i] * rtU.Bfield_body[i];

    // S-Function (sdsp2norm2): '<S4>/Normalization' incorporates:
    //   Inport: '<Root>/Bfield_body'

    acc += rtU.Bfield_body[idx1] * rtU.Bfield_body[idx1];
    idx1++;
  }

  // Sqrt: '<S10>/Sqrt4' incorporates:
  //   DotProduct: '<S10>/Dot Product6'

  rtb_Normalization_0 = std::sqrt(rtb_Normalization_0);

  // Trigonometry: '<S3>/Trigonometric Function5'
  if (rtb_Normalization_0 > 1.0)
  {
    rtb_Normalization_0 = 1.0;
  }

  rtb_TrigonometricFunction5 = std::asin(rtb_Normalization_0);

  // End of Trigonometry: '<S3>/Trigonometric Function5'

  // SampleTimeMath: '<S8>/TSamp'
  //
  //  About '<S8>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_o = rtb_TrigonometricFunction5 * (1.0 / step_size);

  // Switch: '<S13>/Switch1' incorporates:
  //   Constant: '<S13>/Constant10'
  //   Constant: '<S13>/Constant9'
  //   Inport: '<Root>/angularvelocity'

  if (rtU.w[2] >= 0.0)
  {
    idx1 = 1;
  }
  else
  {
    idx1 = -1;
  }

  // Switch: '<S12>/Switch' incorporates:
  //   Constant: '<S12>/Constant3'
  //   Constant: '<S12>/Constant4'
  //   DotProduct: '<S14>/Dot Product6'
  //   DotProduct: '<S15>/Dot Product6'
  //   Product: '<S12>/Divide6'
  //   Sqrt: '<S14>/Sqrt4'
  //   Sqrt: '<S15>/Sqrt4'
  //   Sum: '<S12>/Add'

  if (((rtb_Normalization[0] + rtb_Normalization[1]) + rtb_Normalization[2]) *
          (1.0 / std::sqrt(u2_tmp)) / std::sqrt(tmp_1) >
      0.0)
  {
    i = 1;
  }
  else
  {
    i = -1;
  }

  // Product: '<S3>/Product7' incorporates:
  //   Gain: '<S3>/Kd Gain'
  //   Gain: '<S3>/Kp Gain'
  //   Product: '<S3>/Product8'
  //   Sum: '<S3>/Sum7'
  //   Sum: '<S8>/Diff'
  //   Switch: '<S12>/Switch'
  //   Switch: '<S13>/Switch1'
  //   UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM

  rtb_Normalization_0 = ((rtb_TSamp_o - rtDW.UD_DSTATE_k) * Kd + Kp *
                                                                     rtb_TrigonometricFunction5) *
                        static_cast<real_T>(idx1 * i) /
                        rtb_Saturation6;

  // Saturate: '<S3>/Saturation6' incorporates:
  //   Saturate: '<S2>/Saturation1'
  //   Saturate: '<S2>/Saturation2'
  //   Saturate: '<S2>/Saturation6'
  //   Saturate: '<S3>/Saturation1'

  rtb_Saturation6 = -k * i_max * A * n;
  u2_tmp = k * i_max * A * n;

  // Gain: '<S3>/Gain'
  rtb_TrigonometricFunction5 = 1.0 / (A * n * k);

  // S-Function (sdsp2norm2): '<S4>/Normalization' incorporates:
  //   Inport: '<Root>/Bfield_body'

  acc = 1.0 / (std::sqrt(acc) + 1.0E-10);
  rtb_Normalization[0] = rtU.Bfield_body[0] * acc;
  rtb_Normalization[1] = rtU.Bfield_body[1] * acc;
  rtb_Normalization[2] = rtU.Bfield_body[2] * acc;

  // Sum: '<S2>/Sum10' incorporates:
  //   Concatenate: '<S7>/Vector Concatenate'
  //   Constant: '<S2>/Identity matrix'

  for (idx1 = 0; idx1 < 9; idx1++)
  {
    rtb_VectorConcatenate_0[idx1] = rtb_VectorConcatenate[idx1] +
                                    rtConstP.Identitymatrix_Value[idx1];
  }

  // End of Sum: '<S2>/Sum10'

  // Update for UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_k = rtb_TSamp_o;

  // Saturate: '<S2>/Saturation1'
  if (rtb_Sqrt4 > u2_tmp)
  {
    rtb_Sqrt4 = u2_tmp;
  }
  else if (rtb_Sqrt4 < rtb_Saturation6)
  {
    rtb_Sqrt4 = rtb_Saturation6;
  }

  // Outport: '<Root>/detumble' incorporates:
  //   Gain: '<S2>/Gain'
  //   Saturate: '<S2>/Saturation1'

  rtY.detumble[0] = tmp * rtb_Sqrt4;

  // Saturate: '<S2>/Saturation2'
  if (rtb_Saturation2 > u2_tmp)
  {
    rtb_Saturation2 = u2_tmp;
  }
  else if (rtb_Saturation2 < rtb_Saturation6)
  {
    rtb_Saturation2 = rtb_Saturation6;
  }

  // Outport: '<Root>/detumble' incorporates:
  //   Gain: '<S2>/Gain'
  //   Saturate: '<S2>/Saturation2'

  rtY.detumble[1] = tmp * rtb_Saturation2;

  // Saturate: '<S2>/Saturation6'
  if (rtb_TrigonometricFunction5_k > u2_tmp)
  {
    rtb_TrigonometricFunction5_k = u2_tmp;
  }
  else if (rtb_TrigonometricFunction5_k < rtb_Saturation6)
  {
    rtb_TrigonometricFunction5_k = rtb_Saturation6;
  }

  // Outport: '<Root>/detumble' incorporates:
  //   Gain: '<S2>/Gain'
  //   Saturate: '<S2>/Saturation6'

  rtY.detumble[2] = tmp * rtb_TrigonometricFunction5_k;

  // Saturate: '<S3>/Saturation1'
  if (u2_tmp < 0.0)
  {
    // Saturate: '<S3>/Saturation2'
    rtb_Sqrt4 = u2_tmp;
  }
  else if (rtb_Saturation6 > 0.0)
  {
    // Saturate: '<S3>/Saturation2'
    rtb_Sqrt4 = rtb_Saturation6;
  }
  else
  {
    // Saturate: '<S3>/Saturation2'
    rtb_Sqrt4 = 0.0;
  }

  // Gain: '<S3>/Gain' incorporates:
  //   Saturate: '<S3>/Saturation1'

  acc = rtb_TrigonometricFunction5 * rtb_Sqrt4;

  // Outport: '<Root>/point' incorporates:
  //   Gain: '<S3>/Gain'

  rtY.point[0] = acc;
  rtY.point[1] = acc;

  // Saturate: '<S3>/Saturation6'
  if (rtb_Normalization_0 > u2_tmp)
  {
    rtb_Normalization_0 = u2_tmp;
  }
  else if (rtb_Normalization_0 < rtb_Saturation6)
  {
    rtb_Normalization_0 = rtb_Saturation6;
  }

  // Outport: '<Root>/point' incorporates:
  //   Gain: '<S3>/Gain'
  //   Saturate: '<S3>/Saturation6'

  rtY.point[2] = rtb_TrigonometricFunction5 * rtb_Normalization_0;

  // DotProduct: '<S4>/Dot Product'
  rtb_Normalization_0 = 0.0;
  for (i = 0; i < 3; i++)
  {
    // Sum: '<S6>/Diff' incorporates:
    //   Product: '<S2>/Product1'
    //
    //  Block description for '<S6>/Diff':
    //
    //   Add in CPU

    acc = rtb_Product1[i];

    // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' incorporates:
    //   Gain: '<S2>/Gain 2'
    //   Gain: '<S2>/Gain 8'
    //   Gain: '<S2>/Gain 9'
    //   Gain: '<S2>/Id inverse'
    //   Product: '<S2>/Product1'
    //   Sum: '<S2>/Sum8'
    //   Sum: '<S6>/Diff'
    //   UnitDelay: '<S6>/UD'
    //
    //  Block description for '<S6>/Diff':
    //
    //   Add in CPU
    //
    //  Block description for '<S6>/UD':
    //
    //   Store in Global RAM

    rtDW.DiscreteTimeIntegrator_DSTATE[i] += ((0.0 - (acc - rtDW.UD_DSTATE[i])) - ((tmp_0 * -rtb_Gain2_idx_0 * c * rtb_VectorConcatenate_0[i] + tmp_0 *
                                                                                                                                                    -rtb_Gain2_idx_1 * c * rtb_VectorConcatenate_0[i + 3]) +
                                                                                   tmp_0 *
                                                                                       -rtb_Gain2_0 * c * rtb_VectorConcatenate_0[i + 6])) *
                                             step_size;

    // Update for UnitDelay: '<S6>/UD'
    //
    //  Block description for '<S6>/UD':
    //
    //   Store in Global RAM

    rtDW.UD_DSTATE[i] = acc;

    // DotProduct: '<S4>/Dot Product' incorporates:
    //   Constant: '<S4>/Constant'

    rtb_Normalization_0 += rtb_Normalization[i] * rtConstP.pooled1[i];
  }

  // Saturate: '<S4>/Saturation3' incorporates:
  //   DotProduct: '<S4>/Dot Product'

  if (rtb_Normalization_0 > 1.0)
  {
    rtb_Normalization_0 = 1.0;
  }
  else if (rtb_Normalization_0 < -1.0)
  {
    rtb_Normalization_0 = -1.0;
  }

  // Outport: '<Root>/pt_error' incorporates:
  //   Gain: '<S4>/Multiply'
  //   Saturate: '<S4>/Saturation3'
  //   Trigonometry: '<S4>/Acos'

  rtY.pt_error = 57.295779513082323 * std::acos(rtb_Normalization_0);

  // End of Outputs for SubSystem: '<Root>/StarshotACS'
}

// Model initialize function
// void StarshotACS::initialize()
// {
//   // SystemInitialize for Atomic SubSystem: '<Root>/StarshotACS'
//   // InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
//   rtDW.DiscreteTimeIntegrator_DSTATE[0] = 0.0;
//   rtDW.DiscreteTimeIntegrator_DSTATE[1] = 0.0;
//   rtDW.DiscreteTimeIntegrator_DSTATE[2] = 1.0;

//   // End of SystemInitialize for SubSystem: '<Root>/StarshotACS'
// }

void StarshotACS::initialize(double step_size_input, double A_input, double Id_input, double Kd_input, double Kp_input, double c_input, double i_max_input, double k_input, double n_input)
{
  // SystemInitialize for Atomic SubSystem: '<Root>/StarshotACS'
  // InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE[0] = 0.0;
  rtDW.DiscreteTimeIntegrator_DSTATE[1] = 0.0;
  rtDW.DiscreteTimeIntegrator_DSTATE[2] = 1.0;

  step_size = step_size_input;
  A = A_input;
  Id = Id_input;
  Kd = Kd_input;
  Kp = Kp_input;
  c = c_input;
  i_max = i_max_input;
  k = k_input;
  n = n_input;

  // End of SystemInitialize for SubSystem: '<Root>/StarshotACS'
}

// Constructor
StarshotACS::StarshotACS() : rtU(),
                             rtY(),
                             rtDW(),
                             rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
StarshotACS::~StarshotACS()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL *StarshotACS::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
```

```lib\ACS_libs\StarshotACS_ert_rtw\StarshotACS.h
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: StarshotACS.h
//
// Code generated for Simulink model 'StarshotACS'.
//
// Model version                  : 13.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Jul 26 17:42:42 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_StarshotACS_h_
#define RTW_HEADER_StarshotACS_h_
#include "rtwtypes.h"
#include <cmath>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// Block signals and states (default storage) for system '<Root>'
struct DW {
  real_T DiscreteTimeIntegrator_DSTATE[3];// '<S2>/Discrete-Time Integrator'
  real_T UD_DSTATE[3];                 // '<S6>/UD'
  real_T UD_DSTATE_k;                  // '<S8>/UD'
};

// Invariant block signals (default storage)
struct ConstB {
  real_T VectorConcatenate[9];         // '<S11>/Vector Concatenate'
};

// Constant parameters (default storage)
struct ConstP {
  // Expression: [1 0 0;0 1 0;0 0 1]
  //  Referenced by: '<S2>/Identity matrix'

  real_T Identitymatrix_Value[9];

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S2>/Discrete-Time Integrator'
  //    '<S3>/e_z'
  //    '<S4>/Constant'

  real_T pooled1[3];
};

// External inputs (root inport signals with default storage)
struct ExtU {
  real_T w[3];                         // '<Root>/angularvelocity'
  real_T Bfield_body[3];               // '<Root>/Bfield_body'
};

// External outputs (root outports fed by signals with default storage)
struct ExtY {
  real_T detumble[3];                  // '<Root>/detumble'
  real_T point[3];                     // '<Root>/point'
  real_T pt_error;                     // '<Root>/pt_error'
};

// Real-time Model Data Structure
struct tag_RTM {
  const char_T * volatile errorStatus;
};

extern const ConstB rtConstB;          // constant block i/o

// Constant parameters (default storage)
extern const ConstP rtConstP;

//
//  Exported Global Parameters
//
//  Note: Exported global parameters are tunable parameters with an exported
//  global storage class designation.  Code generation will declare the memory for
//  these parameters and exports their symbols.
//

extern real_T A;                       // Variable: A
                                          //  Referenced by:
                                          //    '<S2>/Gain'
                                          //    '<S2>/Saturation1'
                                          //    '<S2>/Saturation2'
                                          //    '<S2>/Saturation6'
                                          //    '<S3>/Gain'
                                          //    '<S3>/Saturation1'
                                          //    '<S3>/Saturation2'
                                          //    '<S3>/Saturation6'
                                          //  Cross Section Area For magnetorquer

extern real_T Id;                      // Variable: Id
                                          //  Referenced by: '<S2>/Id inverse'
                                          //  Id for the Detumble

extern real_T Kd;                      // Variable: Kd
                                          //  Referenced by: '<S3>/Kd Gain'
                                          //  Kd for the pointing

extern real_T Kp;                      // Variable: Kp
                                          //  Referenced by: '<S3>/Kp Gain'
                                          //  Kp for the pointing

extern real_T c;                       // Variable: c
                                          //  Referenced by:
                                          //    '<S2>/Gain 8'
                                          //    '<S2>/Kane damping'
                                          //  c for the Detumble

extern real_T i_max;                   // Variable: i_max
                                          //  Referenced by:
                                          //    '<S2>/Saturation1'
                                          //    '<S2>/Saturation2'
                                          //    '<S2>/Saturation6'
                                          //    '<S3>/Saturation1'
                                          //    '<S3>/Saturation2'
                                          //    '<S3>/Saturation6'
                                          //  Max Current for each magnetorquer

extern real_T k;                       // Variable: k
                                          //  Referenced by:
                                          //    '<S2>/Gain'
                                          //    '<S2>/Saturation1'
                                          //    '<S2>/Saturation2'
                                          //    '<S2>/Saturation6'
                                          //    '<S3>/Gain'
                                          //    '<S3>/Saturation1'
                                          //    '<S3>/Saturation2'
                                          //    '<S3>/Saturation6'
                                          //  Gain for magnetorquer

extern real_T n;                       // Variable: n
                                          //  Referenced by:
                                          //    '<S2>/Gain'
                                          //    '<S2>/Saturation1'
                                          //    '<S2>/Saturation2'
                                          //    '<S2>/Saturation6'
                                          //    '<S3>/Gain'
                                          //    '<S3>/Saturation1'
                                          //    '<S3>/Saturation2'
                                          //    '<S3>/Saturation6'
                                          //  Wire Turn For magnetorquer


// Class declaration for model StarshotACS
class StarshotACS
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL * getRTM();

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
//   void initialize();
  void initialize(double step_size_input, double A_input, double Id_input, double Kd_input, double Kp_input, double c_input, double i_max_input, double k_input, double n_input);

      // model step function
  void step();

  // Constructor
  StarshotACS();

  // Destructor
  ~StarshotACS();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // Real-Time Model
  RT_MODEL rtM;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S6>/Data Type Duplicate' : Unused code path elimination
//  Block '<S8>/Data Type Duplicate' : Unused code path elimination
//  Block '<S3>/Gain1' : Unused code path elimination
//  Block '<S3>/Scope' : Unused code path elimination
//  Block '<S3>/Scope1' : Unused code path elimination
//  Block '<S2>/Gain 1' : Eliminated nontunable gain of 1


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('starshotsim_dev/StarshotACS')    - opens subsystem starshotsim_dev/StarshotACS
//  hilite_system('starshotsim_dev/StarshotACS/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'starshotsim_dev'
//  '<S1>'   : 'starshotsim_dev/StarshotACS'
//  '<S2>'   : 'starshotsim_dev/StarshotACS/KD_controller'
//  '<S3>'   : 'starshotsim_dev/StarshotACS/Orientation controller'
//  '<S4>'   : 'starshotsim_dev/StarshotACS/Point Error'
//  '<S5>'   : 'starshotsim_dev/StarshotACS/KD_controller/Cross'
//  '<S6>'   : 'starshotsim_dev/StarshotACS/KD_controller/Discrete Derivative'
//  '<S7>'   : 'starshotsim_dev/StarshotACS/KD_controller/Skew matrix S(m)1'
//  '<S8>'   : 'starshotsim_dev/StarshotACS/Orientation controller/Discrete Derivative'
//  '<S9>'   : 'starshotsim_dev/StarshotACS/Orientation controller/Norm2'
//  '<S10>'  : 'starshotsim_dev/StarshotACS/Orientation controller/Norm3'
//  '<S11>'  : 'starshotsim_dev/StarshotACS/Orientation controller/Skew matrix S(tau_c)1'
//  '<S12>'  : 'starshotsim_dev/StarshotACS/Orientation controller/Subsystem1'
//  '<S13>'  : 'starshotsim_dev/StarshotACS/Orientation controller/sign_function1'
//  '<S14>'  : 'starshotsim_dev/StarshotACS/Orientation controller/Subsystem1/Norm4'
//  '<S15>'  : 'starshotsim_dev/StarshotACS/Orientation controller/Subsystem1/Norm5'

#endif                                 // RTW_HEADER_StarshotACS_h_

//
// File trailer for generated code.
//
// [EOF]
//
```

```lib\ACS_libs\StarshotACS_ert_rtw\StarshotACS_data.cpp
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: StarshotACS_data.cpp
//
// Code generated for Simulink model 'StarshotACS'.
//
// Model version                  : 13.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Jul 26 17:42:42 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "StarshotACS.h"

// Invariant block signals (default storage)
const ConstB rtConstB = {
  {
    0.0,
    1.0,
    -0.0,
    -1.0,
    0.0,
    0.0,
    0.0,
    -0.0,
    0.0
  }
  // '<S11>/Vector Concatenate'
};

// Constant parameters (default storage)
const ConstP rtConstP = {
  // Expression: [1 0 0;0 1 0;0 0 1]
  //  Referenced by: '<S2>/Identity matrix'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S2>/Discrete-Time Integrator'
  //    '<S3>/e_z'
  //    '<S4>/Constant'

  { 0.0, 0.0, 1.0 }
};

//
// File trailer for generated code.
//
// [EOF]
//
```

```lib\ACS_libs\StarshotACS_ert_rtw\rtwtypes.h
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: rtwtypes.h
//
// Code generated for Simulink model 'StarshotACS'.
//
// Model version                  : 13.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Wed Jul 26 17:42:42 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//

#ifndef RTWTYPES_H
#define RTWTYPES_H

// Logical type definitions
#if (!defined(__cplusplus))
#ifndef false
#define false                          (0U)
#endif

#ifndef true
#define true                           (1U)
#endif
#endif

//=======================================================================*
//  Target hardware information
//    Device type: ARM Compatible->ARM Cortex-M
//    Number of bits:     char:   8    short:   16    int:  32
//                        long:  32    long long:  64
//                        native word size:  32
//    Byte ordering: LittleEndian
//    Signed integer division rounds to: Zero
//    Shift right on a signed integer as arithmetic shift: on
// =======================================================================

//=======================================================================*
//  Fixed width word size data types:                                     *
//    int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
//    uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
//    real32_T, real64_T           - 32 and 64 bit floating point numbers *
// =======================================================================
typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef long long int64_T;
typedef unsigned long long uint64_T;
typedef float real32_T;
typedef double real64_T;

//===========================================================================*
//  Generic type definitions: boolean_T, char_T, byte_T, int_T, uint_T,       *
//                            real_T, time_T, ulong_T, ulonglong_T.           *
// ===========================================================================
typedef double real_T;
typedef double time_T;
typedef unsigned char boolean_T;
typedef int int_T;
typedef unsigned int uint_T;
typedef unsigned long ulong_T;
typedef unsigned long long ulonglong_T;
typedef char char_T;
typedef unsigned char uchar_T;
typedef char_T byte_T;

//=======================================================================*
//  Min and Max:                                                          *
//    int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
//    uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
// =======================================================================
#define MAX_int8_T                     ((int8_T)(127))
#define MIN_int8_T                     ((int8_T)(-128))
#define MAX_uint8_T                    ((uint8_T)(255U))
#define MAX_int16_T                    ((int16_T)(32767))
#define MIN_int16_T                    ((int16_T)(-32768))
#define MAX_uint16_T                   ((uint16_T)(65535U))
#define MAX_int32_T                    ((int32_T)(2147483647))
#define MIN_int32_T                    ((int32_T)(-2147483647-1))
#define MAX_uint32_T                   ((uint32_T)(0xFFFFFFFFU))
#define MAX_int64_T                    ((int64_T)(9223372036854775807LL))
#define MIN_int64_T                    ((int64_T)(-9223372036854775807LL-1LL))
#define MAX_uint64_T                   ((uint64_T)(0xFFFFFFFFFFFFFFFFULL))

// Block D-Work pointer type
typedef void * pointer_T;

#endif                                 // RTWTYPES_H

//
// File trailer for generated code.
//
// [EOF]
//
```

```lib\ArduinoEigen\ArduinoEigen.h
#pragma once
#ifndef ARDUINO_EIGEN_H
#define ARDUINO_EIGEN_H

#include <Arduino.h>
#include "ArduinoEigen/ArduinoEigenCommon.h"

// guarantee that the Eigen code that you are #including is licensed under the MPL2
#define EIGEN_MPL2_ONLY

#include "ArduinoEigen/Eigen/Eigen"
#include "ArduinoEigen/ArduinoEigenExtension.h"

#endif  // ARDUINO_EIGEN_H
```

```lib\ArduinoEigen\ArduinoEigenDense.h
#pragma once
#ifndef ARDUINO_EIGEN_DENSE_H
#define ARDUINO_EIGEN_DENSE_H

//#include <Arduino.h>
#include "ArduinoEigen/ArduinoEigenCommon.h"

// guarantee that the Eigen code that you are #including is licensed under the MPL2
#define EIGEN_MPL2_ONLY

#include "ArduinoEigen/Eigen/Dense"
#include "ArduinoEigen/ArduinoEigenExtension.h"

#endif  // ARDUINO_EIGEN_DENSE_H
```

```lib\ArduinoEigen\ArduinoEigenSparse.h
#pragma once
#ifndef ARDUINO_EIGEN_SPARSE_H
#define ARDUINO_EIGEN_SPARSE_H

#include <Arduino.h>
#include "ArduinoEigen/ArduinoEigenCommon.h"

// guarantee that the Eigen code that you are #including is licensed under the MPL2
#define EIGEN_MPL2_ONLY
#include "ArduinoEigen/Eigen/Sparse"

#include "ArduinoEigen/ArduinoEigenExtension.h"

#endif  // ARDUINO_EIGEN_SPARSE_H
```


```lib\ekf\ekf.h
#ifndef EKF_H
#define EKF_H

#pragma once

#include <ArduinoEigen.h>
#include <vector>

class EKF {
public:
    EKF();

    void initialize(double delta_t, const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance, const Eigen::MatrixXd &process_noise_covariance, const Eigen::MatrixXd &noise_covariance, const Eigen::MatrixXd &Hd);
    void step();

    Eigen::VectorXd state;
    Eigen::VectorXd Z;
    Eigen::MatrixXd covariance;

private:
    double dt;
    Eigen::MatrixXd Q;  // Process noise covariance
    Eigen::MatrixXd R_d; // (measurement noise variance) Matrices
    Eigen::MatrixXd H_d;

    void predict(const Eigen::MatrixXd &J_k_k);
    void correct();
    Eigen::MatrixXd CalculateJacobian();
};

#endif // EKF_H
```

```src\DataLogging.cpp
#include "DataLogging.hpp"

const int chipSelect = BUILTIN_SDCARD;
File DataFile;


void DataLogSetup(String s){

    if (!SD.begin(chipSelect)) {
        Serial.println("SD Card initialization failed!");
        return;
    }

    Serial.println("SD Card initialization done.");

    DataFile = SD.open(s.append(".txt").c_str(), FILE_WRITE);

    DataFile.println("---START---");

    DataFile.close();
}

void DataLog(double Data[], int size, String s)
{
    DataFile = SD.open(s.append(".txt").c_str(), FILE_WRITE);
    if (DataFile) {
        for(int i = 0 ; i< size;i++) {
            DataFile.print(Data[i]);
            DataFile.print(", ");

            Serial.print(Data[i]);
            Serial.print(", ");
        }

        DataFile.println();
        Serial.println();
        DataFile.close();
        //Serial.println("done.");
    } else {
        // if the file didn't open, print an error:
        Serial.println("error opening");
    }
    DataFile.close();
}
```

```src\DataLogging.hpp
#include "Arduino.h"
#include <SD.h>
#include <SPI.h>


//Initializes the SD card
void DataLogSetup(String s);

//Opens the Data.txt file, writes data, close the file when exits
void DataLog( double Data[], int size, String s);
```

```src\MagneticFieldFilter.hpp
#ifndef MAGNETICFIELDFILTER_HPP
#define MAGNETICFIELDFILTER_HPP

// Standard library includes
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>

// External library includes
#include <SD.h>
#include "../lib/ArduinoEigen/ArduinoEigenDense.h"
#include "ekf.h"

/**
 * @brief A class for filtering magnetic field measurements using an Extended Kalman Filter
 * and soft-iron calibration data for three axes.
 */
class MagneticFieldFilter {
private:
    // EKF-related members
    Eigen::VectorXd state;                    // State vector for EKF
    Eigen::MatrixXd covariance;               // Covariance matrix
    Eigen::MatrixXd processNoiseCovariance;   // Process noise covariance matrix
    Eigen::MatrixXd measurementNoiseCovariance; // Measurement noise covariance matrix
    Eigen::MatrixXd H_d;                      // Measurement matrix
    EKF ekf;                                  // Extended Kalman Filter instance

    // Calibration-related members
    float voltage;                            // Current battery voltage for calibration
    std::vector<std::vector<float>> softironDataX; // Calibration coefficients for X axis
    std::vector<std::vector<float>> softironDataY; // Calibration coefficients for Y axis
    std::vector<std::vector<float>> softironDataZ; // Calibration coefficients for Z axis

    /**
     * @brief Loads calibration data from a CSV file on the SD card
     * @param filePath Path to the CSV file
     * @param data Vector to store the loaded calibration data
     * @throws std::runtime_error if file cannot be opened or read
     */
    void loadCSV(const std::string& filePath, std::vector<std::vector<float>>& data) {
        File file = SD.open(filePath.c_str());
        if (!file) {
            throw std::runtime_error("Failed to open file: " + filePath);
        }

        data.clear(); // Ensure vector is empty before loading new data

        while (file.available()) {
            String line = file.readStringUntil('\n');
            if (line.length() == 0) continue; // Skip empty lines

            std::vector<float> row;
            char* end;
            const char* cLine = line.c_str();

            while (*cLine) {
                float value = strtof(cLine, &end);
                if (cLine == end) break;
                row.push_back(value);
                cLine = end;
                while (*cLine == ',' || *cLine == ' ') cLine++; // Skip delimiter and whitespace
            }

            if (!row.empty()) {
                data.push_back(row);
            }
        }
        file.close();
    }

    /**
     * @brief Calculates the voltage index for calibration data lookup
     * @return Index corresponding to current voltage value
     */
    int getVoltageIndex() const {
        int index = static_cast<int>((voltage - 3.6f) * 10.0f);
        return std::max(0, std::min(index, 6)); // Clamp to valid range [0, 6]
    }

public:
    /**
     * @brief Constructs a new MagneticFieldFilter
     * @param initialVoltage Initial battery voltage (3.6V - 4.2V)
     */
    MagneticFieldFilter(float initialVoltage)
        : voltage(std::round(initialVoltage * 10.0f) / 10.0f) {
        voltage = std::max(3.6f, std::min(voltage, 4.2f));
    }

    /**
     * @brief Updates the battery voltage used for calibration
     * @param newVoltage New battery voltage (3.6V - 4.2V)
     */
    void setVoltage(float newVoltage) {
        voltage = std::round(newVoltage * 10.0f) / 10.0f;
        voltage = std::max(3.6f, std::min(voltage, 4.2f));
    }

    /**
     * @brief Loads calibration files for all three axes
     * @param xFile Path to X-axis calibration file
     * @param yFile Path to Y-axis calibration file
     * @param zFile Path to Z-axis calibration file
     * @throws std::runtime_error if any file fails to load
     */
    void loadCalibrationFiles(const std::string& xFile, const std::string& yFile, const std::string& zFile) {
        loadCSV(xFile, softironDataX);
        loadCSV(yFile, softironDataY);
        loadCSV(zFile, softironDataZ);
    }

    /**
     * @brief Initializes the Extended Kalman Filter with default parameters
     */
    void initializeEKF() {
        state = Eigen::VectorXd::Zero(6);
        covariance = Eigen::MatrixXd::Identity(6, 6);
        processNoiseCovariance = Eigen::MatrixXd::Identity(6, 6) * 0.01;
        measurementNoiseCovariance = Eigen::MatrixXd::Identity(6, 6) * 0.05;
        H_d = Eigen::MatrixXd::Identity(6, 6);

        ekf.initialize(0.01, state, covariance, processNoiseCovariance,
                      measurementNoiseCovariance, H_d);
    }

    /**
     * @brief Calculates magnetic field offsets for a given PWM value and axis
     * @param pwmNum PWM value
     * @param axis Axis identifier ("x", "y", or "z")
     * @return Vector containing calculated offsets {offsetX, offsetY, offsetZ}
     * @throws std::invalid_argument if axis is invalid
     * @throws std::out_of_range if voltage index is out of range
     */
    std::vector<float> calculateOffsets(float pwmNum, const std::string& axis) {
        const std::vector<std::vector<float>>* calibrationData;

        if (axis == "x") calibrationData = &softironDataX;
        else if (axis == "y") calibrationData = &softironDataY;
        else if (axis == "z") calibrationData = &softironDataZ;
        else throw std::invalid_argument("Invalid axis: " + axis);

        size_t voltageIndex = getVoltageIndex();
        if (voltageIndex >= (*calibrationData)[0].size()) {
            throw std::out_of_range("Voltage index out of range");
        }

        std::vector<float> coefficients;
        coefficients.reserve(9); // Pre-allocate for 9 coefficients
        for (const auto& row : *calibrationData) {
            coefficients.push_back(row[voltageIndex]);
        }

        // Calculate polynomial offsets for each axis
        float offsetX = coefficients[0] * pwmNum +
                       coefficients[1] * std::pow(pwmNum, 2) +
                       coefficients[2] * std::pow(pwmNum, 3);
        float offsetY = coefficients[3] * pwmNum +
                       coefficients[4] * std::pow(pwmNum, 2) +
                       coefficients[5] * std::pow(pwmNum, 3);
        float offsetZ = coefficients[6] * pwmNum +
                       coefficients[7] * std::pow(pwmNum, 2) +
                       coefficients[8] * std::pow(pwmNum, 3);

        return {offsetX, offsetY, offsetZ};
    }

    /**
     * @brief Filters magnetic field measurements using the EKF
     * @param measuredValues Vector of measured magnetic field values
     * @param gyroValues Vector of measured gyroscope values
     * @return Filtered magnetic field values
     */
    Eigen::Vector3d filter(const Eigen::Vector3d& measuredValues, const Eigen::Vector3d& gyroValues) {
        ekf.Z << measuredValues(0), measuredValues(1), measuredValues(2),
                 gyroValues(0), gyroValues(1), gyroValues(2);
        ekf.step();
        return Eigen::Vector3d(ekf.state(0), ekf.state(1), ekf.state(2));
    }
};

#endif // MAGNETICFIELDFILTER_HPP
```

```src\main.cpp
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <SD.h>
#include "MagneticFieldFilter.hpp" // Include SD library for Teensy
#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"

#include "DataLogging.hpp"

#include <errno.h>
#include <sys/stat.h>

// Stub implementation for the `_open` system call
extern "C" int _open(const char* pathname, int flags, int mode) {
    errno = ENOSYS; // Function not implemented
    return -1;      // Return error
}

// Pins for inputs
#define AIN1 31
#define AIN2 32
#define PWMA 30
#define STBY 29
#define LED 13
#define file_name "test"

static StarshotACS starshotObj;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
MagneticFieldFilter magneticFieldFilter(3.8f); // Initialize with nominal voltage

int current2PWM(float current) {
    if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) < 8.062)
        return 0;
    else if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) > 255)
        return 255;
    else
        return int(633.5 * pow(fabs(current), 0.6043) + 8.062);
}

// Starshot parameters
double A_input = 4.0E-5;
double Id_input = 0.0021;
double Kd_input = 0.0001;
double Kp_input = 5;
double c_input = 0.004;
double i_max_input = 0.25;
double k_input = 13.5;
double n_input = 500.0;
double step_size_input = 0.2; // sec

void ACSWrite(float current) {
    int PWM = current2PWM(current);
    if (PWM == 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    } else if (current < 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, PWM);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, PWM);
    }
}

void setup() {
    Serial.begin(9600);

    DataLogSetup(file_name);
    pinMode(LED, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    digitalWrite(LED, HIGH);

    starshotObj.initialize(step_size_input, A_input, Id_input, Kd_input, Kp_input, c_input, i_max_input, k_input, n_input);

    if (!imu.begin()) {
        while (1) {
            Serial.println("IMU initialization failed.");
            delay(100);
        }
    }

    Serial.println("Setting up LSM9DS1 9DOF IMU...");
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_12GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

    // Load calibration files for the MagneticFieldFilter
    magneticFieldFilter.loadCalibrationFiles(
        "../csv/softirons_x.csv",
        "../csv/softirons_y.csv",
        "../csv/softirons_z.csv"
    );

    magneticFieldFilter.initializeEKF();

    delay(1000);
}

void loop() {
    unsigned long start = millis();

    sensors_event_t accel, mag, gyro, temp;
    imu.getEvent(&accel, &mag, &gyro, &temp);

    // Hard iron offsets
    float mag_hardiron_x = -4.9547;
    float mag_hardiron_y = 49.75155;
    float mag_hardiron_z = -13.8556;

    // Get battery voltage
    int voltage_value_pin = 23;
    float voltage_ref = 3.3;
    int resolution = 1023;
    int r1 = 4700;
    int r2 = 10000;
    float voltage = analogRead(voltage_value_pin) * voltage_ref / resolution * (r1 + r2) / r2;

    // Update voltage in MagneticFieldFilter
    magneticFieldFilter.setVoltage(voltage);

    // Get current PWM value for offset calculation
    double current_adjust = starshotObj.rtY.point[2] * 5.0;
    int PWM = current2PWM(current_adjust);

    // Calculate soft iron offsets for each axis using MFF
    std::vector<float> xOffsets = magneticFieldFilter.calculateOffsets(PWM, "x");
    std::vector<float> yOffsets = magneticFieldFilter.calculateOffsets(PWM, "y");
    std::vector<float> zOffsets = magneticFieldFilter.calculateOffsets(PWM, "z");

    // Filter magnetic field values using EKF
    Eigen::Vector3d measuredValues(
        mag.magnetic.x - xOffsets[0],
        mag.magnetic.y - yOffsets[1],
        mag.magnetic.z - zOffsets[2]
    );
    Eigen::Vector3d gyroValues(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    Eigen::Vector3d filteredValues = magneticFieldFilter.filter(measuredValues, gyroValues);

    // Apply hard iron offsets and assign filtered values to Starshot ACS
    starshotObj.rtU.Bfield_body[0] = filteredValues(0) - mag_hardiron_x;
    starshotObj.rtU.Bfield_body[1] = filteredValues(2) - mag_hardiron_z;
    starshotObj.rtU.Bfield_body[2] = -filteredValues(1) - mag_hardiron_y;

    // Calibrating and assigning gyro values
    float gyro_hardiron_x = 0.0728;
    float gyro_hardiron_y = 0.0202;
    float gyro_hardiron_z = 0.0160;

    starshotObj.rtU.w[0] = gyro.gyro.x - gyro_hardiron_x;
    starshotObj.rtU.w[1] = gyro.gyro.z - gyro_hardiron_z;
    starshotObj.rtU.w[2] = -gyro.gyro.y - gyro_hardiron_y;

    starshotObj.step();

    // Apply ACS output
    ACSWrite(current_adjust);

    // Log data
    double IMUData[7] = {
        starshotObj.rtY.pt_error,
        current_adjust,
        (double)PWM,
        starshotObj.rtU.Bfield_body[0],
        starshotObj.rtU.Bfield_body[2],
        starshotObj.rtU.Bfield_body[1],
        voltage
    };
    DataLog(IMUData, 7, file_name);
    Serial.println("Data logged.");

    unsigned long end = millis();
    delay(100 - (end - start));
}

/*/
void loop()
{

  unsigned long start = millis();

  sensors_event_t accel, mag, gyro, temp;
  imu.getEvent(&accel, &mag, &gyro, &temp);

  int pwm_y = 0; // will be re-assigned to the previous PWM value, previous PWM value needs to be used to calculate the current offset

  // Remap axis(rotate around x-axis by 90 deg)

  // Calibrating and assigning magnetometer values
  float mag_hardiron_x = -4.9547000000000025;
  float mag_hardiron_y = 49.75155;
  float mag_hardiron_z = -13.855600000000003;

  float pwmY_ox_1 = 8.83096680e-03;
  float pwmY_ox_2 = 4.26409072e-07;
  float pwmY_ox_3 = -6.69370023e-09;
  float pwmY_oy_1 = -2.64514092e-01;
  float pwmY_oy_2 = -9.82458813e-06;
  float pwmY_oy_3 = 9.11136691e-08;
  float pwmY_oz_1 = -1.90567242e-02;
  float pwmY_oz_2 = -5.99945842e-06;
  float pwmY_oz_3 = 7.85718685e-10;

  // Change all naming to oop-flight-code naming
  float pwmY_ox = (pwmY_ox_1 * pwm_y) + (pwmY_ox_2 * pow(pwm_y, 2)) + (pwmY_ox_3 * pow(pwm_y, 3));
  float pwmY_oy = (pwmY_oy_1 * pwm_y) + (pwmY_oy_2 * pow(pwm_y, 2)) + (pwmY_oy_3 * pow(pwm_y, 3));
  float pwmY_oz = (pwmY_oz_1 * pwm_y) + (pwmY_oz_2 * pow(pwm_y, 2)) + (pwmY_oz_3 * pow(pwm_y, 3));

  starshotObj.rtU.Bfield_body[0] = mag.magnetic.x - mag_hardiron_x - pwmY_ox;
  starshotObj.rtU.Bfield_body[1] = mag.magnetic.z - mag_hardiron_z - pwmY_oz;
  starshotObj.rtU.Bfield_body[2] = -mag.magnetic.y - mag_hardiron_y - pwmY_oy;

  // Calibrating and assigning gyro values
  float gyro_hardiron_x = 0.07280736884261114;
  float gyro_hardiron_y = 0.020224269122947534;
  float gyro_hardiron_z = 0.016019223067681217;

  starshotObj.rtU.w[0] = gyro.gyro.x - gyro_hardiron_x;
  starshotObj.rtU.w[1] = gyro.gyro.z - gyro_hardiron_z;
  starshotObj.rtU.w[2] = -gyro.gyro.y - gyro_hardiron_y;

  // Getting battery voltage
  int voltage_value_pin = 23;
  float voltage_ref = 3.3;
  int resolution = 1023;
  int r1 = 4700;
  int r2 = 10000;

  float voltage = analogRead(voltage_value_pin) * voltage_ref / resolution * (r1 + r2) / r2;

  // starshotObj.rtU.w[0] = gyro.gyro.x;
  // starshotObj.rtU.w[1] = gyro.gyro.z;
  // starshotObj.rtU.w[2] = -gyro.gyro.y;

  // starshotObj.rtU.Bfield_body[0] = mag.magnetic.x;
  // starshotObj.rtU.Bfield_body[1] = mag.magnetic.z;
  // starshotObj.rtU.Bfield_body[2] = -mag.magnetic.y;
  starshotObj.step();

  // test bench current adjust due to high B field
  double current_adjust = starshotObj.rtY.point[2] * 5.0;
  ACSWrite(current_adjust);

  // data
  int PWM = current2PWM(current_adjust);
  pwm_y = PWM;
  double IMUData[7] = {starshotObj.rtY.pt_error, current_adjust, PWM, starshotObj.rtU.Bfield_body[0], starshotObj.rtU.Bfield_body[2], starshotObj.rtU.Bfield_body[1], voltage};
  DataLog(IMUData, 7, file_name);
  Serial.println("data logged");

  unsigned long end = millis();
  delay(100 - (end - start));
}//*/
```
