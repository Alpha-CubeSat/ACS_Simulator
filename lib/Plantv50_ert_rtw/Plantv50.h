//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Plantv50.h
//
// Code generated for Simulink model 'Plantv50'.
//
// Model version                  : 10.28
// Simulink Coder version         : 9.6 (R2021b) 14-May-2021
// C/C++ source code generated on : Sun Jun  5 03:42:49 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_Plantv50_h_
#define RTW_HEADER_Plantv50_h_
#include <stddef.h>
#include <cmath>
#include "rtwtypes.h"

// Model Code Variants

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTMPlant RT_MODELPlant;

// Block signals and states (default storage) for system '<Root>'
struct DWPlant {
  real_T DiscreteTimeIntegrator1_DSTATE[4];// '<S5>/Discrete-Time Integrator1'
  real_T DiscreteTimeIntegrator_DSTATE;// '<S1>/Discrete-Time Integrator'
  real_T DiscreteTimeIntegrator1_DSTAT_l;// '<S1>/Discrete-Time Integrator1'
  real_T DiscreteTimeIntegrator2_DSTATE;// '<S1>/Discrete-Time Integrator2'
  real_T DiscreteTimeIntegrator1_DSTA_ls;// '<S6>/Discrete-Time Integrator1'
  real_T DiscreteTimeIntegrator2_DSTAT_e;// '<S6>/Discrete-Time Integrator2'
  real_T DiscreteTimeIntegrator3_DSTATE;// '<S6>/Discrete-Time Integrator3'
  real_T DiscreteTimeIntegrator4_DSTATE;// '<S6>/Discrete-Time Integrator4'
  real_T DiscreteTimeIntegrator5_DSTATE;// '<S6>/Discrete-Time Integrator5'
  real_T DiscreteTimeIntegrator6_DSTATE;// '<S6>/Discrete-Time Integrator6'
};

// Constant parameters (default storage)
struct ConstPPlant {
  // Expression: starshot.IC.massproperties.I
  //  Referenced by: '<S3>/Constant'

  real_T Constant_Value[9];

  // Expression: starshot.IC.massproperties.Iinv
  //  Referenced by: '<S3>/Constant1'

  real_T Constant1_Value[9];

  // Pooled Parameter (Expression: eye(3,3))
  //  Referenced by:
  //    '<S7>/Matrix Gain'
  //    '<S8>/Matrix Gain'

  real_T pooled5[9];
};

// External inputs (root inport signals with default storage)
struct ExtUplant {
  real_T current[3];                   // '<Root>/current'
};

// External outputs (root outports fed by signals with default storage)
struct ExtYplant {
  real_T angularvelocity[3];           // '<Root>/angular velocity'
  real_T magneticfield[3];             // '<Root>/magnetic field'
  real_T xyzposition[3];               // '<Root>/xyzposition'
  real_T quaternion[4];                // '<Root>/quaternion'
};

// Real-time Model Data Structure
struct tag_RTMPlant {
  const char_T * volatile errorStatus;
};

// Constant parameters (default storage)
extern const ConstPPlant rtConstPPlant;
extern "C" {
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"
  extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static void rt_InitInfAndNaN(size_t realSize);
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
  struct BigEndianIEEEDouble {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  };

  struct LittleEndianIEEEDouble {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  };

  struct IEEESingle {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  };
}                                      // extern "C"

extern "C" {
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"
  // Class declaration for model Plantv50
  class Plantv50ModelClass
{
  // public data and function members
 public:
  real_T Area;
  real_T Loops;
  real_T MagTorqAmpFac;
  // Real-Time Model get method
  RT_MODELPlant * getRTM();

  // External inputs
  ExtUplant rtU;

  // External outputs
  ExtYplant rtY;

  // model initialize function
  void initialize(float DiscreteTimeIntegrator_DSTATE, float DiscreteTimeIntegrator1_DSTAT_l, float DiscreteTimeIntegrator2_DSTATE, float quat0, float quat1, float quat2, float quat3, float altitude, float inclination, float csarea, float no_loops, float ampFactor);

  // model step function
  void step(float step_size);

  // Constructor
  Plantv50ModelClass();

  // Destructor
  ~Plantv50ModelClass();

  // private data and function members
 private:
  // Block states
  DWPlant rtDW;

  // private member function(s) for subsystem '<S1>/qtoQ'
  static void qtoQ(const real_T rtu_q[4], real_T rty_Q[9]);

  // Real-Time Model
  RT_MODELPlant rtM;
}

;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Scope' : Unused code path elimination


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
//  hilite_system('starshotsimv5/Plantv5')    - opens subsystem starshotsimv5/Plantv5
//  hilite_system('starshotsimv5/Plantv5/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'starshotsimv5'
//  '<S1>'   : 'starshotsimv5/Plantv5'
//  '<S2>'   : 'starshotsimv5/Plantv5/Cross Product'
//  '<S3>'   : 'starshotsimv5/Plantv5/Dynamics'
//  '<S4>'   : 'starshotsimv5/Plantv5/Magnetic Field Model'
//  '<S5>'   : 'starshotsimv5/Plantv5/Quaternion Integration'
//  '<S6>'   : 'starshotsimv5/Plantv5/Tranlational Dynamics'
//  '<S7>'   : 'starshotsimv5/Plantv5/qtoQ'
//  '<S8>'   : 'starshotsimv5/Plantv5/qtoQ1'
//  '<S9>'   : 'starshotsimv5/Plantv5/Dynamics/Cross Product'
//  '<S10>'  : 'starshotsimv5/Plantv5/Magnetic Field Model/Dipole->ECI'
//  '<S11>'  : 'starshotsimv5/Plantv5/Quaternion Integration/q_normalize'
//  '<S12>'  : 'starshotsimv5/Plantv5/Quaternion Integration/qderiv'
//  '<S13>'  : 'starshotsimv5/Plantv5/qtoQ/Subsystem'
//  '<S14>'  : 'starshotsimv5/Plantv5/qtoQ1/Subsystem'

#endif                                 // RTW_HEADER_Plantv50_h_

//
// File trailer for generated code.
//
// [EOF]
//
