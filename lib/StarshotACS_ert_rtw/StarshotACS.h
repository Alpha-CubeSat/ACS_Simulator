//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: StarshotACS.h
//
// Code generated for Simulink model 'StarshotACS'.
//
// Model version                  : 1.77
// Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
// C/C++ source code generated on : Sat Oct 23 12:00:04 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_StarshotACS_h_
#define RTW_HEADER_StarshotACS_h_
#include <cmath>
#ifndef StarshotACS_COMMON_INCLUDES_
#define StarshotACS_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif // StarshotACS_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// Block signals and states (default storage) for system '<Root>'
typedef struct
{
  real_T DiscreteTimeIntegrator_DSTATE[3]; // '<S2>/Discrete-Time Integrator'
  real_T UD_DSTATE[3];                     // '<S5>/UD'
  real_T UD_DSTATE_k;                      // '<S7>/UD'
} DW;

// Invariant block signals (default storage)
typedef const struct tag_ConstB
{
  real_T VectorConcatenate[9]; // '<S10>/Vector Concatenate'
} ConstB;

// Constant parameters (default storage)
typedef struct
{
  // Expression: [1 0 0;0 1 0;0 0 1]
  //  Referenced by: '<S2>/Identity matrix'

  real_T Identitymatrix_Value[9];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct
{
  real_T w[3];           // '<Root>/angularvelocity'
  real_T Bfield_body[3]; // '<Root>/Bfield_body'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct
{
  real_T detumble[3]; // '<Root>/detumble'
  real_T point[3];    // '<Root>/point'
} ExtY;

// Real-time Model Data Structure
struct tag_RTM
{
  const char_T *volatile errorStatus;
};

extern const ConstB rtConstB; // constant block i/o

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model StarshotACS
class StarshotACSModelClass
{
  // public data and function members
public:
  real_T pointing_error;

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  StarshotACSModelClass();

  // Destructor
  ~StarshotACSModelClass();

  // Real-Time Model get method
  RT_MODEL *getRTM();

  // private data and function members
private:
  // Block signals and states
  DW rtDW;

  // Real-Time Model
  RT_MODEL rtM;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S5>/Data Type Duplicate' : Unused code path elimination
//  Block '<S7>/Data Type Duplicate' : Unused code path elimination
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
//  hilite_system('StarshotACS0/StarshotACS')    - opens subsystem StarshotACS0/StarshotACS
//  hilite_system('StarshotACS0/StarshotACS/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'StarshotACS0'
//  '<S1>'   : 'StarshotACS0/StarshotACS'
//  '<S2>'   : 'StarshotACS0/StarshotACS/KD_controller'
//  '<S3>'   : 'StarshotACS0/StarshotACS/Orientation controller'
//  '<S4>'   : 'StarshotACS0/StarshotACS/KD_controller/Cross'
//  '<S5>'   : 'StarshotACS0/StarshotACS/KD_controller/Discrete Derivative'
//  '<S6>'   : 'StarshotACS0/StarshotACS/KD_controller/Skew matrix S(m)1'
//  '<S7>'   : 'StarshotACS0/StarshotACS/Orientation controller/Discrete Derivative'
//  '<S8>'   : 'StarshotACS0/StarshotACS/Orientation controller/Norm2'
//  '<S9>'   : 'StarshotACS0/StarshotACS/Orientation controller/Norm3'
//  '<S10>'  : 'StarshotACS0/StarshotACS/Orientation controller/Skew matrix S(tau_c)1'
//  '<S11>'  : 'StarshotACS0/StarshotACS/Orientation controller/Subsystem1'
//  '<S12>'  : 'StarshotACS0/StarshotACS/Orientation controller/sign_function1'
//  '<S13>'  : 'StarshotACS0/StarshotACS/Orientation controller/Subsystem1/Norm4'
//  '<S14>'  : 'StarshotACS0/StarshotACS/Orientation controller/Subsystem1/Norm5'

#endif // RTW_HEADER_StarshotACS_h_

//
// File trailer for generated code.
//
// [EOF]
//
