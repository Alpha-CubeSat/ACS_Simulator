//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Plantv50.h
//
// Code generated for Simulink model 'Plantv50'.
//
// Model version                  : 1.79
// Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
// C/C++ source code generated on : Fri Oct 22 23:44:40 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_Plantv50_h_
#define RTW_HEADER_Plantv50_h_
#include <stddef.h>
#include <cmath>
#include <math.h>
#ifndef Plantv50_COMMON_INCLUDES_
# define Plantv50_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // Plantv50_COMMON_INCLUDES_

// Macros for accessing real-time model data structure

// Block signals and states (default storage) for system '<Root>'
typedef struct {
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
} DWPlant;

// Constant parameters (default storage)
typedef struct {
  // Expression: starshot.IC.massproperties.I
  //  Referenced by: '<S3>/Constant'

  real_T Constant_Value[9];

  // Expression: starshot.IC.massproperties.Iinv
  //  Referenced by: '<S3>/Constant1'

  real_T Constant1_Value[9];

  // Expression: eye(3,3)
  //  Referenced by: '<S7>/Matrix Gain'

  real_T MatrixGain_Gain[9];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T current[3];                   // '<Root>/current'
} ExtUPlant;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T angularvelocity[3];           // '<Root>/angular velocity'
  real_T magneticfield[3];             // '<Root>/magnetic field'
  real_T xyzposition[3];               // '<Root>/xyzposition'
  real_T quaternion[4];                // '<Root>/quaternion'
} ExtYPlant;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model Plantv50
class Plantv50ModelClass {
  // public data and function members
 public:
  // External inputs
  ExtUPlant rtU;

  // External outputs
  ExtYPlant rtY;

  // model initialize function
  void initialize(float DiscreteTimeIntegrator_DSTATE, float DiscreteTimeIntegrator1_DSTAT_l, float DiscreteTimeIntegrator2_DSTATE);

  // model step function
  void step();

  // Constructor
  Plantv50ModelClass();

  // Destructor
  ~Plantv50ModelClass();

  // private data and function members
 private:
  // Block signals and states
  DWPlant rtDW;
};

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
//  hilite_system('Plantv5/Plantv5')    - opens subsystem Plantv5/Plantv5
//  hilite_system('Plantv5/Plantv5/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'Plantv5'
//  '<S1>'   : 'Plantv5/Plantv5'
//  '<S2>'   : 'Plantv5/Plantv5/Cross Product'
//  '<S3>'   : 'Plantv5/Plantv5/Dynamics'
//  '<S4>'   : 'Plantv5/Plantv5/Magnetic Field Model'
//  '<S5>'   : 'Plantv5/Plantv5/Quaternion Integration'
//  '<S6>'   : 'Plantv5/Plantv5/Tranlational Dynamics'
//  '<S7>'   : 'Plantv5/Plantv5/qtoQ'
//  '<S8>'   : 'Plantv5/Plantv5/Dynamics/Cross Product'
//  '<S9>'   : 'Plantv5/Plantv5/Magnetic Field Model/Dipole->ECI'
//  '<S10>'  : 'Plantv5/Plantv5/Quaternion Integration/q_normalize'
//  '<S11>'  : 'Plantv5/Plantv5/Quaternion Integration/qderiv'
//  '<S12>'  : 'Plantv5/Plantv5/qtoQ/Subsystem'

#endif                                 // RTW_HEADER_Plantv50_h_

//
// File trailer for generated code.
//
// [EOF]
//
