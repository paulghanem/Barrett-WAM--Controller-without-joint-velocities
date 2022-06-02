/*
 * back_stoch_data.c
 *
 * Code generation for model "back_stoch".
 *
 * Model version              : 1.35
 * Simulink Coder version : 8.8 (R2015a) 09-Feb-2015
 * C source code generated on : Fri Mar 03 01:08:01 2017
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "back_stoch.h"
#include "back_stoch_private.h"

/* Block parameters (auto storage) */
P_back_stoch_T back_stoch_P = {
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay2'
                                        */

  /*  Expression: diag([1e-8 1e-10 1e-5 1e-9 1e-9 8e-9 1e-10 1e-10 1e-10 1e-10 1e-10 1e-10 1e-10 1e-10])
   * Referenced by: '<Root>/IC3'
   */
  { 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0E-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0E-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0E-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 8.0E-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0E-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0E-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0E-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0E-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-10 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay1'
                                        */

  /*  Expression: [0 -1.9 -0.1 3 0.2 0 -0.1]
   * Referenced by: '<Root>/IC1'
   */
  { 0.0, -1.9, -0.1, 3.0, 0.2, 0.0, -0.1 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay4'
                                        */

  /*  Expression: [0 0 0 0 0 0 0]
   * Referenced by: '<Root>/IC2'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /*  Expression: [0 0 0 0 0 0 0]
   * Referenced by: '<Root>/Unit Delay3'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
};
