/*
 * back_stoch.h
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

#ifndef RTW_HEADER_back_stoch_h_
#define RTW_HEADER_back_stoch_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <float.h>
#ifndef back_stoch_COMMON_INCLUDES_
# define back_stoch_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* back_stoch_COMMON_INCLUDES_ */

#include "back_stoch_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T U[784];
  real_T posd[7];                      /* '<Root>/posd ' */
  real_T fposd[7];                     /* '<Root>/fposd ' */
  real_T veld[7];                      /* '<Root>/veld' */
  real_T faccd[7];                     /* '<Root>/faccd' */
  real_T IC3[196];                     /* '<Root>/IC3' */
  real_T IC1[7];                       /* '<Root>/IC1' */
  real_T IC2[7];                       /* '<Root>/IC2' */
  real_T DigitalClock;                 /* '<Root>/Digital Clock' */
  real_T UnitDelay3[7];                /* '<Root>/Unit Delay3' */
  real_T u[7];                         /* '<Root>/Backstepping_Stochastic' */
} B_back_stoch_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay2_DSTATE[196];       /* '<Root>/Unit Delay2' */
  real_T UnitDelay1_DSTATE[7];         /* '<Root>/Unit Delay1' */
  real_T UnitDelay4_DSTATE[7];         /* '<Root>/Unit Delay4' */
  real_T UnitDelay3_DSTATE[7];         /* '<Root>/Unit Delay3' */
  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } posd_PWORK;                        /* '<Root>/posd ' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } fposd_PWORK;                       /* '<Root>/fposd ' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } veld_PWORK;                        /* '<Root>/veld' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } faccd_PWORK;                       /* '<Root>/faccd' */

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<Root>/To Workspace1' */

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                /* '<Root>/To Workspace2' */

  struct {
    void *LoggedData;
  } ToWorkspace3_PWORK;                /* '<Root>/To Workspace3' */

  struct {
    void *LoggedData;
  } ToWorkspace4_PWORK;                /* '<Root>/To Workspace4' */

  uint32_T method;                     /* '<Root>/Backstepping_Stochastic' */
  uint32_T state;                      /* '<Root>/Backstepping_Stochastic' */
  uint32_T state_n[2];                 /* '<Root>/Backstepping_Stochastic' */
  uint32_T state_j[625];               /* '<Root>/Backstepping_Stochastic' */
  struct {
    int_T PrevIndex;
  } posd_IWORK;                        /* '<Root>/posd ' */

  struct {
    int_T PrevIndex;
  } fposd_IWORK;                       /* '<Root>/fposd ' */

  struct {
    int_T PrevIndex;
  } veld_IWORK;                        /* '<Root>/veld' */

  struct {
    int_T PrevIndex;
  } faccd_IWORK;                       /* '<Root>/faccd' */

  boolean_T IC3_FirstOutputTime;       /* '<Root>/IC3' */
  boolean_T IC1_FirstOutputTime;       /* '<Root>/IC1' */
  boolean_T IC2_FirstOutputTime;       /* '<Root>/IC2' */
  boolean_T state_not_empty;           /* '<Root>/Backstepping_Stochastic' */
} DW_back_stoch_T;

/* Parameters (auto storage) */
struct P_back_stoch_T_ {
  real_T UnitDelay2_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay2'
                                        */
  real_T IC3_Value[196];               /* Expression: diag([1e-8 1e-10 1e-5 1e-9 1e-9 8e-9 1e-10 1e-10 1e-10 1e-10 1e-10 1e-10 1e-10 1e-10])
                                        * Referenced by: '<Root>/IC3'
                                        */
  real_T UnitDelay1_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay1'
                                        */
  real_T IC1_Value[7];                 /* Expression: [0 -1.9 -0.1 3 0.2 0 -0.1]
                                        * Referenced by: '<Root>/IC1'
                                        */
  real_T UnitDelay4_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay4'
                                        */
  real_T IC2_Value[7];                 /* Expression: [0 0 0 0 0 0 0]
                                        * Referenced by: '<Root>/IC2'
                                        */
  real_T UnitDelay3_InitialCondition[7];/* Expression: [0 0 0 0 0 0 0]
                                         * Referenced by: '<Root>/Unit Delay3'
                                         */
};

/* Real-time Model Data Structure */
struct tag_RTM_back_stoch_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (auto storage) */
extern P_back_stoch_T back_stoch_P;

/* Block signals (auto storage) */
extern B_back_stoch_T back_stoch_B;

/* Block states (auto storage) */
extern DW_back_stoch_T back_stoch_DW;

/* Model entry point functions */
extern void back_stoch_initialize(void);
extern void back_stoch_step(void);
extern void back_stoch_terminate(void);

/* Real-time Model object */
extern RT_MODEL_back_stoch_T *const back_stoch_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'back_stoch'
 * '<S1>'   : 'back_stoch/Backstepping_Stochastic'
 */
#endif                                 /* RTW_HEADER_back_stoch_h_ */
