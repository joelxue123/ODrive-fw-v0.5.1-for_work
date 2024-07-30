/*
 * File: ADRC.h
 *
 * Code generated for Simulink model 'ADRC'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Thu May 30 18:04:55 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_ADRC_h_
#define RTW_HEADER_ADRC_h_
#ifndef ADRC_COMMON_INCLUDES_
#define ADRC_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ADRC_COMMON_INCLUDES_ */

#include "ADRC_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE[2];          /* '<S10>/Unit Delay' */
  real_T Integrator_DSTATE;            /* '<S1>/Integrator' */
} DW_ADRC_T;

/* Real-time Model Data Structure */
struct tag_RTM_ADRC_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
  } Timing;
};

/* Block states (default storage) */
extern DW_ADRC_T ADRC_DW;

/* Model entry point functions */
extern void ADRC_initialize(void);
extern void ADRC_step(void);
extern void ADRC_terminate(void);

/* Real-time Model object */
extern RT_MODEL_ADRC_T *const ADRC_M;

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
 * '<Root>' : 'ADRC'
 * '<S1>'   : 'ADRC/Subsystem'
 * '<S2>'   : 'ADRC/Subsystem/Active Disturbance Rejection Control'
 * '<S3>'   : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC'
 * '<S4>'   : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/error feedback'
 * '<S5>'   : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/extended state observer'
 * '<S6>'   : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/input saturation'
 * '<S7>'   : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/rZOH'
 * '<S8>'   : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/yZOH'
 * '<S9>'   : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/error feedback/first order'
 * '<S10>'  : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/extended state observer/discrete time'
 * '<S11>'  : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/input saturation/passthrough'
 * '<S12>'  : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/rZOH/enabled'
 * '<S13>'  : 'ADRC/Subsystem/Active Disturbance Rejection Control/ADRC/yZOH/enabled'
 */
#endif                                 /* RTW_HEADER_ADRC_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
