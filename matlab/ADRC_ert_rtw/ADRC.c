/*
 * File: ADRC.c
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

#include "ADRC.h"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_ADRC_T ADRC_DW;

/* Real-time model */
static RT_MODEL_ADRC_T ADRC_M_;
RT_MODEL_ADRC_T *const ADRC_M = &ADRC_M_;

/* Model step function */
void ADRC_step(void)
{
  real_T rtb_Gain1;
  real_T rtb_Gain2_i;
  real_T rtb_Gain2_o;
  real_T rtb_Integrator;
  real_T rtb_Subtract;

  /* DiscreteIntegrator: '<S1>/Integrator' */
  rtb_Integrator = ADRC_DW.Integrator_DSTATE;

  /* Gain: '<S10>/Gain2' incorporates:
   *  UnitDelay: '<S10>/Unit Delay'
   */
  rtb_Gain2_i = 1.0 * ADRC_DW.UnitDelay_DSTATE[0] + ADRC_DW.UnitDelay_DSTATE[1] *
    0.001;
  rtb_Gain2_o = ADRC_DW.UnitDelay_DSTATE[0] * 0.0 + ADRC_DW.UnitDelay_DSTATE[1] *
    1.0;

  /* Sum: '<S10>/Subtract' incorporates:
   *  Gain: '<S10>/Gain'
   *  UnitDelay: '<S10>/Unit Delay'
   *  ZeroOrderHold: '<S13>/Zero-Order Hold'
   */
  rtb_Subtract = rtb_Integrator - (1.0 * ADRC_DW.UnitDelay_DSTATE[0] + 0.0 *
    ADRC_DW.UnitDelay_DSTATE[1]);

  /* Sum: '<S10>/Add' incorporates:
   *  Gain: '<S10>/Gain1'
   *  Gain: '<S10>/Gain3'
   *  UnitDelay: '<S10>/Unit Delay'
   */
  ADRC_DW.UnitDelay_DSTATE[0] += 0.18126924692201829 * rtb_Subtract;
  ADRC_DW.UnitDelay_DSTATE[1] += 9.0559170060627228 * rtb_Subtract;

  /* Step: '<Root>/Step' */
  if (((ADRC_M->Timing.clockTick0) * 0.001) < 0.0) {
    rtb_Gain1 = 0.0;
  } else {
    rtb_Gain1 = 1.0;
  }

  /* Gain: '<S9>/Gain1' incorporates:
   *  Gain: '<S9>/Gain'
   *  Step: '<Root>/Step'
   *  Sum: '<S9>/Sum'
   *  Sum: '<S9>/Sum1'
   */
  rtb_Gain1 = ((rtb_Gain1 - ADRC_DW.UnitDelay_DSTATE[0]) * 10.0 -
               ADRC_DW.UnitDelay_DSTATE[1]) * 1.0;

  /* Gain: '<S10>/Gain4' incorporates:
   *  Gain: '<S10>/Gain3'
   *  UnitDelay: '<S10>/Unit Delay'
   */
  ADRC_DW.UnitDelay_DSTATE[0] = 0.19032516392808102 * rtb_Subtract;

  /* Sum: '<S10>/Add1' incorporates:
   *  Gain: '<S10>/Gain3'
   *  UnitDelay: '<S10>/Unit Delay'
   */
  ADRC_DW.UnitDelay_DSTATE[0] = (0.001 * rtb_Gain1 + ADRC_DW.UnitDelay_DSTATE[0])
    + rtb_Gain2_i;

  /* Gain: '<S10>/Gain4' incorporates:
   *  Gain: '<S10>/Gain3'
   *  UnitDelay: '<S10>/Unit Delay'
   */
  ADRC_DW.UnitDelay_DSTATE[1] = 9.0559170060627228 * rtb_Subtract;

  /* Sum: '<S10>/Add1' incorporates:
   *  Gain: '<S10>/Gain3'
   *  UnitDelay: '<S10>/Unit Delay'
   */
  ADRC_DW.UnitDelay_DSTATE[1] = (0.0 * rtb_Gain1 + ADRC_DW.UnitDelay_DSTATE[1])
    + rtb_Gain2_o;

  /* Update for DiscreteIntegrator: '<S1>/Integrator' */
  ADRC_DW.Integrator_DSTATE += 0.001 * rtb_Gain1;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.001, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  ADRC_M->Timing.clockTick0++;
}

/* Model initialize function */
void ADRC_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void ADRC_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
