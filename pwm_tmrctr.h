/**
*
* @file pwm_tmrctr.h
*
* @author Roy Kravitz (roy.kravitz@pdx.edu)
* @copyright Portland State University, 2014-2015, 2016
*
* This file contain the constand definitions and function prototypes for pwm_tmrctr.c.  
* pwm_tmrctrc.c provides an API for Pulse-width modulation using the Xilinx timer/counter (tmrctr IP).  The API is * provided because the Xilinx timer/counter
* driver does not support PWM mode.  This driver is based on the * high-level driver model supported by Xilinx and borrows/adapts code 
* from the tmrctr driver source code.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a	rhk	12/20/14	First release of driver for Vivado/Nexys4
* </pre>
*
******************************************************************************/
 
#ifndef PWM_TMRCTR_H	/* prevent circular inclusions */
#define PWM_TMRCTR_H	/* by using protection macros */

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include "stdbool.h"
#include "math.h"
#include "xil_types.h"
#include "xstatus.h"
#include "xparameters.h"
#include "xtmrctr.h"

/************************** Constant Definitions *****************************/
#define PWM_TIMER_WIDTH		32
#define PWM_MAXCNT			4294967295.00

#define PWM_PERIOD_TIMER	0
#define PWM_DUTY_TIMER		1

/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/
int PWM_Initialize(XTmrCtr *InstancePtr, u16 DeviceId, bool EnableInterrupts, u32 clkfreq);
int PWM_Start(XTmrCtr *InstancePtr);
int PWM_Stop(XTmrCtr *InstancePtr);
int PWM_SetParams(XTmrCtr *InstancePtr, u32 freq, u32 dutyfactor);
int PWM_GetParams(XTmrCtr *InstancePtr, u32 *freq, u32 *dutyfactor);

/************************** Variable Definitions *****************************/

#ifdef __cplusplus
}
#endif

#endif /* end of protection macro */
