/**
*
* @file finalproject.c
*
* @author Christopher Dean (cdean@pdx.edu)
* @author Meng Lei (lmeng@pdx.edu)
*
* This program is the main c program for our final project: Object Detection Using Sound Localization. 
* It takes 2 microphone inputs and detects the phase difference from the same sound source, then output
* a pwm signal to a servo which then points to the corresponding direction that that sound is coming
* from. The hardware for PWM is done with a Xilinx Timer/Counter module set in PWM mode. The PWM library
* builds on the Timer/Counter drivers provided by Xilinx and encapsulates common PWM functions. The
* program also uses a Xilinx fixed interval timer module to generate a periodic interrupt for handling
* time-based (maybe) and/or sampled inputs/outputs.
*
* @note
* The minimal hardware configuration for this test is a Microblaze-based system with 32KB of memory,
* an instance of Nexys4IO, an instance of the PMod544IOR2, an instance of an axi_timer, an instance of an axi_gpio
* and an instance of an axi_uartlite (used for xil_printf() console output)
*
******************************************************************************/

/************************ Include Files **************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xparameters.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "xgpio.h"
#include "mb_interface.h"
#include "platform.h"
#include "Nexys4IO.h"
#include "PMod544IOR2.h"
#include "pwm_tmrctr.h"


/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// PWM and pulse detect timer parameters
#define PWM_TIMER_DEVICE_ID		XPAR_TMRCTR_0_DEVICE_ID

// Nexys4IO parameters
#define NX4IO_DEVICE_ID			XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR			XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR			XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Pmod544IO parameters
#define PMDIO_DEVICE_ID			XPAR_PMOD544IOR2_0_DEVICE_ID
#define PMDIO_BASEADDR			XPAR_PMOD544IOR2_0_S00_AXI_BASEADDR
#define PMDIO_HIGHADDR			XPAR_PMOD544IOR2_0_S00_AXI_HIGHADDR

// GPIO parameters
#define GPIO_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_1_DEVICE_ID		XPAR_AXI_GPIO_1_DEVICE_ID
#define GPIO_INPUT_CHANNEL		1
#define GPIO_OUTPUT_CHANNEL		2									
		
// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR
#define PWM_TIMER_INTERRUPT_ID	XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40	

// Neutral frequency and duty cycle for servo
#define SERVO_NEUTRAL_FREQ	50	// 50Hz neutral frequency
#define SERVO_NEUTRAL_DUTY	7	// 7% neutral duty cycle

#define	PWM_SIGNAL_MSK			0x01
#define CLKFIT_MSK				0x01
#define PWM_FREQ_MSK			0x03
#define PWM_DUTY_MSK			0xFF

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/************************** Variable Definitions ****************************/	
// Microblaze peripheral instances
XIntc 	IntrptCtlrInst;						// Interrupt Controller instance
XTmrCtr	PWMTimerInst;						// PWM timer instance
XGpio	GPIOInst;							// GPIO 0 instance
XGpio	GPIO_1_Inst;						// GPIO 1 instance

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
// "clkfit" toggles each time the FIT interrupt handler is called so its frequency will
// be 1/2 FIT_CLOCK_FREQ_HZ.  timestamp increments every 1msec and is used in delay_msecs()
volatile unsigned int	clkfit;				// clock signal is bit[0] (rightmost) of gpio 0 output port									
volatile unsigned long  timestamp;			// timestamp since the program began
volatile u32			gpio_in;			// GPIO input port

// The following variables are shared between the functions in the program
// such that they must be global
int						pwm_freq;			// PWM frequency 
int						pwm_duty;			// PWM duty cycle
bool					new_perduty;		// new period/duty cycle flag
int						phase_diff = 0;		// phase difference between signal 1 and 2, in clock count


				
/*---------------------------------------------------------------------------*/					
int						debugen = 0;		// debug level/flag
/*---------------------------------------------------------------------------*/
		
/*****************************************************************************/	
	

/************************** Function Prototypes ******************************/
int				do_init(void);											// initialize system
void			delay_msecs(unsigned int msecs);						// busy-wait delay for "msecs" miliseconds
void			voltstostrng(float v, char* s);							// converts volts to a string
void			update_lcd(int freq, int dutyccyle, u32 linenum);		// update LCD display
				
void			FIT_Handler(void);										// fixed interval timer interrupt handler


/************************** MAIN PROGRAM ************************************/
int main()
{
	XStatus 	status;
	bool		done = false;
		
	init_platform();

	// initialize devices and set up interrupts, etc.
 	status = do_init();
 	xil_printf("Device Initialization Success\n\r");
 	if (status != XST_SUCCESS)
 	{
 		xil_printf("Device Initialization Failed\n\r");
 		exit(XST_FAILURE);
 	}
 	
	// initialize the global variables
	clkfit = 0;
	
	// There's no new period/duty to output to pwm
	new_perduty = false;
    
	// set the initial servo position to neutral
	pwm_freq = SERVO_NEUTRAL_FREQ;
	pwm_duty = SERVO_NEUTRAL_DUTY;

	// start the PWM timer and kick of the processing by enabling the Microblaze interrupt
	PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	PWM_Start(&PWMTimerInst);
    microblaze_enable_interrupts();
    delay_msecs(50);
	// display the greeting   
    xil_printf("Greetings!\n\r");
    
	// Set up old phase variable
	// It's compared to the new phase difference
	// If new phase difference is different, then update the corresponding pwm parameters
	int old_phase_diff = 0;
		
    // main loop
	do
	{
			// If new phase diff is different than old
			if (phase_diff != old_phase_diff)
			{
				// calculate the corresponding pwm duty cycle.
				// phase diff can vary from -2500 to +2500, we need to limit the duty cycle to
				// 7%(+-4%)
				pwm_duty = phase_diff * 4 / 25000 + SERVO_NEUTRAL_DUTY;
				
				// update the old_phase_diff for next comparison				
				old_phase_diff = phase_diff;
				
				// set flag to update pwm parameters
				new_perduty = true;
			}
			
			// else there's no new parameters to be updated
			else new_perduty = false;
		
			// update generated frequency and duty cycle
			if (new_perduty)
			{
				// set the new PWM parameters - PWM_SetParams stops the timer
				status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
				PWM_Start(&PWMTimerInst);
				delay_msecs(1000);
				xil_printf("pwm output successful\n\r");
				
				// pwm parameters updated. wait for next comparison
				new_perduty = false;
			}
	} while (!done);
	
	
	// we're done,  say goodbye
	xil_printf("\nThat's All Folks!\n\n");
	delay_msecs(5000);
	cleanup_platform();
	exit(0);
 }


/**************************** HELPER FUNCTIONS ******************************/
		
/****************************************************************************/
/**
* initialize the system
* 
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/
int do_init(void)
{
	int status;				// status from Xilinx Lib calls
	
	// initialize the Nexys4IO and Pmod544IO hardware and drivers
	// rotary encoder is set to increment from 0 by DUTY_CYCLE_CHANGE 
 	status = NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	status = PMDIO_initialize(PMDIO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	
	// initialize the GPIO instance
	status = XGpio_Initialize(&GPIOInst, GPIO_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 8-bit input port.  bit[7:1] = reserved, bit[0] = PWM output (for duty cycle calculation)
	// GPIO channel 2 is an 8-bit output port.  bit[7:1] = reserved, bit[0] = FIT clock
	XGpio_SetDataDirection(&GPIOInst, GPIO_OUTPUT_CHANNEL, 0xFE);
	
	// initialize the GPIO 1 instance
	status = XGpio_Initialize(&GPIO_1_Inst, GPIO_1_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 32-bit input port time1.
	// GPIO channel 2 is an 32-bit input port time2.
	XGpio_SetDataDirection(&GPIO_1_Inst, 1, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIO_1_Inst, 2, 0xFFFFFFFF);
			
	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts.  Clock frequency is the AXI clock frequency
	status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false, AXI_CLOCK_FREQ_HZ);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
    if (status != XST_SUCCESS)
    {
       return XST_FAILURE;
    }

	// connect the fixed interval timer (FIT) handler to the interrupt
    status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
                           (XInterruptHandler)FIT_Handler,
                           (void *)0);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }
 
	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
    status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

	// enable the FIT interrupt
    XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);

	return XST_SUCCESS;
}
		

/****************************************************************************/
/**
* delay execution for "n" msecs
* 
* Uses a busy-wait loop to delay execution.  Timing is approximate but we're not 
*  looking for precision here, just a uniform delay function.  The function uses the 
*  global "timestamp" which is incremented every msec by FIT_Handler().
*
* @note
* Assumes that this loop is running faster than the fit_interval ISR 
*
* @note
* If your program seems to hang it could be because the function never returns
* Possible causes for this are almost certainly related to the FIT timer.  Check
* your connections...is the timer clocked?  is it stuck in reset?  is the interrupt 
* output connected? You would not be the first student to face this...not by a longshot 
*****************************************************************************/
void delay_msecs(unsigned int msecs)
{
	unsigned long target;

	if ( msecs == 0 )
	{
		return;
	}
	target = timestamp + msecs;
	while ( timestamp != target )
	{
		// spin until delay is over
	}
}

	
/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler 
*  
* updates the global "timestamp" every millisecond.  "timestamp" is used for the delay_msecs() function
* and as a time stamp for data collection and reporting.  Toggles the FIT clock which can be used as a visual
* indication that the interrupt handler is being called.  Also makes RGB1 a PWM duty cycle indicator
*
* @note
* ECE 544 students - When you implement your software solution for pulse width detection in
* Project 1 this could be a reasonable place to do that processing.
 *****************************************************************************/
void FIT_Handler(void)
{
		
	static int ts_interval = 0;			// interval counter for incrementing timestamp
	u32 time1_count = 0;			// signal 1 posedge counter
    u32 time2_count = 0;			// signal 2 posedge counter
	u32 diff = 0;					// slock count difference between 2 signals
	int direction;					// indicator of which signal is ahead
	
	// Read timestamp1 and timestamp2 from GPIO 1
	time1_count = XGpio_DiscreteRead(&GPIO_1_Inst, 1);
	time2_count = XGpio_DiscreteRead(&GPIO_1_Inst, 2);
	

	// toggle FIT clock
	clkfit ^= 0x01;
	XGpio_DiscreteWrite(&GPIOInst, GPIO_OUTPUT_CHANNEL, clkfit);	

	// update timestamp	
	ts_interval++;	
	if (ts_interval > FIT_COUNT_1MSEC)
	{
		timestamp++;
		ts_interval = 1;
	}

	// Compare them to see which leads and by how much
	// If the count between them is valid, update global "difference"
	if(time1_count > time2_count)
	{
		// See if phase difference is valid
		if(time1_count - time2_count > 25000) return;
		
		// Valid, signal 1 is ahead
		direction = 1;
		
		// Calculate phase difference
		diff = time1_count - time2_count;
		
		// Combine phase difference with direction
		phase_diff = (int)diff * direction;
	}
	
	else if(time2_count > time1_count)
	{
		// See if phase difference is valid
		if(time2_count - time1_count > 25000) return;
		
		// Valid, signal 2 is ahead
		direction = -1;
		
		// Calculate the phase difference
		diff = time2_count - time1_count;
		
		// Combine phase difference with direction
		phase_diff = (int)diff * direction;
	}
	
}	
