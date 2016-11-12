`timescale 1ns / 1ps
/******************************************************/
// n4fpga.v - Top level module for the ECE 544 Final Project
//
//
// Created By:	Chris Dean, Meng Lei
// Date:		3/16/16
// Version:		1.0
//
// Description:
// ------------
// This module provides the top level for the final project hardware.
// The module assumes that the two amplified microphone signals come from
// JD[0] and JD[1]; and the pwm output to the servo is at JC[0].
//
// It creates an instance of Phase_Detection outside of the system EMBSYS which
// reads from the microphone inputs and outputs 2 time stamps of the rising edges
// of the signals, which are then used to calculate the phase difference and
// direction.
//
// Most of this module is the same as n4fpga.v provided for Getting Started, 
// with the main changes being the instance of Phase_Detection (at the bottom) 
// and a few wires to make the GPIO connections.
/******************************************************/
module n4fpga(
    input				clk,			// 100Mhz clock input
    input				btnC,			// center pushbutton
    input				btnU,			// UP (North) pusbhbutton
    input				btnL,			// LEFT (West) pushbutton
    input				btnD,			// DOWN (South) pushbutton  - used for system reset
    input				btnR,			// RIGHT (East) pushbutton
	input				btnCpuReset,	// CPU reset pushbutton
    input	[15:0]		sw,				// slide switches on Nexys 4
    output	[15:0] 		led,			// LEDs on Nexys 4   
    output              RGB1_Blue,      // RGB1 LED (LD16) 
    output              RGB1_Green,
    output              RGB1_Red,
    output              RGB2_Blue,      // RGB2 LED (LD17)
    output              RGB2_Green,
    output              RGB2_Red,
    output [7:0]        an,             // Seven Segment display
    output [6:0]        seg,
    output              dp,
    
    input				uart_rtl_rxd,	// USB UART Rx and Tx on Nexys 4
    output				uart_rtl_txd,	
    
    output	[7:0] 		JA,				// JA Pmod connector - PmodCLP data bus
										// both rows are used
    output	[7:0] 		JB,				// JB Pmod connector - PmodCLP control signals
										// only the bottom row is used
    output	[7:0] 		JC,				// JC Pmod connector - debug signals
										// only the bottom row is used
	input	[7:0]		JD				// JD Pmod connector - PmodENC signals
);

// internal variables
wire				sysclk;
wire				sysreset_n, sysreset;
wire				rotary_a, rotary_b, rotary_press, rotary_sw;
wire	[7:0]		lcd_d;
wire				lcd_rs, lcd_rw, lcd_e;

wire	[7:0]	gpio_in;				// embsys GPIO input port
wire	[7:0]	gpio_out;				// embsys GPIO output port
wire            pwm_out;                // PWM output from the axi_timer

// make the connections to the GPIO port.  Most of the bits are unused in the Getting
// Started project but GPIO's provide a convenient way to get the inputs and
// outputs from logic you create to and from the Microblaze.  For example,
// you may decide that using an axi_gpio peripheral is a good way to interface
// your hardware pulse-width detect logic with the Microblaze.  Our application
// is simple.

// The FIT interrupt routine synthesizes a 20KHz signal and makes it
// available on GPIO2[0].  Bring it to the top level as an
// indicator that the system is running...could be handy for debug
wire            clk_20khz;
assign clk_20khz = gpio_out[0];

/*********************************************************************
Below are the system port additions for hardware pulse-width detect. 
We need to wire the clk_out2 and feed it into the HW_PWD clock input,
and the two 32-bit outputs from the HW_PWD module need to go back into
the GPIO 1 system inputs.
*********************************************************************/

// Signals for final project
wire    clk2;
wire    [31:0] time_1;  // Timestamp of signal 1's posedge
wire    [31:0] time_2;  // Timestamp of signal 2's posedge
wire    signal_1, signal_2;  // Input pulses from mic amplifier

// make the connections
assign signal_2 = JD[1];
assign signal_1 = JD[0];
assign JC = {7'b0, pwm_out};

// system-wide signals
assign sysclk = clk;
assign sysreset_n = btnCpuReset;		// The CPU reset pushbutton is asserted low.  The other pushbuttons are asserted high
										// but the microblaze for Nexys 4 expects reset to be asserted low
assign sysreset = ~sysreset_n;			// Generate a reset signal that is asserted high for any logic blocks expecting it.

// instantiate the embedded system
system EMBSYS
       (.PmodCLP_DataBus(lcd_d),
        .PmodCLP_E(lcd_e),
        .PmodCLP_RS(lcd_rs),
        .PmodCLP_RW(lcd_rw),
        .PmodENC_A(rotary_a),
        .PmodENC_B(rotary_b),
        .PmodENC_BTN(rotary_press),
        .PmodENC_SWT(rotary_sw),
        .RGB1_Blue(RGB1_Blue),
        .RGB1_Green(RGB1_Green),
        .RGB1_Red(RGB1_Red),
        .RGB2_Blue(RGB2_Blue),
        .RGB2_Green(RGB2_Green),
        .RGB2_Red(RGB2_Red),
        .an(an),
        .btnC(btnC),
        .btnD(btnD),
        .btnL(btnL),
        .btnR(btnR),
        .btnU(btnU),
        .dp(dp),
        .led(led_int),
        .seg(seg),
        .sw(sw),
        .sysreset_n(sysreset_n),
        .sysclk(sysclk),
        .uart_rtl_rxd(uart_rtl_rxd),
        .uart_rtl_txd(uart_rtl_txd),
        .gpio_0_GPIO2_tri_o(gpio_out),
        .gpio_0_GPIO_tri_i(gpio_in),
        .pwm0(pwm_out),

        // These are the added signals for final project hardware solution
        .clk2(clk2),
        .time_1_tri_i(time_1),
        .time_2_tri_i(time_2));

// Instance of hardware phase detection module
Phase_Detection Hardware_detect
    (.signal_1(signal_1), 
    .signal_2(signal_2), 
    .clock(clk2), 
    .time_1(time_1),
    .time_2(time_2));

endmodule

