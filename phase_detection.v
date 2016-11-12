/******************************************************/
// MODULE: Phase_Detection 
//
// FILE NAME:	phase_detection.v
// VERSION: 1.0
// DATE:	3/16/16
// AUTHOR:	Chris Dean, Meng Lei
//
// DESCRIPTION:	
// Custom module for phase detection hardware used in
// ECE 544 final project.
//
/******************************************************/


// MODULE
module Phase_Detection(clock, signal_1, signal_2, time_1, time_2);

	input clock;				           	// Reference clock
	input signal_1;				        	// Input signal from mic1
	input signal_2;                        // Input signal from mic2
	output reg [31:0] time_1, time_2;      // Measured arrival timestamp 

	reg [31:0] counter;						// local timestamp counter
	
	// These 2 variables are needed because it's generally not allowed to
	// perform edge detection in standard i/o.
	// Used to store "old" signal levels in order to achieve "edge detection"
	reg prev_1, prev_2;

	// Initialize values to zero
	initial
	begin
	   counter = 0;
	   time_1 = 0;
	   time_2 = 0;
	   prev_1 = 0;
	   prev_2 = 0;
	end
	
	// On each clock tick sample the signal and update counters
	always @(posedge clock)
	begin
        // Increment counter
        if (counter < 0'hFFFFFFFF)
            counter = counter + 1;
       
        // If counter is at max, reset to zero
        else
          counter = 0;
          
        
		// Detect signal 1 rising edge
		if ((prev_1 == 0) && (signal_1 == 1))
			// Record the time stamp
            time_1 = counter;
		
		// Store current timestamp for next comparison
        prev_1 = signal_1;
			
		// Detect signal 2 rising edge	
        if ((prev_2 == 0) && (signal_2 == 1))
			//Record the time stamp
            time_2 = counter; 
			
		// Store current timestamp for next comparison
        prev_2 = signal_2; 
          
	end


endmodule




