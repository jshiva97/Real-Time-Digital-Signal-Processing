/********************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		  LAB 3: Interrupt I/O

 				            ********* I N T I O. C **********

  Demonstrates inputing and outputing data from the DSK's audio port using interrupts. 

 **********************************************************************
 				Updated for use on 6713 DSK by Danny Harvey: May-Aug 2006
				Updated for CCS V4 Sept 10
 *********************************************************************/
/*
 *	You should modify the code so that interrupts are used to service the 
 *  audio port.
 */
/**************************** Pre-processor statements ******************************/

#include <stdlib.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// math library (trig functions)
#include <math.h>

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>
#include "fir_coef.txt"

#define N sizeof(b)/sizeof(float)-1			//customisable order of filter
float x[N+1];								//input buffer
float output,sample;						//initialisation of global variables
int ptr=0;									//circular buffer input pointer
int mid;									//middle element of input buffer
/******************************* Global declarations ********************************/

/* Audio port configuration settings: these values set registers in the AIC23 audio 
   interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
			 /**********************************************************************/
			 /*   REGISTER	            FUNCTION			      SETTINGS         */ 
			 /**********************************************************************/\
    0x0017,  /* 0 LEFTINVOL  Left line input channel volume  0dB                   */\
    0x0017,  /* 1 RIGHTINVOL Right line input channel volume 0dB                   */\
    0x01f9,  /* 2 LEFTHPVOL  Left channel headphone volume   0dB                   */\
    0x01f9,  /* 3 RIGHTHPVOL Right channel headphone volume  0dB                   */\
    0x0011,  /* 4 ANAPATH    Analog audio path control       DAC on, Mic boost 20dB*/\
    0x0000,  /* 5 DIGPATH    Digital audio path control      All Filters off       */\
    0x0000,  /* 6 DPOWERDOWN Power down control              All Hardware on       */\
    0x0043,  /* 7 DIGIF      Digital audio interface format  16 bit                */\
    0x008d,  /* 8 SAMPLERATE Sample rate control             8 KHZ                 */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};


// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

 /******************************* Function prototypes ********************************/
void init_hardware(void);     
void init_HWI(void);                  
void ISR_AIC(void);
void circ_FIR();		//function that implements FIR
/********************************** Main routine ************************************/
void main(){      
	
	if(N+1%2==0)				//middle element calculation for odd and even cases
	{							//as loop counter limit
		mid = (N-1)/2;
	}
	else
	{
		mid = (N-2)/2;
	}
	// initialize board and the audio port
	init_hardware();
		
	/* initialize hardware interrupts */
	init_HWI();
	  	 		
	/* loop indefinitely, waiting for interrupts */  					
	while(1) 
	{};
  
}
        
/********************************** init_hardware() **********************************/  
void init_hardware()
{
    // Initialize the board support library, must be called first 
    DSK6713_init();
    
    // Start the AIC23 codec using the settings defined above in config 
    H_Codec = DSK6713_AIC23_openCodec(0, &Config);

	/* Function below sets the number of bits in word used by MSBSP (serial port) for 
	receives from AIC23 (audio port). We are using a 32 bit packet containing two 
	16 bit numbers hence 32BIT is set for  receive */
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);	

	/* Configures interrupt to activate on each consecutive available 32 bits 
	from Audio port hence an interrupt is generated for each L & R sample pair */	
	MCBSP_FSETS(SPCR1, RINTM, FRM);

	/* These commands do the same thing as above but applied to data transfers to  
	the audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);	
	MCBSP_FSETS(SPCR1, XINTM, FRM);	
	

}

/********************************** init_HWI() **************************************/  
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt (used by the debugger)
	IRQ_map(IRQ_EVT_RINT1,4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts
} 

/******************** WRITE YOUR INTERRUPT SERVICE ROUTINE HERE***********************/  

void ISR_AIC()
{
	sample = mono_read_16Bit();			//takes input sample
	output=0;							//resetting output to zero for accumulation
	circ_FIR();
	mono_write_16Bit((short)floor(output));		//writing to output
}

void circ_FIR()					//circular buffer implementation of FIR with symmetry and wrap prediction
{
	int i,j,k,wrap,diff;
	x[ptr] = sample;			//sample input into index ptr
	i = ptr;					//index pointer that moves left (to lower indices)
	j = ptr+1;					//index pointer that moves right (to higher indices)
	if((N-j)<=i)				//check to if i wraps around
	{
		wrap = 0;				//wrap set according to check
		diff = N-j;				//difference between ptr and N
	}
	else						//check to if j wraps around
	{
		wrap = 1;				//wrap set according to check
		diff = i;				//difference between ptr and 0
	}
	if(wrap==0)
	{
		if(diff>=mid)			//case of ptr in the middle
		{
			for(k=0;k<=mid;k++)	//MAC implementation
			{
				output += b[k]*(x[i--]+x[j++]);			
			}
		}
		else
		{
			for(k=0;k<=diff;k++) //MAC implementation upto wrap around
			{
				output += b[k]*(x[i--]+x[j++]);			
			}
			j=0;				//wrap around
			for(k=diff+1;k<=mid;k++)	//MAC implementation till middle element
			{
				output += b[k]*(x[i--]+x[j++]);			
			}
		}
		if(N+1%2!=0)			//handling of middle element in case of odd number of taps
		{
			output += b[mid+1]*x[i--];
		}
	}
	else
	{
		if(diff>=mid)			//case of ptr in the middle
		{
			for(k=0;k<=mid;k++)	//MAC implementation
			{
				output += b[k]*(x[i--]+x[j++]);			
			}
		}
		else
		{
			for(k=0;k<=diff;k++)	//MAC implementation upto wrap around
			{
				output += b[k]*(x[i--]+x[j++]);			
			}
			i=N;				//wrap around
			for(k=diff+1;k<=mid;k++)	//MAC implementation till middle element
			{	
				output += b[k]*(x[i--]+x[j++]);			
			}
		}
		if(N+1%2!=0)
		{
			output += b[mid+1]*x[j++];	//handling of middle element in case of odd number of taps
		}
	}	
	if(ptr==N)				//conditional statement to check wrap around of ptr
	{
		ptr=0;
	}
	else				
	{		
		ptr++;				//incrementation of ptr
	}			
}