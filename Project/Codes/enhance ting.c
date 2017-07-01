/**************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		 PROJECT: Frame Processing

 				            ********* ENHANCE. C **********
							 Shell for speech enhancement 

  		Demonstrates overlap-add frame processing (interrupt driven) on the DSK. 

 ***************************************************************
 				             By Danny Harvey: 21 July 2006
							 Updated for use on CCS v4 Sept 2010
 ***************************************************************/
/*
 *	You should modify the code so that a speech enhancement project is built 
 *  on top of this template.
 */
/**************************** Pre-processor statements ******************************/
//  library required when using calloc
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

/* Some functions to help with Complex algebra and FFT. */
#include "cmplx.h"      
#include "fft_functions.h"  

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

#define WINCONST 0.85185			/* 0.46/0.54 for Hamming window */
#define FSAMP 8000.0		/* sample frequency, ensure this matches Config for AIC */
#define FFTLEN 256					/* fft length = frame length 256/8000 = 32 ms*/
#define NFREQ (1+FFTLEN/2)			/* number of frequency bins from a real FFT */
#define OVERSAMP 4					/* oversampling ratio (2 or 4) */  
#define FRAMEINC (FFTLEN/OVERSAMP)	/* Frame increment */
#define CIRCBUF (FFTLEN+FRAMEINC)	/* length of I/O buffers */

#define OUTGAIN 16000.0				/* Output gain for DAC */
#define INGAIN  (1.0/16000.0)		/* Input gain for ADC  */
// PI defined here for use in your code 
#define PI 3.141592653589793
#define TFRAME FRAMEINC/FSAMP       /* time between calculation of each frame */

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
    0x008d,  /* 8 SAMPLERATE Sample rate control        8 KHZ-ensure matches FSAMP */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};

// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

float *inbuffer, *outbuffer;   		/* Input/output circular buffers */
complex *inframe, *outframe;          /* Input and output frames */
float *inwin, *outwin;              /* Input and output windows */
float ingain, outgain;				/* ADC and DAC gains */ 
float cpufrac; 						/* Fraction of CPU time used */
volatile int io_ptr=0;              /* Input/ouput pointer for circular buffers */
volatile int frame_ptr=0;           /* Frame pointer */
volatile int enable = 1;
volatile int counter = 0;
float *m1,*m2,*m3,*m4,*mag,*noisemag,*temp,*p2,*noisemag_prev;
float ALPHA=1;
float LAMBDA=0.005;
float TAU = 0.08;
float lpf_const,threshold=5,gain=3;
float SNR=0,g;
 /******************************* Function prototypes *******************************/
void init_hardware(void);    	/* Initialize codec */ 
void init_HWI(void);            /* Initialize hardware interrupts */
void ISR_AIC(void);             /* Interrupt service routine for codec */
void process_frame(void);       /* Frame processing routine */
float max(float x, float y);
//float magnitude(complex x);
void enhance();
/********************************** Main routine ************************************/
void main()
{      

  	int k; // used in various for loops
  	lpf_const = exp(-TFRAME/TAU);
/*  Initialize and zero fill arrays */  

	inbuffer	= (float *) calloc(CIRCBUF, sizeof(float));	/* Input array */
    outbuffer	= (float *) calloc(CIRCBUF, sizeof(float));	/* Output array */
	inframe		= (complex *) calloc(FFTLEN, sizeof(complex));	/* Array for processing*/
    outframe	= (complex *) calloc(FFTLEN, sizeof(complex));	/* Array for processing*/
    inwin		= (float *) calloc(FFTLEN, sizeof(float));	/* Input window */
    outwin		= (float *) calloc(FFTLEN, sizeof(float));	/* Output window */
	m1			= (float *) calloc(FFTLEN, sizeof(float));
	m2			= (float *) calloc(FFTLEN, sizeof(float));
	m3 			= (float *) calloc(FFTLEN, sizeof(float));
	m4 			= (float *) calloc(FFTLEN, sizeof(float));
	mag 		= (float *) calloc(FFTLEN, sizeof(float));
	noisemag	= (float *) calloc(FFTLEN, sizeof(float));
	temp		= (float *) calloc(FFTLEN, sizeof(float));
	p2			= (float *) calloc(FFTLEN, sizeof(float));
	noisemag_prev = (float *) calloc(FFTLEN, sizeof(float));
	
	/* initialize board and the audio port */
  	init_hardware();
  
  	/* initialize hardware interrupts */
  	init_HWI();    
  
/* initialize algorithm constants */  
                       
  	for (k=0;k<FFTLEN;k++)
	{                           
	inwin[k] = sqrt((1.0-WINCONST*cos(PI*(2*k+1)/FFTLEN))/OVERSAMP);
	outwin[k] = inwin[k];
	} 
  	ingain=INGAIN;
  	outgain=OUTGAIN;        

 							
  	/* main loop, wait for interrupt */  
  	while(1) 	process_frame();
}
    
/********************************** init_hardware() *********************************/  
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

	/* These commands do the same thing as above but applied to data transfers to the 
	audio port */
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
        
/******************************** process_frame() ***********************************/  
void process_frame(void)
{
	int m, i; 
	int io_ptr0;  
	/* work out fraction of available CPU time used by algorithm */    
	cpufrac = ((float) (io_ptr & (FRAMEINC - 1)))/FRAMEINC;  
		
	/* wait until io_ptr is at the start of the current frame */ 	
	while((io_ptr/FRAMEINC) != frame_ptr); 
	counter++;
	//if 312 frames have been processed, then roughly 2.5 seconds have passed
	//each frame is 8ms. 0.008*312 = 2.5 to 1 decimal point.
	if(counter == 312)
		counter = 0;
	/* then increment the framecount (wrapping if required) */ 
	if (++frame_ptr >= (CIRCBUF/FRAMEINC)) frame_ptr=0;
 	
 	/* save a pointer to the position in the I/O buffers (inbuffer/outbuffer) where the 
 	data should be read (inbuffer) and saved (outbuffer) for the purpose of processing */
 	io_ptr0=frame_ptr * FRAMEINC;
	
	/* copy input data from inbuffer into inframe (starting from the pointer position) */ 
	 
	m=io_ptr0;
    for (i=0;i<FFTLEN;i++)
	{                           
		inframe[i] = cmplx(inbuffer[m] * inwin[i],0); 
		if (++m >= CIRCBUF) m=0; /* wrap if required */
	} 
	
	/************************* DO PROCESSING OF FRAME  HERE *******/
	
	enhance();			

	/**************************************************************/
	
    /* multiply outframe by output window and overlap-add into output buffer */  
                           
	m=io_ptr0;
    for (i=0;i<(FFTLEN-FRAMEINC);i++) 
	{    										/* this loop adds into outbuffer */                       
	  	outbuffer[m] = outbuffer[m]+(outframe[i].r)*outwin[i];   
		if (++m >= CIRCBUF) m=0; /* wrap if required */
	}         
    for (;i<FFTLEN;i++) 
	{                           
		outbuffer[m] = (outframe[i].r)*outwin[i];   /* this loop over-writes outbuffer */        
	    m++;
	}                                  
}        
/*************************** INTERRUPT SERVICE ROUTINE  ************/

// Map this to the appropriate interrupt in the CDB file
   
void ISR_AIC(void)
{       
	short sample;
	/* Read and write the ADC and DAC using inbuffer and outbuffer */
	
	sample = mono_read_16Bit();
	inbuffer[io_ptr] = ((float)sample)*ingain;
		/* write new output data */
	if(enable==0)		
		mono_write_16Bit((short)(sample));
	else
		mono_write_16Bit((short)(outbuffer[io_ptr]*outgain)); 
	/* update io_ptr and check for buffer wraparound */    
	
	if (++io_ptr >= CIRCBUF) io_ptr=0;
}

/******************************************************************/
//function returning the larger of two floats
float max(float x, float y)
{
	if(x>y)
		return x;
	else
		return y;
}

void enhance()
{
	int i;
	//fast fourier transform the discrete-time input signal
	fft(FFTLEN,inframe);
	//store previous noise magnitudes
	noisemag_prev = noisemag;
	//check if 2.5 seconds have passed, and therefore buffers are filled
	if(counter==0)
	{
		//shuffle buffers
		temp = m4;
		m4 = m3;
		m3 = m2;
		m2 = m1;
		m1 = temp;
		//assign m1 as maximum float value to take in new values
		for(i=0;i<FFTLEN;i++)
		{
			m1[i] = FLT_MAX;
		}
	}
	for(i=0;i<FFTLEN;i++)
	{
		//calculate magnitude of input spectrum
		mag[i] = cabs(inframe[i]);
		//calculate the minimum estimated noise in each frequency bin over the past 10 seconds
		noisemag[i] = m1[i];
		if(noisemag[i]>m2[i])
			noisemag[i] = m2[i];
		if(noisemag[i]>m3[i])
			noisemag[i] = m3[i];
		if(noisemag[i]>m4[i])
			noisemag[i] = m4[i];
		//low-pass filter noise magnitude
		noisemag[i] = (1-lpf_const)*noisemag[i]+lpf_const*noisemag_prev[i];
		//low-pass filter magnitude in power domain			
		p2[i] = (1-lpf_const)*mag[i]*mag[i] + lpf_const*p2[i];
		//assign magnitude as square root
		mag[i] = sqrt(p2[i]);
		//assign the minimum magnitude between current and past inputs
		if(m1[i]>mag[i])
			m1[i] = mag[i];
		//vary oversampling factor depending on frequency bin
		if((i<30)||(i>FFTLEN-30))
		{
			//calculate SNR
			SNR = (mag[i]/noisemag[i]);
			//vary oversampling factor depending on SNR
			if(SNR>threshold)
				noisemag[i] = ALPHA*noisemag[i];
			else
				noisemag[i] = ALPHA*gain*noisemag[i];
		}
		else
		{
			//penalise high frequency bins
			noisemag[i] = ALPHA*gain*gain*noisemag[i];
		}
		//calculate g
		g = max(LAMBDA*p2[i]/mag[i],1-noisemag[i]/mag[i]);
		//calculate output
		outframe[i].r = g*inframe[i].r;
		outframe[i].i = g*inframe[i].i;
	}				     
	//inverse fourier transform	 	
	ifft(FFTLEN,outframe);	
}
