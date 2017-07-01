/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		 PROJECT: Frame Processing

 				            ********* ENHANCE. C **********
							 Shell for speech enhancement 

  		Demonstrates overlap-add frame processing (interrupt driven) on the DSK. 

 *************************************************************************************
 				             By Danny Harvey: 21 July 2006
							 Updated for use on CCS v4 Sept 2010
 ************************************************************************************/
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
volatile int enable = 0;
volatile int enhance = 0;
volatile int option = 1;
volatile int counter = 0;
float *m1,*m2,*m3,*m4,*mag,*noisemag,*temp,*g,*p,*p_prev,*p2,*p2_prev,*noisemag_prev;
float ALPHA=5;
float LAMBDA=0.01;
float TAU = 0.02;
float lpf_const;
 /******************************* Function prototypes *******************************/
void init_hardware(void);    	/* Initialize codec */ 
void init_HWI(void);            /* Initialize hardware interrupts */
void ISR_AIC(void);             /* Interrupt service routine for codec */
void process_frame(void);       /* Frame processing routine */
float max(float x, float y);          
void original();
void enhance_1();
void enhance_2();
void enhance_3();
void enhance_4();
void enhance_5();
void enhance_6();
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
	g			= (float *) calloc(FFTLEN, sizeof(float));
	p			= (float *) calloc(FFTLEN, sizeof(float));
	p_prev		= (float *) calloc(FFTLEN, sizeof(float));
	p2			= (float *) calloc(FFTLEN, sizeof(float));
	p2_prev		= (float *) calloc(FFTLEN, sizeof(float));
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
	
	/************************* DO PROCESSING OF FRAME  HERE **************************/
	
	switch(enhance)
	{
		case 0: original();
				break;
		case 1: enhance_1();
				break;
		case 2: enhance_2();
				break;
		case 3: enhance_3();
				break;
		case 4: enhance_4();
				break;
		case 5: enhance_5();
				break;
		case 6: enhance_6();
				break;
		default:original();
	}					
//    for (k=0;k<FFTLEN;k++)
//	{                           
//		outframe[k].r = g[i]*inframe[k].r;/* copy input straight into output */ 
//		outframe[k].i = g[i]*inframe[k].i;
//	} 
//		
	/********************************************************************************/
	
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
/*************************** INTERRUPT SERVICE ROUTINE  *****************************/

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

/************************************************************************************/
float max(float x, float y)
{
	if(x>y)
		return x;
	else
		return y;
}

void original()
{
	float noiseratio;
	int i;
	fft(FFTLEN,inframe);
	if(counter==0)
	{
		temp = m4;
		m4 = m3;
		m3 = m2;
		m2 = m1;
		m1 = temp;
		for(i=0;i<FFTLEN;i++)
		{
			noisemag[i] = m1[i];
			if(noisemag[i]>m2[i])
				noisemag[i] = m2[i];
			if(noisemag[i]>m3[i])
				noisemag[i] = m3[i];
			if(noisemag[i]>m4[i])
				noisemag[i] = m4[i];
			noisemag[i] = ALPHA*noisemag[i];
		}
	}
	for(i=0;i<FFTLEN;i++)
	{
		mag[i] = cabs(inframe[i]);
		if(counter==0)
		{
			m1[i] = mag[i];
		}
		if(m1[i]>mag[i])
			m1[i] = mag[i];
		noiseratio = noisemag[i]/mag[i];
		g[i] = max(LAMBDA,1-noiseratio);
		outframe[i].r = g[i]*inframe[i].r;
		outframe[i].i = g[i]*inframe[i].i;
	}				      	
	ifft(FFTLEN,outframe);	
}

void enhance_1()
{
	float noiseratio;
	int i;
	fft(FFTLEN,inframe);
	if(counter==0)
	{
		temp = m4;
		m4 = m3;
		m3 = m2;
		m2 = m1;
		m1 = temp;
		for(i=0;i<FFTLEN;i++)
		{
			noisemag[i] = m1[i];
			if(noisemag[i]>m2[i])
				noisemag[i] = m2[i];
			if(noisemag[i]>m3[i])
				noisemag[i] = m3[i];
			if(noisemag[i]>m4[i])
				noisemag[i] = m4[i];
			noisemag[i] = ALPHA*noisemag[i];
		}
	}
	for(i=0;i<FFTLEN;i++)
	{
		mag[i] = cabs(inframe[i]);
		if(counter==0)
		{
			m1[i] = mag[i];
			p_prev = p;
			p[i] = mag[i];
		}
		p[i] = (1-lpf_const)*mag[i] + lpf_const*p_prev[i];
		mag[i] = p[i];
		if(m1[i]>mag[i])
			m1[i] = mag[i];
		noiseratio = noisemag[i]/mag[i];
		g[i] = max(LAMBDA,1-noiseratio);
		outframe[i].r = g[i]*inframe[i].r;
		outframe[i].i = g[i]*inframe[i].i;
	}				      	
	ifft(FFTLEN,outframe);	
}

void enhance_2()
{
	float noiseratio;
	int i;
	fft(FFTLEN,inframe);
	if(counter==0)
	{
		temp = m4;
		m4 = m3;
		m3 = m2;
		m2 = m1;
		m1 = temp;
		for(i=0;i<FFTLEN;i++)
		{
			noisemag[i] = m1[i];
			if(noisemag[i]>m2[i])
				noisemag[i] = m2[i];
			if(noisemag[i]>m3[i])
				noisemag[i] = m3[i];
			if(noisemag[i]>m4[i])
				noisemag[i] = m4[i];
			noisemag[i] = ALPHA*noisemag[i];
		}
	}
	for(i=0;i<FFTLEN;i++)
	{
		mag[i] = cabs(inframe[i]);
		if(counter==0)
		{
			m1[i] = mag[i];
			p2_prev = p2;
			p2[i] = mag[i]*mag[i];
		}
		p2[i] = (1-lpf_const)*mag[i]*mag[i] + lpf_const*p2_prev[i];
		mag[i] = sqrt(p2[i]);
		if(m1[i]>mag[i])
			m1[i] = mag[i];
		noiseratio = noisemag[i]/mag[i];
		g[i] = max(LAMBDA,1-noiseratio);
		outframe[i].r = g[i]*inframe[i].r;
		outframe[i].i = g[i]*inframe[i].i;
	}				      	
	ifft(FFTLEN,outframe);	
}

void enhance_3()
{
	float noiseratio;
	int i;
	fft(FFTLEN,inframe);
	if(counter==0)
	{
		temp = m4;
		m4 = m3;
		m3 = m2;
		m2 = m1;
		m1 = temp;
		noisemag_prev = noisemag;
		for(i=0;i<FFTLEN;i++)
		{
			noisemag[i] = m1[i];
			if(noisemag[i]>m2[i])
				noisemag[i] = m2[i];
			if(noisemag[i]>m3[i])
				noisemag[i] = m3[i];
			if(noisemag[i]>m4[i])
				noisemag[i] = m4[i];
			noisemag[i] = ALPHA*noisemag[i];
			noisemag[i] = (1-lpf_const)*noisemag[i]+lpf_const*noisemag_prev[i];
		}
	}
	for(i=0;i<FFTLEN;i++)
	{
		mag[i] = cabs(inframe[i]);
		if(counter==0)
		{
			m1[i] = mag[i];
			p2_prev = p2;
			p2[i] = mag[i]*mag[i];
		}
		p2[i] = (1-lpf_const)*mag[i]*mag[i] + lpf_const*p2_prev[i];
		mag[i] = sqrt(p2[i]);
		if(m1[i]>mag[i])
			m1[i] = mag[i];
		noiseratio = noisemag[i]/mag[i];
		g[i] = max(LAMBDA,1-noiseratio);
		outframe[i].r = g[i]*inframe[i].r;
		outframe[i].i = g[i]*inframe[i].i;
	}				      	
	ifft(FFTLEN,outframe);	
}
void enhance_4()
{
	float noiseratio;
	int i;
	fft(FFTLEN,inframe);
	if(counter==0)
	{
		temp = m4;
		m4 = m3;
		m3 = m2;
		m2 = m1;
		m1 = temp;
		p2_prev = p2;
		noisemag_prev = noisemag;
		for(i=0;i<FFTLEN;i++)
		{
			noisemag[i] = m1[i];
			if(noisemag[i]>m2[i])
				noisemag[i] = m2[i];
			if(noisemag[i]>m3[i])
				noisemag[i] = m3[i];
			if(noisemag[i]>m4[i])
				noisemag[i] = m4[i];
			noisemag[i] = ALPHA*noisemag[i];
			noisemag[i] = (1-lpf_const)*noisemag[i]+lpf_const*noisemag_prev[i];
		}
	}
	for(i=0;i<FFTLEN;i++)
	{
		mag[i] = cabs(inframe[i]);
		if(counter==0)
		{
			m1[i] = mag[i];
			p2[i] = mag[i]*mag[i];
		}
		p2[i] = (1-lpf_const)*mag[i]*mag[i] + lpf_const*p2_prev[i];
		mag[i] = sqrt(p2[i]);
		if(m1[i]>mag[i])
			m1[i] = mag[i];
		noiseratio = noisemag[i]/mag[i];
		switch(option)
		{
			case 1: g[i] = max(LAMBDA,1-noiseratio);
					break;
			case 2: g[i] = max(LAMBDA*noiseratio,1-noiseratio);
					break;
			case 3: g[i] = max(LAMBDA*p2[i]/mag[i],1-noiseratio);
					break;
			case 4: g[i] = max(LAMBDA*noisemag[i]/p2[i],1-noisemag[i]/p2[i]);
					break;
			case 5: g[i] = max(LAMBDA,1-noisemag[i]/p2[i]);
					break;
			case 6: g[i] = max(LAMBDA,sqrt(1-noiseratio*noiseratio));
					break;
			default:g[i] = max(LAMBDA,1-noiseratio);			
		}
		outframe[i].r = g[i]*inframe[i].r;
		outframe[i].i = g[i]*inframe[i].i;
	}				      	
	ifft(FFTLEN,outframe);	
}

void enhance_5()
{	
	float noiseratio;
	int i;
	fft(FFTLEN,inframe);
	if(counter==0)
	{
		temp = m4;
		m4 = m3;
		m3 = m2;
		m2 = m1;
		m1 = temp;
		for(i=0;i<FFTLEN;i++)
		{
			noisemag[i] = m1[i];
			if(noisemag[i]>m2[i])
				noisemag[i] = m2[i];
			if(noisemag[i]>m3[i])
				noisemag[i] = m3[i];
			if(noisemag[i]>m4[i])
				noisemag[i] = m4[i];
			noisemag[i] = ALPHA*noisemag[i];
			noisemag[i] = (1-lpf_const)*noisemag[i];
		}
	}
	for(i=0;i<FFTLEN;i++)
	{
		mag[i] = cabs(inframe[i]);
		if(counter==0)
		{
			m1[i] = mag[i];
			p2_prev = p2;
			p2[i] = mag[i]*mag[i];
		}
		p2[i] = (1-lpf_const)*mag[i]*mag[i] + lpf_const*p2_prev[i];
		mag[i] = sqrt(p2[i]);
		if(m1[i]>mag[i])
			m1[i] = mag[i];
		noiseratio = noisemag[i]/mag[i];
		g[i] = max(LAMBDA,sqrt(1-noiseratio*noiseratio));
		outframe[i].r = g[i]*inframe[i].r;
		outframe[i].i = g[i]*inframe[i].i;
	}				      	
	ifft(FFTLEN,outframe);	
}

void enhance_6()
{
	float noiseratio;
	int i;
	float rejection = ALPHA;
	fft(FFTLEN,inframe);
	if(counter==0)
	{
		temp = m4;
		m4 = m3;
		m3 = m2;
		m2 = m1;
		m1 = temp;
		p2_prev = p2;
		noisemag_prev = noisemag;
		for(i=0;i<FFTLEN;i++)
		{
			noisemag[i] = m1[i];
			if(noisemag[i]>m2[i])
				noisemag[i] = m2[i];
			if(noisemag[i]>m3[i])
				noisemag[i] = m3[i];
			if(noisemag[i]>m4[i])
				noisemag[i] = m4[i];
			if(((i<3)||(i>32))&&(i<128))
				rejection = 30*ALPHA;
			else if(((i<160)||(i>253))&&(i>128))
				rejection = 30*ALPHA;
			else
				rejection = ALPHA;
			noisemag[i] = rejection*noisemag[i];
			noisemag[i] = (1-lpf_const)*noisemag[i]+lpf_const*noisemag_prev[i];
		}
	}
	for(i=0;i<FFTLEN;i++)
	{
		mag[i] = cabs(inframe[i]);
		if(counter==0)
		{
			m1[i] = mag[i];
			p2[i] = mag[i]*mag[i];
		}
		p2[i] = (1-lpf_const)*mag[i]*mag[i] + lpf_const*p2_prev[i];
		mag[i] = sqrt(p2[i]);
		if(m1[i]>mag[i])
			m1[i] = mag[i];
		noiseratio = noisemag[i]/mag[i];
		switch(option)
		{
			case 1: g[i] = max(LAMBDA,1-noiseratio);
					break;
			case 2: g[i] = max(LAMBDA*noiseratio,1-noiseratio);
					break;
			case 3: g[i] = max(LAMBDA*p2[i]/mag[i],1-noiseratio);
					break;
			case 4: g[i] = max(LAMBDA*noisemag[i]/p2[i],1-noisemag[i]/p2[i]);
					break;
			case 5: g[i] = max(LAMBDA,1-noisemag[i]/p2[i]);
					break;
			case 6: g[i] = max(LAMBDA,sqrt(1-noiseratio*noiseratio));
					break;
			default:g[i] = max(LAMBDA,1-noiseratio);			
		}
		outframe[i].r = g[i]*inframe[i].r;
		outframe[i].i = g[i]*inframe[i].i;
	}				      	
	ifft(FFTLEN,outframe);	
}
