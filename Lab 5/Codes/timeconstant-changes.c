//initialisations
double y[2] = {0};				//output buffer
double x[2] = {0};				//input buffer 

//Interrupt Service Routine
void ISR_AIC()
{
	double a1 = 0.05882353, a2 = 0.88235294; //locally specified coefficient values
	x[1] = x[0];							 //time shifting of current samples at index 0
	y[1] = y[0];
	x[0] = mono_read_16Bit();				//reading in current input
	y[0] = a1*(x[0]+x[1])+a2*y[1];			//calculation of current output
	mono_write_16Bit((short)y[0]);			//writing to output
}