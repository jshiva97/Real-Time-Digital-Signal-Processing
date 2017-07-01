//Interrupt service routine implementing a fourth order IIR filter
void ISR_AIC()
{
	int i;
	double input = mono_read_16Bit();//read input value

	x[0] = x[1] + b[0]*input;//assign output
	for (i=1;i<=N;i++)
	{
		x[i] = x[i+1] + (b[i]*input)-(a[i]*x[0]);//perform all-zero and all-pole accumulations
	}
	mono_write_16Bit((short)x[0]);//write to output
}

