void ISR_AIC()
{
	int i;
	double y=0;//initialise output
	double input = mono_read_16Bit();//read input values

	x[0] = input;
	for (i=1;i<=N;i++)
	{
		x[0] -= x[i]*a[i];//all-pole accumulation
	}
	for (i=N;i>0;i--)
	{
		y += b[i]*x[i];//all-zero accumulation
		x[i] = x[i-1];//shift samples
	}
	y += b[0]*x[0];//calculate output value
	mono_write_16Bit((short)y);//write to output
}
