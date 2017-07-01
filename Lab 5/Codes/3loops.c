void ISR_AIC()
{
	int i;
	double y=0;//initialise output
	double input = mono_read_16Bit();//read input value
	for (i=N;i>0;i--)
	{
		x[i] = x[i-1];//shift samples
	}
	x[0] = input;
	for (i=1;i<=N;i++)
	{
		x[0] -= x[i]*a[i];//all-pole accumulation
	}

	for (i=0;i<=N;i++)
	{
		y += b[i]*x[i];//all-zero accumulation
	}
	mono_write_16Bit((short)y);//write to output
}
