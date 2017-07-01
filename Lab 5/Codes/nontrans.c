void ISR_AIC()
{
	int i;
	double y=0; //initialising output variable
	double input = mono_read_16Bit(); //read input value

	x[0] = input; //initilaise output of first adder block	
	for (i=N;i>0;i--)
	{
		x[0] -= x[i]*a[i];//all-pole accumulation
		y += b[i]*x[i];//all-zero accumulation
		x[i] = x[i-1];//time shift
	}
	y += b[0]*x[0];//assign output
	mono_write_16Bit((short)y);//write to output
}