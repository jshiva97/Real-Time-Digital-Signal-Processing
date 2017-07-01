	//fast fourier transform the discrete-time input signal (time to frequency domain)
	fft(FFTLEN,inframe);
	
	//inverse fourier transform	(frequency to time domain)      	
	ifft(FFTLEN,outframe);	