%calculate filter coefficients
[N,Fo,Ao,W]=firpmord([375,410,1620,1655],[0,1,0],[0.005,0.023,0.005],8000); 
b = firpm(N,Fo,Ao,W);
a = 1;
a(end) = 1;

%calculate frequency and phase responses
[h,w] = freqz(b);
[phi,w] = phasez(b);

%plot amplitude response
figure
plot(w*4000/pi,20*log10(abs(h)));
xlabel('Frequency (Hz)');
ylabel('Gain (dB)');
title('Amplitude Response of the filter');
ax.XTick = 0:.5:2;
grid on
grid minor

%plot phase response
figure
plot(w*4000/pi,phi);
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title('Phase Response of the filter');
ax.XTick = 0:.5:2;
grid on
grid minor

%plot filter coefficients
figure
stem(b);
xlabel('Index number (n)');
ylabel('b[n]');
title('Impulse Response of the filter');
ax.XTick = 0:1:size(b);
grid on
grid minor

%zero-pole plot
figure
zplane(b,a);
xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Zero-pole plot');
grid on
grid minor


%save filter coefficients to 
fileID = fopen('fir_coef2.txt','w');
fprintf(fileID,'double b[] = {');
fprintf(fileID,'%.15e, ',b);
fprintf(fileID,'};');


