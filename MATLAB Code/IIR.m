%calculate filter coefficients
n = 6;
rp = 0.3;
rs = 20;
pb = [200 450]/4000;
[b,a] = ellip(n,rp,rs,pb,'bandpass');

%calculate frequency and phase responses
[h,w] = freqz(b,a);
[phase,w] = phasez(b,a);

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
plot(w*4000/pi,phase);
xlabel('Frequency (Hz)');
ylabel('Phase (rad)');
title('Phase Response of the filter');
ax.XTick = 0:.5:2;
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
fileID = fopen('iir_coef7.txt','w');
fprintf(fileID,'double a[] = {');
fprintf(fileID,'%.15e, ',a);
fprintf(fileID,'};');
fprintf(fileID,'\n double b[] = {');
fprintf(fileID,'%.15e, ',b);
fprintf(fileID,'};');
