load('\\icnas2.cc.ic.ac.uk/jsl314/MATLAB/RTDSP/s6.mat')
load('\\icnas2.cc.ic.ac.uk/jsl314/MATLAB/RTDSP/w6.mat')

s6=s;
w6=w;

load('\\icnas2.cc.ic.ac.uk/jsl314/MATLAB/RTDSP/s8.mat')
load('\\icnas2.cc.ic.ac.uk/jsl314/MATLAB/RTDSP/w8.mat')

s8=s;
w8=w;

load('\\icnas2.cc.ic.ac.uk/jsl314/MATLAB/RTDSP/s5.mat')
load('\\icnas2.cc.ic.ac.uk/jsl314/MATLAB/RTDSP/w5.mat')

s5=s;
w5=w;

figure
hold on
plot(w2,abs(s2(:,1)))
plot(w2,abs(s2(:,2)))
plot(w2,abs(s2(:,3)))
xlabel('Frequency (Hz)')
ylabel('Magnitude')
legend('1','2','3');
title('Effect of varying tau')