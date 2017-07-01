%calculate filter coefficients
n = 7;
rp = 0.3;
rs = 20;
pb = [200 450]/4000;
[b,a] = ellip(n,rp,rs,pb,'bandpass');