k = exp(-64/8000/0.08);
b = 1-k;
a = [1,k];
zplane(b,a);