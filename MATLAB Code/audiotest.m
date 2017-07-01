clear;
recObj = audiorecorder
disp('start speaking you faggot')
recordblocking(recObj, 38);
disp('End of recording')

y = getaudiodata(recObj);
figure;
spectrogram(y,hann(8192),0,8192,8000,'yaxis')