clear;
afr = dsp.AudioFileReader('clean.wav');
adw = audioDeviceWriter('SampleRate', afr.SampleRate);

while ~isDone(afr)
    audio = step(afr);
    step(adw,audio);
end
release(afr);
release(adw);

recObj = audiorecorder;
disp('start speaking you faggot')
recordblocking(recObj, 38);
disp('End of recording')

play(recObj);
y = getaudiodata(recObj);
figure;
spectrogram(y,hann(8192),0,8192,8000,'yaxis')