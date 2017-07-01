    clear;
    folder = '\\icnas2.cc.ic.ac.uk\jsl314\RTDSPlab\project_pt2\wav\car1.wav';
    filename = strcat(folder);
    
    actx = actxcontrol('WMPlayer.ocx.7');
    media = actx.newMedia(filename);
    actx.CurrentMedia = media;
    actx.Controls.play
    recObj = audiorecorder(8000,16,1);
    disp('start speaking you faggot')
    recordblocking(recObj, 38);
    disp('End of recording')
    y = getaudiodata(recObj);
    [s,w] = spectrogram(y,hann(256),0,256,8000,'yaxis');