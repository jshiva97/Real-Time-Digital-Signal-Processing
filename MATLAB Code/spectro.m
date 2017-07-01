clear;
folder = '\\icnas2.cc.ic.ac.uk\jsl314\RTDSPlab\project_pt2\wav\';
savefolder = '\\icnas2.cc.ic.ac.uk\jsl314\pictures\RTDSP\enhancement2\';
files = {'clean' 'car1' 'factory1' 'factory2' 'lynx1' 'lynx2' 'phantom1' 'phantom2' 'phantom4'};
for i =1:size(files,2)
    filename = strcat(folder,files{1,i},'.wav')
    savefilename = strcat(savefolder,files{1,i})
    fig_name = sprintf('Spectrogram of the %s Audio', files{1,i});
    
    actx = actxcontrol('WMPlayer.ocx.7');
    media = actx.newMedia(filename);
    actx.CurrentMedia = media;
    actx.Controls.play
    recObj = audiorecorder(8000,16,1);
    disp('start speaking you faggot')
    recordblocking(recObj, 38);
    disp('End of recording')
    y = getaudiodata(recObj);
    figure;
    spectrogram(y,hann(256),0,256,8000,'yaxis');
    title(fig_name,'fontsize',17);
    print(savefilename,'-dpng');
end