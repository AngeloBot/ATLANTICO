fileID_x =-1;
errmsg_x = '';
fileID_t =-1;
errmsg_t = '';

multiple_plots =input('Multiple plots?(number) ','s');

multiple_plots = str2num(['uint8(',multiple_plots,')']);
for c =1:multiple_plots
    
    while fileID_x <0
        filename = input('Open x_ file: ', 's');
        fileID_x = fopen(filename);
    end
    while fileID_t <0
        filename = input('Open t_ file: ', 's');
        fileID_t = fopen(filename);
    end
    
    
    formatSpec='%f';
    t=fscanf(fileID_t,formatSpec);
    x=fscanf(fileID_x,formatSpec)
    
    t=t.';
    x=x.';
    
    length(x)
    length(t)
    
    figure(1);
    plot(t,x)
    hold on
    
    Fs=input('Frequency: ','s');
    Fs= str2num(['uint8(',Fs,')']);
    [pxx,f] = pwelch(x,[],[],[],Fs);   
    figure(2);
    plot(f,pow2db(pxx))
    hold on
    
    fclose(fileID_x);
    fclose(fileID_t);
end

figure(1)
hold off

figure(2)
hold off


