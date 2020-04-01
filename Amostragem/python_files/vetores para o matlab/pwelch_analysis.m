fileID_x =-1;
errmsg_x = '';
fileID_t =-1;
errmsg_t = '';

mode =input('Select Mode: Window(w) or Multiple Plot(m) ','s');

if mode=='m'

    multiple_plots =input('Multiple plots?(number) ','s');
    multiple_plots = str2num(['uint8(',multiple_plots,')']);
    for c =1:multiple_plots

        while fileID_x <0
            filename = input('Open x_ file(data in degrees): ', 's');
            fileID_x = fopen(filename);
        end
        while fileID_t <0
            filename = input('Open t_ file(data in degrees): ', 's');
            fileID_t = fopen(filename);
        end


        formatSpec='%f';
        t=fscanf(fileID_t,formatSpec);
        x=fscanf(fileID_x,formatSpec);


        t=t.';
        x=x.';

        length(x);
        length(t);

        figure(1);

        title('Signal')
        xlabel('Time(s)')
        ylabel('Desvio da média(degree)')

        plot(t,x)
        hold on

        Fs=input('Frequency(Hz): ','s');
        Fs= str2num(['uint8(',Fs,')']);
        %[pxx,f] = pwelch(x,[],[],[],Fs);

        Fs=double(Fs);

        figure(2);

        %{
        title('Power Density Spectrum - Pwelch')
        xlabel('Frequency(Hz)')
        ylabel('Power Density (dB/Hz)')
        %}
        pwelch(x,[],[],[],Fs)
        %plot(f,pow2db(pxx)*(Fs/2))
        hold on

        figure(3);

        title('FFT')
        xlabel('Frequency(Hz)')
        ylabel('Amplitude')

        vect_len=length(x);

        Y=fft(x,vect_len);

        Pyy=Y.*conj(Y)/vect_len;

        f=Fs*(0:(vect_len/2))/vect_len;

        plot(f,Pyy(1:(ceil(vect_len/2) +1)))
        
        hold on
        
        fclose(fileID_x);
        fclose(fileID_t);

        fileID_x=-1;
        fileID_t=-1;

        i=i+1;
    end

    figure(1)
    hold off

    figure(2)
    hold off
    
    figure(3)
    hold off

else
    
    while fileID_x <0
            filename = input('Open x_ file(data in degrees): ', 's');
            fileID_x = fopen(filename);
    end
        
        formatSpec='%f';
        x=fscanf(fileID_x,formatSpec);
        Fs=input('Frequency(Hz): ','s');
        Fs= str2num(['uint8(',Fs,')']);
        Fs=double(Fs);
        x=x.';
        whos x
        whos Fs
        whos pxx
        
        
        
        for len=100:50:650
            
            figure(1)
            
            [pxx,f] = pwelch(x,len,[],len,Fs);
            
            plot(f,pow2db(pxx).*(Fs/2))
            
            hold on
            
            figure(2)
            
            pwelch(x,len,[],len,Fs);
            
            hold on
            
        end
        
        figure(1)
        hold off
        
        figure(2)
        hold off
        
end

