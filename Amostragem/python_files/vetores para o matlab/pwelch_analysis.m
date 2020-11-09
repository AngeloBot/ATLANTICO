fileID_x =-1;
errmsg_x = '';
fileID_t =-1;
errmsg_t = '';

mode =input('Select Mode: Window(w) or Multiple Plot(m) or analysis (A) or else(E)','s');

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
        ylabel('Mean (deg)')

        plot(t,x)
        hold on

        Fs=input('Frequency(Hz): ','s');
        Fs= str2num(['uint8(',Fs,')']);
        %[pxx,f] = pwelch(x,[],[],[],Fs);

        Fs=double(Fs);

        figure(2)
        
        
        pwelch(x,[],[],[],Fs)
        
        %plot(f,pow2db(pxx))
        %{
        title('Power Density Spectrum - Pwelch')
        xlabel('Frequency(Hz)')
        ylabel('Power Density (dB/Hz)')
        %}
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

elseif mode=='A'
    
    while fileID_x <0
            filename = input('Open x_ file(data in degrees): ', 's');
            fileID_x = fopen(filename);
    end
    
    formatSpec='%f';
    x=fscanf(fileID_x,formatSpec);
    Fs=input('Frequency(Hz): ','s');
    Fs= str2num(['uint8(',Fs,')']);
    Fs=double(Fs);
    whos x
    whos Fs

    figure(1)
    
    overlap_vect=[0.1 0.25 0.5 0.75];
    n_segment=8;
    size_segment=length(x);
    
    for c=1:length(overlap_vect)

        [pxx,f] = pwelch(x,hamming(size_segment),[],floor(overlap_vect(c)*size_segment),Fs);
        
        plot(f,pow2db(pxx).*(Fs/2))
        
        hold on
        
    end
    
    hold off
    
    title('Power Density Spectrum - Effect of overlap')
    xlabel('Frequency(Hz)')
    ylabel('Power Density (dB/Hz)')
    legend('10%','25%','50%','75%')
    
    grid on
    
    figure(2)
    
    size_segment=[];
    n_vect=[10 8 4 2];
    
    for c=1:length(n_vect)
        
        size_segment(c)=floor(length(x)/n_vect(c));
        [pxx,f] = pwelch(x,hamming(size_segment(c)),[],[],Fs);
        
        plot(f,pow2db(pxx).*(Fs/2))
        
        hold on
        
    end
    
    figure(2);
    
    hold off
    
    title('Power Density Spectrum - Effect of window size (samples)')
    xlabel('Frequency(Hz)')
    ylabel('Power Density (dB/Hz)')
    legend(num2str(size_segment.'))
    
    grid on

elseif mode=='E'
    
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
    
    len_sig=300;
    
    [t, index] = unique(t(1,len_sig:2*len_sig));
    
    vect=t(1):0.001:t(end);
    
    figure(1)
    
    interp_vect=interp1(t, x(index),vect);
    plot(vect,interp_vect)
    hold on
    plot(t,x(index),'o')
    plot(vect,interp_vect,'x')
    hold off
    
    figure(2)
    
    y = fft(interp_vect)                      % Compute DFT of x
    m = abs(y);                               % Magnitude
    y(m<1e-6) = 0;
    p = unwrap(angle(y));                     % Phase
    f = (0:length(y)-1)*100/length(y);        % Frequency vector

    subplot(2,1,1)
    plot(f,m)
    title('Magnitude')
    %ax = gca;
    %ax.XTick = [15 40 60 85];

    subplot(2,1,2)
    plot(f,p*180/pi)
    title('Phase')
    %ax = gca;
    %ax.XTick = [15 40 60 85];
    
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
        
        figure(3)
        
        pwelch(x,[],[],[],Fs)
end

