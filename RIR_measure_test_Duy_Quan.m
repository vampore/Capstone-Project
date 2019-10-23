[data, fs] = audioread('speaker.wav');
speaker=data;

load simulation_real_sensor_data
%% noiseless simulation
y=sensor_noiseless_data(:,1);
x=speaker;
Y=fft(y);
X=fft(x,size(Y,1));
H=Y./X;
h_est=real(ifft(H));
sig_len=size(y,1)-size(x,1);
h_new=h_est(1:sig_len); %impulse response result
figure;plot(1:sig_len,h_new) %impulse response result
figure;plot(1:84096,sensor_noiseless_data(:,1))
%play sound
sound(speaker)
soundsc(sensor_noiseless_data(:,1),16000)

%% noisy simulation
y=sensor_noisy_data(:,1);
x=speaker;
Y=fft(y);
X=fft(x,size(Y,1));
H=Y./X;
h_est=real(ifft(H));
sig_len=size(y,1)-size(x,1);
h_new=h_est(1:sig_len); %impulse response result
figure;plot(1:sig_len,h_new) %impulse response result
figure;plot(1:84096,sensor_noisy_data(:,1))
%play sound
sound(speaker)
soundsc(sensor_noisy_data(:,1),16000)

%% from real sensors
y=data(:,1);
x=speaker;
Y=fft(y);
X=fft(x,size(Y,1));
H=Y./X;
h_est=real(ifft(H));
sig_len=size(y,1)-size(x,1);
h_new=h_est(1:sig_len);
figure;plot(1:sig_len,h_new) %impulse response result
figure; plot(1:128000,data(:,1)) %impulse response result
%play sound
sound(speaker)
soundsc(data(:,1),25600)









