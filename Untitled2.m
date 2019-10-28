clc
clear

s = daq.createSession('ni');
addAnalogInputChannel(s,'cDAQ1Mod1', 0:3, 'Microphone');
addAnalogInputChannel(s,'cDAQ2Mod1', 0, 'Microphone');
s.DurationInSeconds = 6;
s.Channels(1).Sensitivity = 0.037;
s.Channels(2).Sensitivity = 0.037;
s.Channels(3).Sensitivity = 0.037;
s.Channels(4).Sensitivity = 0.037;
s.Channels(5).Sensitivity = 0.037;
s.Rate = 16000;

disp ('start recoding')


[data,time] = startForeground(s);
disp ('DONE recoding')

plot(data);
% [signalData,fs] = audioread('speaker.wav');

% cutPoint = 5333;

% IR = ifft(fft(data(cutPoint:end,:))./fft(signalData));
% figure(1);
% subplot(3,2,1);
% plot(time(cutPoint:end), IR(:,1));
% subplot(3,2,2);
% plot(time(cutPoint:end), IR(:,2));
% subplot(3,2,3);
% plot(time(cutPoint:end), IR(:,3));
% subplot(3,2,4);
% plot(time(cutPoint:end), IR(:,4));
% subplot(3,2,5);
% plot(time(cutPoint:end), IR(:,5));

% figure(2);
% subplot(3,2,1);
% plot(time, data(:,1));
% subplot(3,2,2);
% plot(time, data(:,2));
% subplot(3,2,3);
% plot(time, data(:,3));
% subplot(3,2,4);
% plot(time, data(:,4));
% subplot(3,2,5);
% plot(time, data(:,5));

% soundsc(data(:,1:2),51200);
