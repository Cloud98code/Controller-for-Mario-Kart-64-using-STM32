clear all
close all
clc

% Acquisition of 5 seconds of the raw signal (acceleration on the z axis)
% through serial port which was then stored in the file 'z_data.mat'

% fsampling=10e-3; % one data every 10 ms was sent through the serial port
% count=5/fsampling;
% device = serialport("COM3",115200);
% y_data=read(device,count,"int32");

% definition of the time axis
time=0:10e-3:5-10e-3;

% loading of the stored signal
load('z_data.mat');
data=z_data;

%% Moving Average

n=5; % number of samples of the filter

% numerator of the filter transfer function
b=zeros(1,n);
for i=1:n
    b(i)=1/n;
end

% denumerator of the filter transfer function
a=1;

% plot of the filter's frequency response
figure
freqz(b,a,512);
title('Frequency response')

% definition of the kernel
kernel=[zeros(1,9) b zeros(1,10)];

% plot of the kernel
figure
plot(kernel,'-o')
grid on
xlim([1 24])
title('Filter Kernel')
xlabel('samples')
ylabel('amplitude')

% filtering with recursive moving average 

filtered=zeros(1,length(data));
z=0;
for i=1:length(data)
    if i<(n-1)/2+2 || i>length(data)- (n-1)/2
        filtered(i)=0;
    else
       filtered(i)=[filtered(i-1) + data(i+(n-1)/2)/n - data(i-((n-1)/2+1))/n];
    end
end

% plot of the raw and filtered signals
figure
plot(time,data)
hold on
plot(time,filtered+mean(data))
grid on
legend('raw signal','filtered signal')
title('comparison between raw signal and filtered signal')
xlabel('time (s)')
ylabel('amplitude (mg)')
