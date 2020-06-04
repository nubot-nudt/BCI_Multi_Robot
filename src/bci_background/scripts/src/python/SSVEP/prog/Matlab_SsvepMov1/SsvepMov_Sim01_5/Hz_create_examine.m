%% FFT���Ƶ�ʷ���׼ȷ��
clc
warning('off')
[slice,fs,bits]=wavread('0616��D');

% temp2 = load('BP_4_35_n96.mat');
% BPfilterssvep = temp2.Num;
% slice=filter(BPfilterssvep, 1, slice);

N=length(slice);

% n = 0:N-1;
% y=fft(slice(:,:) ,N);    %���źŽ��п���Fourier�任
% mag=abs(y);     %���Fourier�任������
% f=n*fs/N;    %Ƶ������
% plot(f,mag);   %�����Ƶ�ʱ仯�����
% plot(f(1:N/2),mag(1:N/2))

[Point_Hz,Amp] = FFT(slice,N,fs);
[Point_Hz1,Amp1] = FFT1(slice,N,fs);
p=find(Amp(:,1)==max(Amp(:,1)));
Amp1(p,1)
(p-1)/length(Point_Hz)*fs/2.; %����ֵʱ֮Ƶ��
% Amp(p,1)=0;
p=100*N/fs;


subplot(121);
plot(Point_Hz(1:p),Amp(1:p,1));

xlabel('Frequency/Hz');
ylabel('amplitude');
title('6Hz FFT')
grid on;

px=find(Amp(1:p,1)==max(Amp(1:p,1)));
zhi=(px-1)/length(Point_Hz)*fs/2.; %����ֵʱ֮Ƶ��
z=num2str(zhi);
text(zhi,max(Amp(1:p,1)),z);


subplot(122);
plot(Point_Hz1(1:p),Amp1(1:p,1));
grid on;
px=find(Amp1(1:p,1)==max(Amp1(1:p,1)));
zhi=(px-1)/length(Point_Hz1)*fs/2.; %����ֵʱ֮Ƶ��
z=num2str(zhi);
text(zhi,max(Amp1(1:p,1)),z);

% plot(Point_Hz(:),Amp(:,1))
% plot(Point_Hz,Amp);
% set(gca,'XTick',1:2:3*p)
% set(gca,'XTickLabel',{'-pi','-pi/2','0','pi/2','pi'})


