% get a section of vowel
[x,fs]=audioread('test.wav');
ms20=fs/50;                 % minimum speech Fx at 50Hz
%
% plot waveform
t=(0:length(x)-1)/fs;        % times of sampling instants
sgtitle('Input Recording of Group Member and Corresponding Auto-correlation');
subplot(2,1,1);
plot(t,x);
grid on
legend('Input Signal');
xlabel('t (s)');
ylabel('Amplitude (m)');
%
% calculate autocorrelation
r=xcorr(x,ms20,'coeff');   
%
% plot autocorrelation
d=(-ms20:ms20)/fs;          % times of delays
subplot(2,1,2);
plot(d,r);
grid on
legend(['Auto-' char(10) 'correlation']);
xlabel('\tau (s)');
ylabel('C(\tau) - Correlation');

ms2=fs/500                 % maximum speech Fx at 500Hz
ms20=fs/50                 % minimum speech Fx at 50Hz
% just look at region corresponding to positive delays
r=r(ms20+1:2*ms20+1)
[rmax,tx]=max(r(ms2:ms20))
fprintf('rmax=%g Fx=%gHz\n',rmax,fs/(ms2+tx-1));

% figureHandle = gcf;
% 
% set(figureHandle,'PaperPosition',3*[0 0 6 4]);
% 
% set(figureHandle,'PaperSize',3*[6 4]);
% 
% set(figureHandle,'PaperUnits','centimeters');
% 
% print -dpdf 'voice2';
