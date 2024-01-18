function [lagAtMaxVal, SampleDiff] = detect_delays(x, y, end_time_x, end_time_y, plot_flag)

if nargin < 5
    plot_results = 0;
else
    plot_results = plot_flag;
end

len_x = length(x);
len_y = length(y);

start_time = 0.0;
Fs_x = len_x/(end_time_x - start_time);
Fs_y = len_y/(end_time_y - start_time);

if Fs_x > Fs_y
    Fs = Fs_x;
else
    Fs = Fs_y;
end

% resample data at the rate of signal with higher frequency
[P_x,Q_x] = rat(Fs/Fs_x, 0.01);  % Rational fraction approximation
[P_y,Q_y] = rat(Fs/Fs_y, 0.01);  % Rational fraction approximation
T_x = resample(x,P_x,Q_x); % Change sampling rate by rational factor
T_y = resample(y,P_y,Q_y); % Change sampling rate by rational factor

% find cross correlation between resampled signals
[c, lag] = xcorr(T_x,T_y);

% find the sample difference and time delay
[~,I] = max(abs(c));
SampleDiff = lag(I);
% Tfinal = SampleDiff*timeDiff;
% compute maximum lag for the resampled data
[~,maxCorrIdx]=max(c);
% Use lagAtMaxVal ( = timeDiff) = SampleDiff/Fs; %  or
lagAtMaxVal=lag(maxCorrIdx)/Fs;

% create time vectors for each x and y
time_x = linspace(start_time, end_time_x, len_x);
time_y = linspace(start_time, end_time_y, len_y);

if plot_results == 1
    figure;
    subplot(211);
    plot(time_x, x, 'LineWidth', 2);hold on;plot(time_y, y, 'LineWidth', 2);hold off;
    grid on; axis tight; title('Actual');legend("X","Y", 'Location','best');
    subplot(212);
    plot(time_x,x,'LineWidth', 2); hold on; plot(time_y+lagAtMaxVal,y,'LineWidth', 2);
    grid on; axis tight;xlabel('Time -- (sec)');legend("X","Y", 'Location','best');
    title('Aligned')
end
