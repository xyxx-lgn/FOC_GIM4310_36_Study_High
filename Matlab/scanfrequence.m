clc;
clear;
close all;

Fs = 20000;   % 电流环执行频率 20kHz
file = 'scan_100hz.txt';

% 直接跳过第一行表头，按4列数值读
M = readmatrix(file, 'Delimiter', ',', 'NumHeaderLines', 1);

% 如果末尾有空行，这里顺手去掉
M = M(~any(isnan(M),2), :);

% 手动取列
sample_index = M(:,1);
frequence    = M(:,2);
iq_ref       = M(:,3);
iq_now       = M(:,4);

% 当前测试频率
f0 = frequence(1);

% 时间轴
t = sample_index / Fs;

% 去直流分量
u = iq_ref - mean(iq_ref);
y = iq_now - mean(iq_now);

% 去掉前30%过渡段
N = length(t);
idx0 = floor(N * 0.3) + 1;
t = t(idx0:end);
u = u(idx0:end);
y = y(idx0:end);

% 同步检波
w0 = 2*pi*f0;
c = cos(w0*t);
s = sin(w0*t);

Uc = 2/length(t) * sum(u .* c);
Us = 2/length(t) * sum(u .* s);
Yc = 2/length(t) * sum(y .* c);
Ys = 2/length(t) * sum(y .* s);

Ain  = sqrt(Uc^2 + Us^2);
Aout = sqrt(Yc^2 + Ys^2);

ph_in  = atan2(Us, Uc);
ph_out = atan2(Ys, Yc);

gain = Aout / Ain;
gain_dB = 20 * log10(gain);
phase_deg = (ph_out - ph_in) * 180/pi;
phase_deg = mod(phase_deg + 180, 360) - 180;

fprintf('测试频率: %.2f Hz\n', f0);
fprintf('输入幅值 Ain = %.6f\n', Ain);
fprintf('输出幅值 Aout = %.6f\n', Aout);
fprintf('增益 = %.6f\n', gain);
fprintf('增益(dB) = %.3f dB\n', gain_dB);
fprintf('相位差 = %.3f deg\n', phase_deg);

% 时域图
figure;
subplot(2,1,1);
plot(sample_index/Fs, iq_ref, 'b', 'LineWidth', 1.2); hold on;
plot(sample_index/Fs, iq_now, 'r', 'LineWidth', 1.2);
grid on;
legend('iq\_ref','iq\_now');
xlabel('Time (s)');
ylabel('Current (A)');
title(sprintf('Raw Time Domain Data at %.1f Hz', f0));

subplot(2,1,2);
plot(t, u, 'b', 'LineWidth', 1.2); hold on;
plot(t, y, 'r', 'LineWidth', 1.2);
grid on;
legend('iq\_ref AC','iq\_now AC');
xlabel('Time (s)');
ylabel('Current (A)');
title('AC Components After DC Removal');