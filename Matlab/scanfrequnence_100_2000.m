clc;
clear;
close all;

%% =========================
% 用户参数
% =========================
Fs = 20000;                           % 控制/采样频率 Hz
file = 'scan_100_4000hz_PI200_ori.csv';   % VOFA导出文件
discard_ratio = 0.30;                 % 每个频点丢弃前30%过渡段
min_points = 20;                      % 每个频点最少点数

%% =========================
% 读取数据
% =========================
M = readmatrix(file);

M = M(~any(isnan(M),2), :);           % 删除 NaN 行
M = M(~all(abs(M) < 1e-12, 2), :);    % 删除全0行

if size(M,2) < 4
    error('数据列数不足，至少需要4列：I0,I1,I2,I3');
end

sample_index = M(:,1);   % I0
freq_hz      = M(:,2);   % I1
iq_ref       = M(:,3);   % I2
iq_now       = M(:,4);   % I3

valid_row = freq_hz > 0;
sample_index = sample_index(valid_row);
freq_hz      = freq_hz(valid_row);
iq_ref       = iq_ref(valid_row);
iq_now       = iq_now(valid_row);

freq_list_all = unique(freq_hz);
freq_list_all = sort(freq_list_all);
n_freq_all = length(freq_list_all);

if isempty(freq_list_all)
    error('没有有效扫频数据，请检查CSV内容。');
end

%% =========================
% 中下图：逐频点分析频响
% 同时为上图提取"每个频点的1个周期"
% =========================
gain_arr        = nan(n_freq_all,1);
gain_dB_arr     = nan(n_freq_all,1);
phase_rad_arr   = nan(n_freq_all,1);
ain_arr         = nan(n_freq_all,1);
aout_arr        = nan(n_freq_all,1);
sample_num_arr  = nan(n_freq_all,1);

% 顶图拼接缓存
x_wave_all = [];
u_wave_all = [];
y_wave_all = [];
freq_wave_all = [];
seg_mid = zeros(n_freq_all,1);

cursor = 0;

for k = 1:n_freq_all
    f0 = freq_list_all(k);

    idx = abs(freq_hz - f0) < 1e-9;

    n = sample_index(idx);
    u = iq_ref(idx);
    y = iq_now(idx);

    sample_num_arr(k) = length(n);

    if length(n) < min_points
        continue;
    end

    t = n / Fs;

    [t, sort_idx] = sort(t);
    u = u(sort_idx);
    y = y(sort_idx);

    % 去掉前面过渡段
    N = length(t);
    idx0 = floor(N * discard_ratio) + 1;

    t = t(idx0:end);
    u = u(idx0:end);
    y = y(idx0:end);

    if length(t) < min_points
        continue;
    end

    % -------------------------
    % 为顶部图提取"1个周期"
    % -------------------------
    cycle_points = max(round(Fs / f0), 2);

    if length(t) >= cycle_points
        u1 = u(end-cycle_points+1:end);
        y1 = y(end-cycle_points+1:end);

        x1 = (cursor : cursor + cycle_points - 1).';
        x_wave_all = [x_wave_all; x1];
        u_wave_all = [u_wave_all; u1];
        y_wave_all = [y_wave_all; y1];
        freq_wave_all = [freq_wave_all; repmat(f0, cycle_points, 1)];

        seg_mid(k) = mean(x1);
        cursor = cursor + cycle_points;
    end

    % -------------------------
    % 频响分析
    % -------------------------
    u_ac = u - mean(u);
    y_ac = y - mean(y);

    w0 = 2*pi*f0;
    c = cos(w0 * t);
    s = sin(w0 * t);

    Uc = 2/length(t) * sum(u_ac .* c);   % A*sin(phi_in)
    Us = 2/length(t) * sum(u_ac .* s);   % A*cos(phi_in)
    Yc = 2/length(t) * sum(y_ac .* c);   % A*sin(phi_out)
    Ys = 2/length(t) * sum(y_ac .* s);   % A*cos(phi_out)

    Ain  = sqrt(Uc^2 + Us^2);
    Aout = sqrt(Yc^2 + Ys^2);

    if Ain < 1e-12
        continue;
    end

    ph_in  = atan2(Uc, Us);
    ph_out = atan2(Yc, Ys);

    gain = Aout / Ain;
    gain_dB = 20 * log10(gain);
    phase_rad = ph_out - ph_in;          % 滞后为负

    gain_arr(k)      = gain;
    gain_dB_arr(k)   = gain_dB;
    phase_rad_arr(k) = phase_rad;
    ain_arr(k)       = Ain;
    aout_arr(k)      = Aout;
end

%% =========================
% 清理无效频点
% =========================
valid = ~isnan(gain_dB_arr) & ~isnan(phase_rad_arr);

freq_list      = freq_list_all(valid);
gain_arr       = gain_arr(valid);
gain_dB_arr    = gain_dB_arr(valid);
phase_rad_arr  = phase_rad_arr(valid);
ain_arr        = ain_arr(valid);
aout_arr       = aout_arr(valid);
sample_num_arr = sample_num_arr(valid);
seg_mid_valid  = seg_mid(valid);

if isempty(freq_list)
    error('没有可用于Bode分析的有效频点。');
end

%% =========================
% 相位展开
% =========================
phase_rad_arr = unwrap(phase_rad_arr);
phase_deg_arr = phase_rad_arr * 180/pi;

%% =========================
% -3dB 带宽
% =========================
bw_hz = NaN;
for k = 2:length(freq_list)
    if gain_dB_arr(k-1) >= -3 && gain_dB_arr(k) <= -3
        f1 = freq_list(k-1);
        f2 = freq_list(k);
        g1 = gain_dB_arr(k-1);
        g2 = gain_dB_arr(k);

        bw_hz = f1 + (-3 - g1) * (f2 - f1) / (g2 - g1);
        break;
    end
end

%% =========================
% 打印结果
% =========================
fprintf('================ 扫频分析结果 ================\n');
fprintf('扫频范围: %.1f Hz ~ %.1f Hz\n', min(freq_list), max(freq_list));
fprintf('有效频点数: %d\n', length(freq_list));

if ~isnan(bw_hz)
    fprintf('估算闭环带宽: %.2f Hz (-3 dB)\n', bw_hz);
else
    fprintf('当前扫频范围内未找到 -3 dB 交点\n');
end

result_table = table(freq_list, sample_num_arr, gain_arr, gain_dB_arr, phase_deg_arr, ...
    'VariableNames', {'Freq_Hz','SampleNum','Gain','Gain_dB','Phase_deg'});
disp(result_table);

%% =========================
% 绘图
% =========================
fig = figure('Color','w','Position',[120 40 1250 920]);

% -------- 图1：每个频率取1个周期后拼接 --------
ax1 = subplot(3,1,1);
plot(x_wave_all, u_wave_all, 'b', 'LineWidth', 0.8); hold on;
plot(x_wave_all, y_wave_all, 'r', 'LineWidth', 0.8);
grid on;
xlabel('采样点');
ylabel('电流 (A)');
title(sprintf('实时波形: %.1f~%.1f Hz', min(freq_list), max(freq_list)));
legend('Iq指令', 'Iq实测', 'Location', 'best');

% -------- 图2：Bode增益 --------
subplot(3,1,2);
semilogx(freq_list, gain_dB_arr, 'o-b', 'LineWidth', 1.5, 'MarkerSize', 5); hold on;
grid on;
xlabel('频率 (Hz)');
ylabel('增益 (dB)');
title('Bode 图 - 增益');

h1 = yline(-3, '--', '-3dB', ...
    'Color', [0.90 0.35 0.35], ...
    'LineWidth', 1.2);
h1.Annotation.LegendInformation.IconDisplayStyle = 'off';

if ~isnan(bw_hz)
    h2 = xline(bw_hz, '--', sprintf('BW=%.1fHz', bw_hz), ...
        'Color', [0.1 0.8 0.1], ...
        'LineWidth', 1.2, ...
        'LabelOrientation', 'horizontal', ...
        'LabelVerticalAlignment', 'middle');
    h2.Annotation.LegendInformation.IconDisplayStyle = 'off';
end

xlim([min(freq_list), max(freq_list)]);

% -------- 图3：Bode相位 --------
subplot(3,1,3);
semilogx(freq_list, phase_deg_arr, 'o-r', 'LineWidth', 1.5, 'MarkerSize', 5); hold on;
grid on;
xlabel('频率 (Hz)');
ylabel('相位 (°)');
title('Bode 图 - 相位');

h3 = yline(-180, '--', '-180°', ...
    'Color', [0.5 0.5 0.5], ...
    'LineWidth', 1.0);
h3.Annotation.LegendInformation.IconDisplayStyle = 'off';

xlim([min(freq_list), max(freq_list)]);

%% =========================
% 自定义 Data Cursor
% =========================
dcm = datacursormode(fig);
set(dcm, 'Enable', 'on', 'UpdateFcn', @(~, event_obj) topWaveformDataTip(event_obj, x_wave_all, freq_wave_all, u_wave_all, y_wave_all));

%% =========================
% 本地函数
% =========================
function txt = topWaveformDataTip(event_obj, x_all, f_all, u_all, y_all)
    pos = event_obj.Position;
    xq = pos(1);

    [~, idx] = min(abs(x_all - xq));

    txt = {
        ['采样点: ', num2str(x_all(idx))], ...
        ['频率: ', num2str(f_all(idx), '%.3f'), ' Hz'], ...
        ['Iq指令: ', num2str(u_all(idx), '%.6f'), ' A'], ...
        ['Iq实测: ', num2str(y_all(idx), '%.6f'), ' A']
        };
end