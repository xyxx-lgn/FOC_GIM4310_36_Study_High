clear; clc; close all;

%% 1) 读数据
file = 'c:\Code\Jlink\kw测试数据.csv';

opts = detectImportOptions(file, ...
    'Delimiter', ';', ...
    'VariableNamingRule', 'preserve');

T = readtable(file, opts);

t_us   = T.("Timestamp");
iq_ref = T.("pid_param.Iq_aim");
iq_now = T.("pid_param.Iq_now");
wm_now = T.("pid_param.Motor_Speed_now");
wm_flt = T.("pid_param.Motor_Speed_filt_now");

t = t_us * 1e-6;   % us -> s

%% 2) 找阶跃点
step_idx = [1; find(diff(iq_ref) ~= 0) + 1];
step_t   = t(step_idx);
step_val = iq_ref(step_idx);

%% 3) 参数设置
amp_target = 0.2;          % 本次测试阶跃幅值
fit_t1 = 0.020;            % 阶跃后20ms
fit_t2 = 0.120;            % 阶跃后120ms

kw_list = [];
kw_pos  = [];
kw_neg  = [];

fit_info = struct([]);

%% 4) 遍历每个 ±0.2A 阶跃
for k = 1:length(step_idx)
    if abs(step_val(k)) ~= amp_target
        continue;
    end

    t0 = step_t(k);

    idx = (t > (t0 + fit_t1)) & (t < (t0 + fit_t2));
    if nnz(idx) < 10
        continue;
    end

    % 拟合滤波速度
    p_flt = polyfit(t(idx), wm_flt(idx), 1);
    slope_flt = p_flt(1);   % rad/s^2

    % 拟合原始速度（对照）
    p_now = polyfit(t(idx), wm_now(idx), 1);
    slope_now = p_now(1);

    iq_avg = mean(iq_now(idx));

    kw_flt = slope_flt / iq_avg;
    kw_now = slope_now / iq_avg;

    kw_abs = abs(kw_flt);
    kw_list(end+1,1) = kw_abs; %#ok<SAGROW>

    if step_val(k) > 0
        kw_pos(end+1,1) = kw_abs; %#ok<SAGROW>
    else
        kw_neg(end+1,1) = kw_abs; %#ok<SAGROW>
    end

    fit_info(end+1).t0 = t0; %#ok<SAGROW>
    fit_info(end).ref = step_val(k);
    fit_info(end).iq_avg = iq_avg;
    fit_info(end).slope_flt = slope_flt;
    fit_info(end).slope_now = slope_now;
    fit_info(end).kw_flt = kw_flt;
    fit_info(end).kw_now = kw_now;
end

%% 5) 输出结果
fprintf('总段数 = %d\n', length(kw_list));
fprintf('Kw 总平均 = %.3f rad/s^2/A\n', mean(kw_list));
fprintf('Kw 正向平均 = %.3f rad/s^2/A\n', mean(kw_pos));
fprintf('Kw 反向平均 = %.3f rad/s^2/A\n', mean(kw_neg));
fprintf('Kw 最小 = %.3f, 最大 = %.3f\n', min(kw_list), max(kw_list));
fprintf('Kw 标准差 = %.3f\n', std(kw_list));

%% 6) 画图：所有段的Kw散点
figure('Name','Kw Identification');
subplot(2,1,1);
plot(kw_list, 'bo-', 'LineWidth', 1.2); grid on;
xlabel('Step index');
ylabel('Kw (rad/s^2/A)');
title(sprintf('Kw per step, mean = %.2f', mean(kw_list)));

subplot(2,1,2);
hold on; grid on;
yline(mean(kw_list), 'r--', 'Mean');
plot(kw_pos, 'g.-', 'DisplayName', 'Positive steps');
plot(kw_neg, 'm.-', 'DisplayName', 'Negative steps');
xlabel('Step index');
ylabel('Kw (rad/s^2/A)');
legend('Location','best');
title('Positive / Negative comparison');

%% 7) 画第一段拟合效果
% 找第一段 +0.2A
first_idx = find(step_val == amp_target, 1, 'first');
if ~isempty(first_idx)
    t0 = step_t(first_idx);
    idx = (t > (t0 + fit_t1)) & (t < (t0 + fit_t2));

    p = polyfit(t(idx), wm_flt(idx), 1);
    yfit = polyval(p, t(idx));

    figure('Name','First Step Fit');
    plot(t, wm_flt, 'b', 'LineWidth', 1.0); hold on; grid on;
    plot(t(idx), yfit, 'r--', 'LineWidth', 1.5);
    xline(t0, 'k--', 'Step start');
    xline(t0 + fit_t1, 'g--', 'Fit start');
    xline(t0 + fit_t2, 'm--', 'Fit end');
    xlabel('Time (s)');
    ylabel('\omega_m (rad/s)');
    title('First +0.2A step fit on Motor\_Speed\_filt\_now');
    legend('Measured speed', 'Linear fit', 'Location', 'best');
end