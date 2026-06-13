clc; clear; close all;

%% ====================== 用户配置区 ======================
% LUT点数
lut_size = 128;

% 你的工程里当前按 pole=14 使用
pole = 14;

% ================= 使用方式选择 =================
% mode_select = 1 -> 只使用正向数据
% mode_select = 2 -> 只使用反向数据
% mode_select = 3 -> 正反向都使用，并做平均
mode_select = 3;

% ================= 数据文件路径 =================
file_fwd = 'C:/Code/Jlink/编码器校准正向数据1.csv';
file_rev = 'C:/Code/Jlink/编码器校准反向数据1.csv';

% ================= 方向定义 =================
dir_fwd = +1;
dir_rev = -1;

% 是否打印 MCU 数组
print_c_array = true;

% 是否保存结果 mat 文件
save_mat_file = true;

%% ====================== 主流程 ======================
scan_fwd = [];
scan_rev = [];
lut_final = [];
lut_center_tick = [];

switch mode_select
    case 1
        % 只使用正向
        scan_fwd = parse_encoder_scan_file_simple(file_fwd, lut_size, dir_fwd, pole);
        lut_final = scan_fwd.lut_tick - mean(scan_fwd.lut_tick);
        lut_center_tick = scan_fwd.lut_center_tick;

    case 2
        % 只使用反向
        scan_rev = parse_encoder_scan_file_simple(file_rev, lut_size, dir_rev, pole);
        lut_final = scan_rev.lut_tick - mean(scan_rev.lut_tick);
        lut_center_tick = scan_rev.lut_center_tick;

    case 3
        % 正反向同时使用
        scan_fwd = parse_encoder_scan_file_simple(file_fwd, lut_size, dir_fwd, pole);
        scan_rev = parse_encoder_scan_file_simple(file_rev, lut_size, dir_rev, pole);

        lut_final = 0.5 * (scan_fwd.lut_tick + scan_rev.lut_tick);
        lut_final = lut_final - mean(lut_final);
        lut_center_tick = scan_fwd.lut_center_tick;

    otherwise
        error('mode_select 只能为 1 / 2 / 3');
end

%% ====================== 画图：正向 ======================
if ~isempty(scan_fwd)
    figure('Name', 'Forward Scan Analysis', 'Color', 'w');

    subplot(3,1,1);
    plot(scan_fwd.idx, scan_fwd.raw, 'b', 'LineWidth', 1.0); hold on;
    plot(scan_fwd.idx, mod(scan_fwd.raw_unwrap, 16384), 'r--', 'LineWidth', 1.0);
    grid on;
    xlabel('Sample Index');
    ylabel('Raw Tick');
    title('Forward: Raw Encoder Data');
    legend('raw', 'raw\_unwrap mod 16384', 'Location', 'best');

    subplot(3,1,2);
    plot(scan_fwd.ideal_tick_unwrap, scan_fwd.raw_unwrap, '.', 'Color', [0.7 0.7 0.7]); hold on;
    plot(scan_fwd.ideal_tick_unwrap, scan_fwd.fit_tick, 'r', 'LineWidth', 1.5);
    grid on;
    xlabel('Ideal Mechanical Tick (unwrap)');
    ylabel('Measured Raw Tick (unwrap)');
    title(sprintf('Forward: Linear Fit, slope=%.6f, offset=%.3f', scan_fwd.poly_p(1), scan_fwd.poly_p(2)));
    legend('raw\_unwrap', 'linear fit', 'Location', 'best');

    subplot(3,1,3);
    plot(scan_fwd.phase_tick, scan_fwd.err_tick, '.', 'Color', [0.75 0.75 0.75]); hold on;
    plot(scan_fwd.lut_center_tick, scan_fwd.lut_tick, 'r', 'LineWidth', 1.8);
    grid on;
    xlabel('Mechanical Angle Tick (0~16383)');
    ylabel('Error Tick');
    title('Forward: Nonlinearity Error and LUT');
    legend('error scatter', 'LUT', 'Location', 'best');
end

%% ====================== 画图：反向 ======================
if ~isempty(scan_rev)
    figure('Name', 'Reverse Scan Analysis', 'Color', 'w');

    subplot(3,1,1);
    plot(scan_rev.idx, scan_rev.raw, 'b', 'LineWidth', 1.0); hold on;
    plot(scan_rev.idx, mod(scan_rev.raw_unwrap, 16384), 'r--', 'LineWidth', 1.0);
    grid on;
    xlabel('Sample Index');
    ylabel('Raw Tick');
    title('Reverse: Raw Encoder Data');
    legend('raw', 'raw\_unwrap mod 16384', 'Location', 'best');

    subplot(3,1,2);
    plot(scan_rev.ideal_tick_unwrap, scan_rev.raw_unwrap, '.', 'Color', [0.7 0.7 0.7]); hold on;
    plot(scan_rev.ideal_tick_unwrap, scan_rev.fit_tick, 'r', 'LineWidth', 1.5);
    grid on;
    xlabel('Ideal Mechanical Tick (unwrap)');
    ylabel('Measured Raw Tick (unwrap)');
    title(sprintf('Reverse: Linear Fit, slope=%.6f, offset=%.3f', scan_rev.poly_p(1), scan_rev.poly_p(2)));
    legend('raw\_unwrap', 'linear fit', 'Location', 'best');

    subplot(3,1,3);
    plot(scan_rev.phase_tick, scan_rev.err_tick, '.', 'Color', [0.75 0.75 0.75]); hold on;
    plot(scan_rev.lut_center_tick, scan_rev.lut_tick, 'r', 'LineWidth', 1.8);
    grid on;
    xlabel('Mechanical Angle Tick (0~16383)');
    ylabel('Error Tick');
    title('Reverse: Nonlinearity Error and LUT');
    legend('error scatter', 'LUT', 'Location', 'best');
end

%% ====================== 最终LUT图 ======================
figure('Name', 'Final LUT', 'Color', 'w');

if ~isempty(scan_fwd) && ~isempty(scan_rev)
    plot(scan_fwd.lut_center_tick, scan_fwd.lut_tick, 'b', 'LineWidth', 1.2); hold on;
    plot(scan_rev.lut_center_tick, scan_rev.lut_tick, 'r', 'LineWidth', 1.2);
    plot(lut_center_tick, lut_final, 'k', 'LineWidth', 2.0);
    legend('Forward LUT', 'Reverse LUT', 'Final LUT', 'Location', 'best');
elseif ~isempty(scan_fwd)
    plot(scan_fwd.lut_center_tick, scan_fwd.lut_tick, 'b', 'LineWidth', 1.2); hold on;
    plot(lut_center_tick, lut_final, 'k', 'LineWidth', 2.0);
    legend('Forward LUT', 'Final LUT', 'Location', 'best');
elseif ~isempty(scan_rev)
    plot(scan_rev.lut_center_tick, scan_rev.lut_tick, 'r', 'LineWidth', 1.2); hold on;
    plot(lut_center_tick, lut_final, 'k', 'LineWidth', 2.0);
    legend('Reverse LUT', 'Final LUT', 'Location', 'best');
end

grid on;
xlabel('Mechanical Angle Tick (0~16383)');
ylabel('Error Tick');
title('Final Encoder Nonlinearity LUT');

%% ====================== 极坐标可视化 ======================
theta_plot = lut_center_tick / 16384 * 2*pi;

if ~isempty(scan_fwd)
    r_before = 1.0 + scan_fwd.lut_tick / 500.0;
else
    r_before = 1.0 + scan_rev.lut_tick / 500.0;
end

r_after = 1.0 + (r_before - 1.0 - lut_final / 500.0);

figure('Name', 'Polar View', 'Color', 'w');
polarplot(theta_plot, r_before, 'r', 'LineWidth', 1.5); hold on;
polarplot(theta_plot, r_after,  'b', 'LineWidth', 1.5);
title('Polar View of Nonlinearity Before/After LUT');
legend('Before compensation', 'Residual after compensation', 'Location', 'best');

%% ====================== 统计信息 ======================
fprintf('\n================ 校准结果统计 ================\n');

if ~isempty(scan_fwd)
    fprintf('Forward file   : %s\n', file_fwd);
    fprintf('Forward dir    : %d\n', scan_fwd.dir);
    fprintf('Forward pole   : %.3f\n', scan_fwd.pole);
    fprintf('Forward samples: %d\n', length(scan_fwd.idx));
    fprintf('Forward err pk-pk : %.3f tick\n', max(scan_fwd.err_tick) - min(scan_fwd.err_tick));
    fprintf('\n');
end

if ~isempty(scan_rev)
    fprintf('Reverse file   : %s\n', file_rev);
    fprintf('Reverse dir    : %d\n', scan_rev.dir);
    fprintf('Reverse pole   : %.3f\n', scan_rev.pole);
    fprintf('Reverse samples: %d\n', length(scan_rev.idx));
    fprintf('Reverse err pk-pk : %.3f tick\n', max(scan_rev.err_tick) - min(scan_rev.err_tick));
    fprintf('\n');
end

fprintf('Final LUT pk-pk : %.3f tick\n', max(lut_final) - min(lut_final));
fprintf('=============================================\n');

%% ====================== 打印 MCU LUT 数组 ======================
if print_c_array
    fprintf('\n// ===== Encoder Nonlinearity LUT =====\n');
    fprintf('#define ENC_NL_LUT_SIZE   %d\n', lut_size);
    fprintf('static const int16_t encoder_nl_lut[%d] = {\n', lut_size);

    for i = 1:lut_size
        if i < lut_size
            fprintf('    %d,\n', round(lut_final(i)));
        else
            fprintf('    %d\n', round(lut_final(i)));
        end
    end
    fprintf('};\n');
end

%% ====================== 保存结果 ======================
if save_mat_file
    save('encoder_nlcal_result.mat', ...
        'scan_fwd', 'scan_rev', 'lut_final', 'lut_center_tick', 'lut_size', 'pole');
end

disp('MATLAB处理完成。');

%% ====================== 局部函数 ======================
function scan = parse_encoder_scan_file_simple(filename, lut_size, dir_sign, pole)

    T = readtable(filename, 'VariableNamingRule', 'preserve');

    if width(T) < 3
        error('文件列数不足，至少需要3列: idx, cmd_mech_tick, encoder_raw');
    end

    % 兼容 I0,I1,I2 这种导出格式
    idx_arr = double(T{:,1});
    cmd_arr = double(T{:,2});
    raw_arr = double(T{:,3});

    % 理想机械角（直接使用打印出来的 cmd_mech_tick）
    ideal_tick_unwrap = cmd_arr;

    % 对 encoder_raw 做 unwrap
    raw_unwrap = raw_arr;
    for k = 2:length(raw_unwrap)
        d = raw_unwrap(k) - raw_unwrap(k-1);
        if d > 8192
            raw_unwrap(k:end) = raw_unwrap(k:end) - 16384;
        elseif d < -8192
            raw_unwrap(k:end) = raw_unwrap(k:end) + 16384;
        end
    end

    % 对理想机械角做 unwrap，避免跨圈跳变影响拟合
    ideal_tick_unwrap2 = ideal_tick_unwrap;
    for k = 2:length(ideal_tick_unwrap2)
        d = ideal_tick_unwrap2(k) - ideal_tick_unwrap2(k-1);
        if d > 8192
            ideal_tick_unwrap2(k:end) = ideal_tick_unwrap2(k:end) - 16384;
        elseif d < -8192
            ideal_tick_unwrap2(k:end) = ideal_tick_unwrap2(k:end) + 16384;
        end
    end

    % 最小二乘拟合，去掉整体比例和相位偏差
    p = polyfit(ideal_tick_unwrap2, raw_unwrap, 1);
    fit_tick = polyval(p, ideal_tick_unwrap2);

    % 周期误差
    err_tick = raw_unwrap - fit_tick;

    % 折叠回一圈
    phase_tick = mod(cmd_arr, 16384);

    % 按角度分bin生成 LUT
    edges = linspace(0, 16384, lut_size + 1);
    lut_tick = zeros(lut_size, 1);
    lut_center_tick = zeros(lut_size, 1);

    for i = 1:lut_size
        mask = phase_tick >= edges(i) & phase_tick < edges(i+1);
        lut_center_tick(i) = 0.5 * (edges(i) + edges(i+1));

        if any(mask)
            lut_tick(i) = mean(err_tick(mask));
        else
            lut_tick(i) = NaN;
        end
    end

    % 空bin用线性插值补齐
    valid = ~isnan(lut_tick);
    if any(~valid)
        lut_tick(~valid) = interp1(lut_center_tick(valid), lut_tick(valid), lut_center_tick(~valid), 'linear', 'extrap');
    end

    % 去均值，只保留周期误差
    lut_tick = lut_tick - mean(lut_tick);

    scan.filename = filename;
    scan.dir = dir_sign;
    scan.pole = pole;

    scan.idx = idx_arr;
    scan.cmd_tick = cmd_arr;
    scan.raw = raw_arr;
    scan.raw_unwrap = raw_unwrap;
    scan.ideal_tick_unwrap = ideal_tick_unwrap2;
    scan.poly_p = p;
    scan.fit_tick = fit_tick;
    scan.err_tick = err_tick;
    scan.phase_tick = phase_tick;
    scan.lut_center_tick = lut_center_tick;
    scan.lut_tick = lut_tick;
end