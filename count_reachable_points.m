clc; clear; close all;

%% =========================================
% 功能：
% 1. 读取 COMSOL 导出的三个线圈在 1 A 下的磁场数据 map1/map2/map3
% 2. 在每个采样点组装驱动矩阵 A(:,:,k)
% 3. 统计在给定电流上限和目标磁场条件下，可达点有多少
% 4. 同时支持两种判据：
%    mode = 'fixed_vector'      指定目标磁场向量 B_target
%    mode = 'all_directions'    任意方向都要达到目标磁场模长 Breq
% 5. 导出结果表、MAT 文件与可视化图
%% =========================================

%% ---------- 文件路径 ----------
folder = 'C:\Users\Liyuan\Desktop\三线圈阵列仿真\驱动矩阵数据';
file1  = fullfile(folder, 'map1.txt');
file2  = fullfile(folder, 'map2.txt');
file3  = fullfile(folder, 'map3.txt');

%% ---------- 模式选择 ----------
% 'fixed_vector'   : 判断某个给定目标磁场向量是否可达
% 'all_directions' : 判断是否对任意方向都能达到给定磁场强度
mode = 'all_directions';

%% ---------- 目标磁场与电流上限 ----------
Imax = 10;              % 每个线圈电流上限，单位 A
Breq_mT = 5;            % 目标磁场强度，单位 mT
Breq_T  = Breq_mT*1e-3; % 转换为 T

% 当 mode = 'fixed_vector' 时使用
% 下面这个例子表示目标磁场为 z 方向 5 mT
B_target = [0; 0; Breq_T];

%% ---------- 任意方向判据的采样数 ----------
% 数值越大，"任意方向" 判据越严格，但计算更慢
numSphereSamples = 1000;

%% ---------- 读取三个文件 ----------
data1 = read_comsol_map(file1);
data2 = read_comsol_map(file2);
data3 = read_comsol_map(file3);

%% ---------- 基本检查 ----------
n1 = size(data1,1);
n2 = size(data2,1);
n3 = size(data3,1);

if ~(n1 == n2 && n2 == n3)
    error('三个文件的采样点数量不一致。');
end

N = n1;

coord_err_12 = max(abs(data1(:,1:3) - data2(:,1:3)), [], 'all');
coord_err_13 = max(abs(data1(:,1:3) - data3(:,1:3)), [], 'all');

if coord_err_12 > 1e-12 || coord_err_13 > 1e-12
    error('三个文件中的采样点坐标不一致，请检查 COMSOL 导出设置。');
end

coords = data1(:,1:3);   % x, y, z，单位 m

%% ---------- 预分配 ----------
A_all          = zeros(3,3,N);
cond_all       = zeros(N,1);
det_all        = zeros(N,1);
rank_all       = zeros(N,1);
reachable_all  = false(N,1);

% fixed_vector 模式下使用
I_all          = nan(N,3);
B_check_all    = nan(N,3);
Imax_req_all   = nan(N,1);
err_abs_all    = nan(N,1);
err_rel_all    = nan(N,1);

% all_directions 模式下使用
Bmin_all_T     = nan(N,1);
Bmin_all_mT    = nan(N,1);

% 方向采样向量
U = fibonacci_sphere(numSphereSamples);  % 3 x M

%% ---------- 逐点组装驱动矩阵并判断可达性 ----------
for k = 1:N
    % COMSOL 数据中的磁场单位为 mT，先转成 T
    b1 = data1(k,4:6).' * 1e-3;
    b2 = data2(k,4:6).' * 1e-3;
    b3 = data3(k,4:6).' * 1e-3;

    A = [b1, b2, b3];
    A_all(:,:,k) = A;

    cond_all(k) = cond(A);
    det_all(k)  = det(A);
    rank_all(k) = rank(A);

    switch mode
        case 'fixed_vector'
            I = pinv(A) * B_target;
            I_all(k,:) = I.';

            B_check = A * I;
            B_check_all(k,:) = B_check.';

            err_abs = norm(B_check - B_target);
            err_rel = err_abs / max(norm(B_target), eps);

            err_abs_all(k) = err_abs;
            err_rel_all(k) = err_rel;

            Imax_req = max(abs(I));
            Imax_req_all(k) = Imax_req;

            reachable_all(k) = (Imax_req <= Imax);

        case 'all_directions'
            % 对每个方向 u，最大可输出投影为 Imax * ||A' * u||_1
            % 取所有方向中的最小值，作为该点“任意方向可达”的保守能力
            projMax = Imax * sum(abs(A.' * U), 1); % 1 x M
            Bmin = min(projMax);

            Bmin_all_T(k)  = Bmin;
            Bmin_all_mT(k) = Bmin * 1e3;

            reachable_all(k) = (Bmin >= Breq_T);

        otherwise
            error('未知 mode：%s', mode);
    end
end

%% ---------- 结果表 ----------
switch mode
    case 'fixed_vector'
        ResultTable = table( ...
            coords(:,1), coords(:,2), coords(:,3), ...
            I_all(:,1), I_all(:,2), I_all(:,3), ...
            Imax_req_all, reachable_all, ...
            B_check_all(:,1), B_check_all(:,2), B_check_all(:,3), ...
            cond_all, det_all, rank_all, err_abs_all, err_rel_all, ...
            'VariableNames', { ...
            'x_m', 'y_m', 'z_m', ...
            'I1_A', 'I2_A', 'I3_A', ...
            'ImaxReq_A', 'Reachable', ...
            'Bx_check_T', 'By_check_T', 'Bz_check_T', ...
            'condA', 'detA', 'rankA', 'errAbs_T', 'errRel'});

        csvName = sprintf('reachable_fixed_%gA_%gmT.csv', Imax, Breq_mT);
        matName = sprintf('reachable_fixed_%gA_%gmT.mat', Imax, Breq_mT);

    case 'all_directions'
        ResultTable = table( ...
            coords(:,1), coords(:,2), coords(:,3), ...
            Bmin_all_mT, reachable_all, ...
            cond_all, det_all, rank_all, ...
            'VariableNames', { ...
            'x_m', 'y_m', 'z_m', ...
            'Bmin_mT', 'Reachable', ...
            'condA', 'detA', 'rankA'});

        csvName = sprintf('reachable_allDir_%gA_%gmT.csv', Imax, Breq_mT);
        matName = sprintf('reachable_allDir_%gA_%gmT.mat', Imax, Breq_mT);
end

%% ---------- 导出 ----------
writetable(ResultTable, fullfile(folder, csvName));

save(fullfile(folder, matName), ...
    'folder', 'mode', 'coords', 'A_all', 'reachable_all', ...
    'Imax', 'Breq_mT', 'Breq_T', 'B_target', ...
    'cond_all', 'det_all', 'rank_all', ...
    'I_all', 'B_check_all', 'Imax_req_all', ...
    'err_abs_all', 'err_rel_all', ...
    'Bmin_all_T', 'Bmin_all_mT', 'numSphereSamples');

%% ---------- 控制台统计 ----------
num_reachable = sum(reachable_all);
ratio_reachable = num_reachable / N * 100;

fprintf('============================================\n');
fprintf('模式 mode = %s\n', mode);
fprintf('采样点总数：%d\n', N);
fprintf('每个线圈电流上限 Imax = %.3f A\n', Imax);
fprintf('目标磁场强度 Breq = %.3f mT\n', Breq_mT);

if strcmp(mode, 'fixed_vector')
    fprintf('目标磁场向量 B_target = [%.4e, %.4e, %.4e] T\n', ...
        B_target(1), B_target(2), B_target(3));
end

fprintf('满足要求的点数：%d / %d (%.2f%%)\n', ...
    num_reachable, N, ratio_reachable);

if strcmp(mode, 'fixed_vector')
    [minI, idxMin] = min(Imax_req_all);
    [maxI, idxMax] = max(Imax_req_all);

    fprintf('最容易满足要求的点：\n');
    fprintf('  坐标 = (%.4f, %.4f, %.4f) m\n', ...
        coords(idxMin,1), coords(idxMin,2), coords(idxMin,3));
    fprintf('  所需最大电流 = %.4f A\n', minI);

    fprintf('最难满足要求的点：\n');
    fprintf('  坐标 = (%.4f, %.4f, %.4f) m\n', ...
        coords(idxMax,1), coords(idxMax,2), coords(idxMax,3));
    fprintf('  所需最大电流 = %.4f A\n', maxI);
else
    [maxBmin, idxBest] = max(Bmin_all_mT);
    [minBmin, idxWorst] = min(Bmin_all_mT);

    fprintf('任意方向能力最强的点：\n');
    fprintf('  坐标 = (%.4f, %.4f, %.4f) m\n', ...
        coords(idxBest,1), coords(idxBest,2), coords(idxBest,3));
    fprintf('  Bmin = %.4f mT\n', maxBmin);

    fprintf('任意方向能力最弱的点：\n');
    fprintf('  坐标 = (%.4f, %.4f, %.4f) m\n', ...
        coords(idxWorst,1), coords(idxWorst,2), coords(idxWorst,3));
    fprintf('  Bmin = %.4f mT\n', minBmin);
end
fprintf('============================================\n');

%% ---------- 可视化 ----------
figure('Color', 'w');
switch mode
    case 'fixed_vector'
        scatter3(coords(:,1), coords(:,2), coords(:,3), 80, Imax_req_all, 'filled');
        title(sprintf('达到目标磁场所需最大电流 | I_{max}=%.2f A, B=%.2f mT', Imax, Breq_mT));
        cb = colorbar;
        ylabel(cb, 'Required max current (A)');

    case 'all_directions'
        scatter3(coords(:,1), coords(:,2), coords(:,3), 80, Bmin_all_mT, 'filled');
        title(sprintf('各点任意方向保守磁场能力 B_{min} | I_{max}=%.2f A', Imax));
        cb = colorbar;
        ylabel(cb, 'B_{min} (mT)');
end
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
grid on;
axis equal;

figure('Color', 'w');
scatter3(coords(:,1), coords(:,2), coords(:,3), 80, double(reachable_all), 'filled');
title(sprintf('可达点分布 | mode=%s', mode));
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
cb = colorbar;
ylabel(cb, 'Reachable');
grid on;
axis equal;

figure('Color', 'w');
reachable_coords = coords(reachable_all, :);
unreachable_coords = coords(~reachable_all, :);

if isempty(reachable_coords) && isempty(unreachable_coords)
    axis off;
    text(0.5, 0.5, 'No reachable points under current constraints', ...
        'Units', 'normalized', 'HorizontalAlignment', 'center', ...
        'FontSize', 12);
    title(sprintf('只保留满足要求的点 | mode=%s', mode));
else
    hold on;

    if ~isempty(unreachable_coords)
        scatter3(unreachable_coords(:,1), unreachable_coords(:,2), unreachable_coords(:,3), ...
            45, [0.85 0.85 0.85], 'filled');
    end

    switch mode
        case 'fixed_vector'
            if ~isempty(reachable_coords)
                scatter3(reachable_coords(:,1), reachable_coords(:,2), reachable_coords(:,3), ...
                    80, Imax_req_all(reachable_all), 'filled');
            end
            cb = colorbar;
            ylabel(cb, 'Required max current (A)');
        case 'all_directions'
            if ~isempty(reachable_coords)
                scatter3(reachable_coords(:,1), reachable_coords(:,2), reachable_coords(:,3), ...
                    80, Bmin_all_mT(reachable_all), 'filled');
            end
            cb = colorbar;
            ylabel(cb, 'B_{min} (mT)');
    end

    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    grid on;
    axis equal;
    hold off;
    title(sprintf('只保留满足要求的点 | mode=%s', mode));
end

%% =========================================
% 本地函数：读取 COMSOL map 文件
% 输出格式：
% data = [x, y, z, Bx_mT, By_mT, Bz_mT]
%% =========================================
function data = read_comsol_map(filename)
fid = fopen(filename, 'r');
if fid == -1
    error('无法打开文件：%s', filename);
end

rows = [];

while ~feof(fid)
    line = strtrim(fgetl(fid));

    if isempty(line) || startsWith(line, '%')
        continue;
    end

    vals = sscanf(line, '%f,%f,%f,%f,%f,%f,%f,%f,%f');
    if numel(vals) ~= 9
        vals = sscanf(line, '%f');
    end

    if numel(vals) == 9
        row = [vals(1), vals(2), vals(3), vals(7), vals(8), vals(9)];
    elseif numel(vals) == 6
        row = vals(:).';
    else
        error('文件格式无法识别：%s\n出错行：%s', filename, line);
    end

    rows = [rows; row]; %#ok<AGROW>
end

fclose(fid);
data = rows;
end

%% =========================================
% 本地函数：Fibonacci sphere 方向采样
% 输出 U 为 3 x N，每一列是单位方向向量
%% =========================================
function U = fibonacci_sphere(N)
idx = 0:(N-1);
phi = pi * (3 - sqrt(5));
z = 1 - 2*(idx + 0.5)/N;
r = sqrt(max(0, 1 - z.^2));
theta = phi * idx;
x = r .* cos(theta);
y = r .* sin(theta);
U = [x; y; z];
end
