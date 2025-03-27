function [odom_time, dPos, dPosSmooth, initPos, quat, t, tm, m, LL] = alignROSTime(imu, mag, odom, quatn)
    
    % 提取并排序里程计数据（确保时间单调递增）提取dPos
    odom_time = [odom.Time]';  % 关键修改点：结构体数组字段提取
    % 排序时间戳并获取索引
    [odom_time_sorted, sort_idx] = sort(odom_time);
    odom_pos_sorted = vertcat(odom(sort_idx).Position);

    odom_time = odom_time_sorted;
    % 计算里程计位置数据的变化量
    dPos = [odom_pos_sorted.X; odom_pos_sorted.Y; odom_pos_sorted.Z]';
%     dPos = [0,0,0]+diff(dPos);
    dPos = diff(dPos);

    % 计算相对时间t（起点为0.0002秒）
    t_bias = odom_time_sorted(1) - 0.0002;  % 计算初始偏移量
    t = odom_time_sorted - t_bias;          % 确保第一个时间为0.0002秒
    tm = [0; diff(t)];                       % 一阶差分（时间增量）

    % 处理mag数据
    mag_time = [mag.Time]';
    [mag_time_sorted, mag_sort_idx] = sort(mag_time);
    mag_sorted = mag(mag_sort_idx);
    % 截取在odom时间范围内的mag数据
    valid_mag = (mag_time_sorted >= odom_time_sorted(1)) & (mag_time_sorted <= odom_time_sorted(end));
    mag_valid = mag_sorted(valid_mag);
    mag_time_valid = mag_time_sorted(valid_mag);
    % 计算odom的平均时间间隔
    num_odom = length(odom_time_sorted);
    avg_interval = (odom_time_sorted(end) - odom_time_sorted(1)) / (num_odom - 1);
    % 生成理论时间轴
    theoretical_times = linspace(odom_time_sorted(1), odom_time_sorted(end), num_odom);
    % 找到最近邻索引
    idx_mag = dsearchn(mag_time_valid, theoretical_times');
    mags = vertcat(mag_valid(idx_mag).Mag);
    m = [mags.X; mags.Y; mags.Z]';  
    m = 100 * m; %1 Gaussian = 100μT

    % 处理quat数据（与mag逻辑相同）
    quat_time = [quatn.Time]';
    [quat_time_sorted, quat_sort_idx] = sort(quat_time);
    quat_sorted = quatn(quat_sort_idx);
    valid_quat = (quat_time_sorted >= odom_time_sorted(1)) & (quat_time_sorted <= odom_time_sorted(end));
    quat_valid = quat_sorted(valid_quat);
    quat_time_valid = quat_time_sorted(valid_quat);
    idx_quat = dsearchn(quat_time_valid, theoretical_times');
    % 合并四元数数据为Nx4矩阵
    x = [quat_valid(idx_quat).X]';
    y = [quat_valid(idx_quat).Y]';
    z = [quat_valid(idx_quat).Z]';
    w = [quat_valid(idx_quat).W]';
    quat = [x, y, z, w];
    initPos = [odom_pos_sorted(1).X,odom_pos_sorted(1).Y,odom_pos_sorted(1).Z];
    
    % Domain boundaries
    x = [odom_pos_sorted.X];
    y = [odom_pos_sorted.Y];
    z = [odom_pos_sorted.Z];
    
    % 计算每个方向的最小值和最大值
    min_x = min(x);
    max_x = max(x);
    min_y = min(y);
    max_y = max(y);
    min_z = min(z);
    max_z = max(z);
    % 计算各方向原始范围（max - min）
    range_x = max_x - min_x;
    range_y = max_y - min_y;
    range_z = max_z - min_z;
    % 找到所有方向的最小正值范围（避免零或负值）
    positive_ranges = [range_x, range_y, range_z];
    positive_ranges(positive_ranges <= 0) = Inf;  % 忽略非正值
    pm = 0.2 * min(positive_ranges);
    % 处理特殊情况：如果所有范围为零，pm设为0
    if isinf(pm)
        pm = 0;
    end
    % 计算扩展后的范围长度（原始范围 + 2*pm）
    LL = [range_x +pm, range_y + pm, range_z - pm];
    % 如果z方向范围为零，强制设为0.20（根据需求调整）
    if range_z == 0
        LL(3) = 0.20;
    end
%   filtered_data.LL = LL;
    dPosSmooth=gotPosSmooth(dPos, 0.15)

    function dPosSmooth = gotPosSmooth(dPos, span)
        % dPos: 输入的包含X, Y, Z三列数据的矩阵
        % span: 局部回归平滑的窗口大小（span参数，越大越平滑）
        % smoothed_dPos: 输出平滑后的dPos矩阵
    
        % 检查输入数据的有效性
        if ndims(dPos) ~= 2 || size(dPos, 2) ~= 3
            error('输入数据dPos必须是一个包含X, Y, Z三列的二维矩阵');
        end
    
        % 初始化平滑后的数据矩阵
        dPosSmooth = zeros(size(dPos));
    
        % 对每一列进行平滑处理
        for i = 1:3
            dPosSmooth(:, i) = smooth(dPos(:, i), span,'rlowess');
        end

    end
end