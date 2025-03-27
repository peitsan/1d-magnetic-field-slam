function groundtruthVisual(file_path)
    % 读取数据
    data = readtable(file_path, 'Delimiter', ' ');
    
    % 提取原始x和y（百分比坐标）
    x_p = data{:, 2};
    y_p = data{:, 3};
    
    % 缩放处理
    x_actual = x_p * 1.5; % x方向缩放因子3
    y_actual = y_p * 0.8 + 0.025; % y方向线性变换：y_actual = y_p * 3.225 + 0.025
    
    % 旋转180度（顺时针）
    x_rot = -x_actual;
    y_rot = -y_actual;
    
    % 更新数据表中的x和y列
    data{:, 2} = x_rot;
    data{:, 3} = y_rot;
    
    % 保存变换后的数据
    [file_dir, file_name, file_ext] = fileparts(file_path);
    new_file_name = file_name+'_transformed'+file_ext;
    new_file_path = fullfile(file_dir, new_file_name);
    writetable(data, new_file_path, 'Delimiter', ' ');
    
    

    % 绘制变换后的轨迹
    figure;
    plot(x_rot, y_rot, 'b-', 'LineWidth', 1.5);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Transformed XY Trajectory');
    grid on;
    
  
    
    % 将坐标原点移动到出发点
    start_x = x_rot(1);
    start_y = y_rot(1);
    x_shifted = x_rot - start_x;
    y_shifted = y_rot - start_y;
    
    % 清除之前的绘图
    clf;
    
    % 绘制相对于出发点的轨迹
    plot(x_shifted, y_shifted, 'b-', 'LineWidth', 1.5);
    xlabel('X Position Relative to Start (m)');
    ylabel('Y Position Relative to Start (m)');
    title('Transformed XY Trajectory Relative to Start');
    grid on;
    % 固定画幅范围 单位m
    axis([-3, 0.1, -0.6, 3]);
    
    % 添加起点和终点标记
    hold on;
    start_idx = 1;
    end_idx = length(x_shifted);
    plot(x_shifted(start_idx), y_shifted(start_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(x_shifted(end_idx), y_shifted(end_idx), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    legend('Trajectory', 'Start', 'End');
    hold off;
end