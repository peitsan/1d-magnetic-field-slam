function [filename]=collectRosData
    % 初始化存储变量
    clear, close all, clc
    rosshutdown

    % 连接到ROS主节点
    setenv('ROS_MASTER_URI','http://192.168.1.107:11311/');
    setenv('ROS_IP','192.168.1.107')  
    rosinit;   
    bag = rosbag()


    persistent imu mag odom tr quatn 

    % 创建订阅者
    imuSub = rossubscriber('/imu/data', @processImuData);
    magSub = rossubscriber('/imu/mag', @processMag);
    timeRefSub = rossubscriber('/imu/time_ref', @processTimeRef);
    odomSub = rossubscriber('/imu2odom_node/imu_odometry', @processOdom);
    quatSub = rossubscriber('/filter/quaternion', @processQuat);
    
    % 等待关闭ROS连接中断
    % 用于退出循环的标志
    exit_flag = false;
    % 无限循环监听键盘输入
    while ~exit_flag
        % 提示用户输入指令
        disp('数据采集中...。请输入指令中断：');
        command = input(' (exit: 中止不保存, save [file]: 录制完成并保存): \n', 's');
        
        % 检查用户输入的指令
        if strcmpi(command, 'exit')
            % 中止程序
            disp('接受中止指令，不保存数据。');
            exit_flag = true; % 设置退出标志
            
        elseif strcmpi(command, 'save')
            % 录制完成并保存
            disp('接受保存指令，准备保存数据。');
            
            % 保存一份原始数据
            filename=sprintf('ros-data-%s.mat',datestr(now,'mm-dd-HH-MM'))
            save(filename, 'imu', 'mag', 'tr', 'odom', 'quatn')
            fprintf('原始数据已保存为%s。\n',filename);

            % 对原始数据进行抽样对齐
            [odom_time, dPos, dPosSmooth, initPos, quat, t, tm, m, LL] = alignROStime(imu, mag, odom, quatn);
            filename= sprintf('square-mag-%s.mat',datestr(now,'mm-dd-HH-MM'))
            save(filename, 'odom_time', 'dPos', 'dPosSmooth', 'initPos', 'quat', 't', 'tm', 'm', 'LL');
            fprintf('数据已对齐并存储为%s。\n', filename);
            toClear = setdiff(who, 'filename');
            clear(toClear{:});
            
            exit_flag = true; % 设置退出标志
            
        else
            % 其他输入，提示用户重新输入
            disp('无效指令，请重新输入。');
        end
    end

    % 程序结束
    disp('程序已退出。');
    rosshutdown;
    

    % 回调函数定义
    function processImuData(~, msg)
        % 提取IMU数据：时间戳、线加速度、角速度、方向四元数
        if isempty(imu)
                imu = struct('Time', {},'Accel', {},'Orient', {});
        end
        imu_data.Time = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec * 1e-9;
        imu_data.Accel = [msg.LinearAcceleration.X, msg.LinearAcceleration.Y, msg.LinearAcceleration.Z];
%         imu_data.Gyro = [msg.AngularVelocity.X, msg.AngularVelocity.Y, msg.AngularVelocity.Z];
        imu_data.Orient = [msg.Orientation.X, msg.Orientation.Y, msg.Orientation.Z, msg.Orientation.W];
        imu(end+1)=imu_data;
            % 每隔50条显示数据
            % 定期同步到工作区（每50条数据）
        if mod(length(imu), 50) == 0
                disp('运动惯性数据: ');
                disp(imu_data);
                assignin('base', 'imu', imu);
        end
    end
    
    function processMag(~, msg)
        %     mag_data = receive(msg,10);
            if isempty(mag)
                mag = struct('Time', {},'Mag', struct('X', {},'Y', {},'Z',{}), 'Seq', {});
            end
            
            updates.Time = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec * 1e-9;
            % 解析磁力强度（μT）
            updates.Mag.X = msg.Vector.X
            updates.Mag.Y = msg.Vector.Y 
            updates.Mag.Z = msg.Vector.Z
            updates.Seq = msg.Header.Seq;
         
            % 追加到持久变量
            mag(end+1) = updates;
            % 每隔20条显示数据
            % 定期同步到工作区（每50条数据）
            if mod(length(mag), 50) == 0
                disp('磁力话题发布: ');
                disp(updates);
                assignin('base', 'mag', mag);
            end
        end

    
    function processTimeRef(~, msg)
        % 提取时间参考
            if isempty(tr)
                tr = struct('t', {})
            end
            time.t = msg.TimeRef.Sec + msg.TimeRef.Nsec * 1e-9;
            tr(end+1) = time;
            if mod(length(tr), 30) == 0
                assignin('base', 'tr', tr);
            end
        end
    
    function processOdom(~, msg)
        % 提取里程计的位置和方向
        if isempty(odom)
            odom = struct('Time', {},'Position', struct('X', {},'Y', {},'Z',{}),'Orientation', struct('X', {},'Y', {},'Z',{},'W',{}),'Seq', {});
        end
        pos = msg.Pose.Pose.Position;
        orit = msg.Pose.Pose.Orientation;
        updates.Time = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec * 1e-9;
        updates.Position.X = pos.X;
        updates.Position.Y = pos.Y;
        updates.Position.Z = pos.Z;
        updates.Orientation.X = orit.X;
        updates.Orientation.Y = orit.Y;
        updates.Orientation.Z = orit.Z;
        updates.Orientation.W = orit.W;
        updates.Seq = msg.Header.Seq;
        odom(end+1) = updates
        if mod(length(odom), 50) == 0
            disp('里程计: ');
            disp(odom);
            assignin('base', 'odom', odom);
        end
    end
    
    function processQuat(~, msg)
        if isempty(quatn)
            quatn = struct('Time', {},'X', {},'Y', {},'Z', {},'W', {})
        end
%         disp(msg)
        % 提取四元数数据
        quaternion.Time = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec * 1e-9;
        quaternion.X = msg.Quaternion.X;
        quaternion.Y = msg.Quaternion.Y;
        quaternion.Z = msg.Quaternion.Z;
        quaternion.W = msg.Quaternion.W;
        quatn(end+1) = quaternion;
        if mod(length(quatn), 50) == 0
            assignin('base', 'quatn', quatn);
        end
    end
end