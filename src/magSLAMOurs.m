function [MF,PF,xs,loop_start,loop_end,wp,wm,m_b,t,pos_odo,pos_gt] = ...
    magSLAMOurs(filename,driftNoiseParams,makePlots,visualiseOutput,makeVideo,axesHandle)
% magSLAMwithLoopClosures - EKF and RTS smoother for one-dimensional magnetic
% field SLAM with loop closures
%
% Syntax:
%   [MF,PF,xs,loop_start,loop_end,wp,wm,m_b,t,pos_odo,pos_gt] = ...
%     magSLAMwithLoopClosures(filename,driftNoiseParams,makePlots,visualiseOutput,makeVideo,axesHandle)
%
% In:
%   filename            - Filename to load data
%   driftNoiseParams    - Struct indicating what bias and what noise
%                           variances to use for generation of odometry data
%   makePlots           - Flag if code is run to make final plots (部分数据集不完全运行)
%   visualiseOutput     - Flag to indicate if plotting while running code
%   makeVideo           - Flag to indicate if making a video of the results
%   axesHandle          - （可选）当 visualiseOutput 为 true 时，若传入一个有效的 UIAxes，
%                         则将在该 axes 上动态绘制轨迹；否则使用默认 figure 显示。
%
% Out:
%   MF              - Struct with filtered state estimates
%   PF              - Struct with filtered state covariances
%   xs              - Array with smoothed position and heading states
%   loop_start      - Start indices of detected loops
%   loop_end        - End indices of detected loops
%   wp              - Position weights at final time instance (for plotting)
%   wm              - Magnetic weights at final time instance (for plotting)
%   m_b             - Heading angle derived from ARKit
%   t               - Sampling times
%   pos_gt          - Position derived from ARKit
%   pos_odo         - Position from odometry only
%
% Description:
%   Run EKF and RTS smoother for one-dimensional magnetic field SLAM with 
%   loop closures. See [1] for details.
%
% Reference:
%   [1] Manon Kok and Arno Solin. Online One-Dimensional Magnetic Field SLAM 
%       with Loop-Closure Detection
%
% Copyright:
%   2024-   Manon Kok and Arno Solin

% 如果没有传入 axesHandle，则设为空
if nargin < 6
    axesHandle = [];
end

%% Preprocessing and settings
disp(".........................................................")
disp("........... MagSLAM result will be show below............")
[dp,omega,m_b,t,pos_gt,pos_odo] = prepareData(filename,driftNoiseParams,visualiseOutput);
dt = diff(t);  

% Pre-allocate variables
count_loops = 0;          % Loop bookkeeping
loop_start = [];          % Loop start indices
loop_end = [];            % Loop end indices
MF = cell(1,size(dp,1));  % Filtered mean over time
MS = cell(1,size(dp,1));  % Smoothed mean over time
PF = cell(1,size(dp,1));  % Filtered covariance over time
GS = cell(1,size(dp,1));  % Smoother gain over time
MP = cell(1,size(dp,1));  % Predicted mean over time
PP = cell(1,size(dp,1));  % Predicted covariance over time
xs = zeros(3,size(dp,1)); % Smoothed position and heading state estimate

% Settings 
runRTSsmoother = true;
sm = 3;           % Magnetometer measurement noise
N_lag = 50;       % Do not loop close on the most recent 50 data points
N_lc = 9;         % N_lc + 1 indices to check before loop closure decision
gamma = 0.25;
gamma_ml = 1E-16;
N_dist = 10;
magThreshold = 3;
sigma2 = 0.01;    % Loop-closing measurement noise variance
beta_d = [1, 1, 0.5, 0.5];  % Directional Loop-closure detector 

% Process noise
Q = diag([driftNoiseParams.sp2 * ones(1,2), driftNoiseParams.sh2]);
if contains(filename,'mall') 
    Q = diag([driftNoiseParams.sp2 * ones(1,2), driftNoiseParams.sh2/10])*10;
end
if contains(filename,'library') 
    gamma_ml = 1E-1;
end

% EKF initialization
m0 = [0, 0, 0, 0]'; 
P0 = diag([1E-8 1E-8 1e-8 1e-4]);
if 3 * sqrt(P0(4,4)) < driftNoiseParams.bias 
    P0(4,4) = driftNoiseParams.bias^2;
end
m = m0;
P = P0;  
  
indLoopClosure = -inf;
 
% 如果不提供 axesHandle，则创建默认 figure 用于在线显示
if visualiseOutput && isempty(axesHandle)
    h = figure(1); clf
    % 调整图窗大小
    screenSize = get(0, 'ScreenSize');
    aspectRatio = [16 9];
    width = screenSize(3) * 0.5;
    height = width * (aspectRatio(2) / aspectRatio(1));
    figurePosition = [screenSize(3)/2 - width/2, screenSize(4)/2 - height/2, width, height];
    set(h, 'Position', figurePosition);
    set(h, 'Color', 'w')
    cmap = 1-gray;
    colormap(cmap)
end

%% Run extended Kalman filter
if makeVideo, VideoFrames(length(dp)) = struct('cdata',[],'colormap',[]); end
for k=1:size(dp,1)
    % Prediction step
    Pf = P;
    [m,F,G] = dynamics(m,dp(k,:)',omega(k),dt(k));
    P = F*P*F' + G*Q*G';
    P = (P+P')/2;
    mp = m;
    Pp = P;
    Gs = (Pf*F')/Pp;
    
    % Loop closure detection 
    loopClosure = 0;
    wm = zeros(k,4); % 扩展为四列：[fwd, bwd, lf, rt]
    wp = zeros(k,1);
    if k > N_lag + 1
        swp = mean(sqrt(diag(P(1:2,1:2))));
        d2 = sum((xs(1:2,1:k-1)-m(1:2)).^2,1);
        wp(1:k-1) = exp(-d2./swp.^2/8);
        for i = N_lc+1:k-N_lc
            % 当前窗口和历史窗口的磁场数据
            window_current = m_b(k-N_lc:k, :);
            window_candidate = m_b(i-N_lc:i, :);

            % 前向方向（fwd）
            dm2_fwd = sum((window_candidate - window_current).^2, 2);
            wm_fwd = prod(exp(-dm2_fwd/(12*sm^2/beta_d(1))));

            % 后向方向（bwd）：反转并旋转180度
            window_candidate_bwd = m_b(i+N_lc:-1:i, :);
            window_candidate_bwd = [-window_candidate_bwd(:,1), -window_candidate_bwd(:,2), window_candidate_bwd(:,3)];
            window_current_bwd = m_b(k-N_lc:k, :);
            window_current_bwd = [-window_current_bwd(:,1), -window_current_bwd(:,2), window_current_bwd(:,3)];
            dm2_bwd = sum((window_candidate_bwd - window_current_bwd).^2, 2);
            wm_bwd = prod(exp(-dm2_bwd/(12*sm^2/beta_d(2))));

            % 左转方向（lf）：比较左侧分量
            dm2_lf = sum((window_candidate(:,2) - window_current(:,2)).^2, 2);
            wm_lf = prod(exp(-dm2_lf/(12*sm^2/beta_d(3))));

            % 右转方向（rt）：比较右侧分量
            dm2_rt = sum((window_candidate(:,3) - window_current(:,3)).^2, 2);
            wm_rt = prod(exp(-dm2_rt/(12*sm^2/beta_d(4))));

            wm(i,1:4) = [wm_fwd, wm_bwd, wm_lf, wm_rt];
        end
        % 计算四方向最大概率
        combined_scores = max(wm(1:k-N_lag,:), [], 2) .* wp(1:k-N_lag);
        [maxProbLoopClosure, indLoopClosure] = max(combined_scores);
        
        % 其余判断条件保持不变
        if maxProbLoopClosure > gamma
            loopClosure = 1;
            if ismember(indLoopClosure,loop_start) || any(k-loop_end < N_dist)
                loopClosure = 0;
            end
            if norm(max(m_b(k-N_lc:k,:)) - min(m_b(k-N_lc:k,:))) < magThreshold
                loopClosure = 0; 
                if visualiseOutput, disp('Not enough excitation'); end 
            end
        end
    end

 

    if visualiseOutput
        % 确保 axesHandle 不是空的并且是有效的 UI 容器
        if ~isempty(axesHandle) && isvalid(axesHandle)
            % 检查是否已有子图，若存在则清空，否则创建新图
            ax1 = findobj(axesHandle, 'Type', 'axes', 'Tag', 'ax1');
            ax2 = findobj(axesHandle, 'Type', 'axes', 'Tag', 'ax2');
            ax3 = findobj(axesHandle, 'Type', 'axes', 'Tag', 'ax3');
    
            % 创建新的 uiaxes
            if isempty(ax1)
                ax1 = uiaxes('Parent', axesHandle, 'Position', [20 300 360 250], 'Tag', 'ax1');
            else
                cla(ax1);
            end
            if isempty(ax2)
                ax2 = uiaxes('Parent', axesHandle, 'Position', [400 50 560 500], 'Tag', 'ax2');
            else
                cla(ax2);
            end
            if isempty(ax3)
                ax3 = uiaxes('Parent', axesHandle, 'Position', [20 50 360 200], 'Tag', 'ax3');
            else
                cla(ax3);
            end
    
            % =========================
            % 在 ax2 绘制轨迹图
            hold(ax2, 'on');
            plot(ax2, pos_gt(:,1), pos_gt(:,2), '--g');  % 真实轨迹
            plot(ax2, xs(1,1:k), xs(2,1:k), '-b');      % SLAM 轨迹
            if numel(MF{k}) > 4
                plot(ax2, MF{k}(5:2:end), MF{k}(6:2:end), 'ob');
            end
            axis(ax2, 'square'); grid(ax2, 'on');
            if loopClosure
                plot(ax2, xs(1,indLoopClosure), xs(2,indLoopClosure), 'g*');
            end  
            hold(ax2, 'off');
    
            % =========================
            % 在 ax1 绘制磁场测量
            hold(ax1, 'on');
            imagesc(ax1, t(1:k), linspace(0,1,2), repmat(abs(wp(:))', 2, 1));
            plot(ax1, t(1:k), m_b(1:k,:));   
            xlim(ax1, [0-eps, t(k)]);
            ylim(ax1, [-50, 20]);
            xlabel(ax1, 'Time, t [s]');
            ylabel(ax1, 'Mag [uT]');
            box(ax1, 'on'); set(ax1, 'Layer', 'top');
            clim(ax1, [0 1]);
            hold(ax1, 'off');
    
            % =========================
            % 在 ax3 绘制权重图
            hold(ax3, 'on');
            plot(ax3, t(1:k), wp, '-b');    
            plot(ax3, t(1:k), wm, '-r');  
            plot(ax3, t(1:k), max(wm(1:end,:), [], 2).*wp, '-k', 'LineWidth', 2);  
            ylim(ax3, [0 1]);
            if k > 1
                xlim(ax3, [0, t(k)]);
            end
            xlabel(ax3, 'Time, t [s]');
            ylabel(ax3, 'Weight');
            box(ax3, 'on');
            hold(ax3, 'off');
    
            drawnow;
            if makeVideo
                VideoFrames(k) = getframe(axesHandle);
            end
    
        else
            % =========================
            % 如果未传入有效的 axesHandle，则使用默认 figure
            subplot(2,2,[2 4]); cla; hold on;
            plot(pos_gt(:,1), pos_gt(:,2), '--g')
            plot(xs(1,1:k), xs(2,1:k), '-b')
            if numel(MF{k}) > 4
                plot(MF{k}(5:2:end), MF{k}(6:2:end), 'ob')
            end
            axis square equal
            if loopClosure
                plot(xs(1,indLoopClosure), xs(2,indLoopClosure), 'g*')
            end
    
            subplot(221); cla; hold on;
            imagesc(t(1:k), [], repmat(abs(wp(:))', 2, 1))
            plot(t(1:k), m_b(1:k,:))   
            xlim([0-eps t(k)]); ylim([-50 20])
            xlabel('Time, t [s]')
            ylabel('Mag [uT]')
            box on, set(gca, 'Layer', 'top')
            clim([0 1])
    
            subplot(223); cla; hold on;
            plot(t(1:k), wp, '-b')    
            plot(t(1:k), wm, '-r')  
            plot(t(1:k), max(wm(1:end,:), [], 2).*wp, '-k', 'LineWidth', 2)  
            ylim([0 1])
            if k > 1
                xlim([0 t(k)])
            end
            xlabel('Time, t [s]'), ylabel('Weight')
            box on 
            drawnow;
            if makeVideo
                VideoFrames(k) = getframe(gcf);
            end
        end
    end

    % EKF measurement update and loop closure handling
    if loopClosure
        loop_start = [loop_start indLoopClosure];
        loop_end = [loop_end k]; 
        [MF_lc,MS_lc,PF_lc,GS_lc,MP_lc,PP_lc,flag] = ...
          run_filter_from_scratch(m0,P0,Q,dt,dp,omega,loop_start,loop_end,sigma2,gamma_ml,visualiseOutput);        
        if ~flag
            MF = MF_lc; MS = MS_lc; PF = PF_lc; GS = GS_lc; MP = MP_lc; PP = PP_lc;
            m = MF{k};
            P = PF{k};
            if visualiseOutput
                disp(['Current bias estimate: ' num2str(m(4))])
            end
            MS = MF(1:k);
            ms = MF{k};
            xs = zeros(3,k);
            xs(:,k) = m(1:3);
            for j = size(MS,2)-1:-1:1
                ms = MF{j} + GS{j+1}*(ms - MP{j+1});
                MS{j} = ms;
                xs(:,j) = ms(1:3);
            end
            count_loops = count_loops + 1;
            if visualiseOutput
                disp(['Current number of loops: ' num2str(count_loops)])
            end
        else
            loopClosure = 0;
            MP{k} = mp;
            PP{k} = Pp;
            GS{k} = Gs;
            MF{k} = m;
            PF{k} = P; 
            loop_start = loop_start(1:end-1);
            loop_end = loop_end(1:end-1);
            xs(:,k) = m(1:3);
        end
    else
        MP{k} = mp;
        PP{k} = Pp;
        GS{k} = Gs;
        MF{k} = m;
        PF{k} = P; 
        xs(:,k) = m(1:3);
    end
    
    % For making the plots in the paper, break conditions (可选)
    if makePlots && strcmp(filename,'data/square.mat') && (k == 734)
        VideoFrames = VideoFrames(1:k);
        break;
    end
    if makePlots && strcmp(filename,'data/eight.mat') && (k == 234)
        VideoFrames = VideoFrames(1:k);
        break;
    end
end

if makeVideo
    vidObj = VideoWriter(['1d-magnetic-slam-' filename(6:end-4) '.mp4'],'MPEG-4'); 
    open(vidObj);
    for i=1:length(VideoFrames)
        writeVideo(vidObj, VideoFrames(i));
    end
    close(vidObj);
end

end
