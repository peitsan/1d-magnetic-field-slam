classdef MainApp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        StartButton                  matlab.ui.control.Button
        CancelButton               matlab.ui.control.Button
        SaveDataButton             matlab.ui.control.Button
        ReconstructButton          matlab.ui.control.Button 
        StatusLabel                matlab.ui.control.Label
        TabGroup                   matlab.ui.container.TabGroup
        MainTab                    matlab.ui.container.Tab
        ResultsTab                 matlab.ui.container.Tab
        ProgressPanel              matlab.ui.container.Panel
        ButtonPanel                matlab.ui.container.Panel
        StatusPanel                matlab.ui.container.Panel
        ResPlotPanel               matlab.ui.container.Panel
        TerminalTextArea           matlab.ui.control.TextArea
        ResUIAxes                  matlab.ui.control.UIAxes
        IsRecording                logical
        RecordedData               {}
        Filenames                  cell
        MakePlots                  logical
        MakeResults                logical
        RunMC                      logical
        MakePlotsMC                logical
        DoComparison               logical
        MakeVideo                  logical
        VisualiseOutput            logical
        SavePlots                  logical
        ControlParams              struct
        Timer                      
        RecordingStartTime         datetime             % 录制开始时间
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: StartButton
        function StartButtonPushed(app, ~)
            global exitFlag saveFlag 
            try
                saveFlag = false;
                exitFlag = false;
                app.StatusLabel.Text = '加载参数...';
                addpath('tools');
                addpath('src');
                rng(0,'twister');
                app.StatusLabel.Text = '参数加载完成';
                app.StatusLabel.Text = '准备录制数据...';
                app.StartButton.Enable = 'off';
                app.CancelButton.Enable = 'on';
                app.SaveDataButton.Enable = 'on';
                app.ReconstructButton.Enable = 'off';
                app.IsRecording = true;
                app.StatusLabel.Text = '录制数据中...';
                
                % 开始计时
                app.RecordingStartTime = datetime('now');
                app.Timer = timer('TimerFcn', @(~,~) app.UpdateTimer(app), 'StartDelay', 1, 'ExecutionMode', 'fixedRate', 'Period', 1);
                start(app.Timer);
                
                [app.Filenames{1}] = collectRosData();
                app.StatusLabel.Text = ['数据文件: ', app.Filenames{1}];
                
                % 停止计时
                stop(app.Timer);
                delete(app.Timer);
                app.Timer = [];
                
            catch ME
                app.StatusLabel.Text = ['发生错误: ', ME.message];
            end
        end

        % Button pushed function: CancelButton
        function CancelButtonPushed(app, ~)
            global exitFlag
            exitFlag = true; % 设置取消录制标志
            app.IsRecording = false;
            app.StatusLabel.Text = '录制已取消';
            app.StartButton.Enable = 'on';
            app.CancelButton.Enable = 'off';
            app.SaveDataButton.Enable = 'off';
            app.ReconstructButton.Enable = 'off';
            
            % 停止计时
            if ~isempty(app.Timer)
                stop(app.Timer);
                delete(app.Timer);
                app.Timer = [];
            end
        end
        
        % 更新计时器显示
        function UpdateTimer(app, ~)
            if app.IsRecording
                elapsedTime = minutes(datetime('now') - app.RecordingStartTime);
                app.StatusLabel.Text = ['录制数据中... 已录制时间: ', num2str(elapsedTime), ' 分钟'];
            end
        end

        % Button pushed function: SaveDataButton
        function SaveDataButtonPushed(app, ~)
             global saveFlag
             saveFlag = true; % 设置保存数据标志
             app.StatusLabel.Text = '保存录制数据...';
             app.StartButton.Enable = 'on';
             app.CancelButton.Enable = 'off';
             app.SaveDataButton.Enable = 'off';
             app.ReconstructButton.Enable = 'on';
             app.IsRecording = false;
        end
     
        % Button pushed function: ReconstructButton
        function ReconstructButtonPushed(app, ~)
            try
                % 设置参数
                app.IsRecording = false;
                driftNoiseParams.bias = 0.005; %rad/s
                driftNoiseParams.sh2 = 1E-4;
                driftNoiseParams.sp2 = 1E-4; 
                indDataSet = 1;   
                rms_drs = [];
                rms_ekfs = [];
                disp("SLAM")
              
                % 先创建 ResultsTab，再创建 UIAxes
                app.ResultsTab = uitab(app.TabGroup, 'Title', '轨迹重建结果');
                
                app.ResPlotPanel = uipanel('Parent', app.ResultsTab);
                app.ResPlotPanel.Position = [20, 20, 920, 600];
                app.ResPlotPanel.BackgroundColor = [0.95 0.95 0.95];
                app.TabGroup.SelectedTab = app.ResultsTab;
                [MF,PF,xs,loop_start,loop_end,wp,wm,m_b,t,pos_odo,pos_gt] = ...
                        magSLAMOurs(app.Filenames{indDataSet},driftNoiseParams,app.MakePlots,app.VisualiseOutput,app.MakeVideo,app.ResPlotPanel);
          
                disp("SLAMoff....")
                
                % 更新 GUI 状态
                app.StatusLabel.Text = '算法运行完成，结果显示在新选项卡中'; 
                % 生成其他结果
                if app.MakePlots
                    [rms_dr,rms_ekf] = makePlotsSLAM(app.Filenames{indDataSet},MF,PF,xs,loop_start,loop_end,wp,wm,m_b,t,pos_odo,pos_gt,app.SavePlots);
                    rms_drs = [rms_drs ; rms_dr];
                    rms_ekfs = [rms_ekfs ; rms_ekf];
                end
                
                % 显示关闭 figure 的弹窗
                response = uialert(app.UIFigure, '是否关闭 Figure 窗口？', '操作完成', 'Style', 'question', 'Buttons', {'是', '否'});
                switch response
                    case '是'
                        if ~isempty(fig)
                            close(fig);
                        end
                    case '否'
                        % 不关闭 figure
                end
                
            catch ME
                app.StatusLabel.Text = ['发生错误: ', ME.message];
            end
            app.StartButton.Enable = 'on';
            app.CancelButton.Enable = 'off';
            app.SaveDataButton.Enable = 'off';
            app.ReconstructButton.Enable = 'off';
        end
    end
    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)
            % Create and hide UIFigure until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1000 800];
            app.UIFigure.Name = 'ROS数据录制与分析工具';
            app.UIFigure.Color = [1 1 1]; % 设置背景为白色
            
            % 创建选项卡组
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [20 20 960 680];
            
            % 创建主选项卡
            app.MainTab = uitab(app.TabGroup, 'Title', '主界面');
            
            % Create ProgressPanel
            app.ProgressPanel = uipanel(app.MainTab);
            app.ProgressPanel.Title = '进度状态';
            app.ProgressPanel.Position = [20 580 920 80];
            app.ProgressPanel.BackgroundColor = [0.95 0.95 0.95];
            
            % Create StatusLabel
            app.StatusLabel = uilabel(app.ProgressPanel);
            app.StatusLabel.Position = [20 40 880 22];
            app.StatusLabel.Text = '欢迎使用ROS数据录制与分析工具';
            app.StatusLabel.FontSize = 12;
            
            % Create ButtonPanel
            app.ButtonPanel = uipanel(app.MainTab);
            app.ButtonPanel.Title = '操作控制';
            app.ButtonPanel.Position = [20 480 920 80];
            app.ButtonPanel.BackgroundColor = [0.95 0.95 0.95];
            
            % Create StartButton
            app.StartButton = uibutton(app.ButtonPanel, 'push');
            app.StartButton.Position = [40 20 200 40];
            app.StartButton.Text = '开始录制';
            app.StartButton.FontSize = 12;
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);
            
            % Create CancelButton
            app.CancelButton = uibutton(app.ButtonPanel, 'push');
            app.CancelButton.Position = [270 20 200 40];
            app.CancelButton.Text = '取消录制';
            app.CancelButton.Enable = 'off';
            app.CancelButton.FontSize = 12;
            app.CancelButton.ButtonPushedFcn = createCallbackFcn(app, @CancelButtonPushed, true);
            
            % Create SaveDataButton
            app.SaveDataButton = uibutton(app.ButtonPanel, 'push');
            app.SaveDataButton.Position = [490 20 200 40];
            app.SaveDataButton.Text = '保存录制';
            app.SaveDataButton.Enable = 'off';
            app.SaveDataButton.FontSize = 12;
            app.SaveDataButton.ButtonPushedFcn = createCallbackFcn(app, @SaveDataButtonPushed, true);
            
            % Create ReconstructButton
            app.ReconstructButton = uibutton(app.ButtonPanel, 'push');
            app.ReconstructButton.Position = [710 20 200 40];
            app.ReconstructButton.Text = '轨迹重建';
            app.ReconstructButton.Enable = 'off';
            app.ReconstructButton.FontSize = 12;
            app.ReconstructButton.ButtonPushedFcn = createCallbackFcn(app, @ReconstructButtonPushed, true);
            
            % 创建终端显示区域
            app.TerminalTextArea = uitextarea(app.MainTab);
            app.TerminalTextArea.Position = [20 670 920 90];
            app.TerminalTextArea.FontSize = 10;
            app.TerminalTextArea.Enable = 'off';
            
            % 设置参数
            app.MakePlots = true;
            app.MakeResults = true;
            app.RunMC = false;
            app.MakePlotsMC = false;
            app.DoComparison = false;
            app.MakeVideo = false;
            app.VisualiseOutput = true;
            app.SavePlots = false;
            app.ControlParams = struct('start', false, 'cancel', false, 'save', false, 'data', []);
            
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = MainApp
            global exitFlag saveFlag
            exitFlag = false;
            saveFlag = false;
            createComponents(app);
            
            % Register the app with App Designer
            registerApp(app, app.UIFigure);
            
            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.UIFigure);
        end
    end
end