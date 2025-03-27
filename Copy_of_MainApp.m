classdef MainApp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        StartButton                matlab.ui.control.Button
        CancelButton               matlab.ui.control.Button
        RunButton                  matlab.ui.control.Button
        StatusLabel                matlab.ui.control.Label
        FigureDisplayAxes          matlab.ui.control.UIAxes
        TabGroup                   matlab.ui.container.TabGroup
        MainTab                    matlab.ui.container.Tab
        ResultsTab                 matlab.ui.container.Tab
        ProgressPanel              matlab.ui.container.Panel
        ButtonPanel                matlab.ui.container.Panel
        StatusPanel                matlab.ui.container.Panel
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
        TerminalTextArea           matlab.ui.control.TextArea
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
                app.RunButton.Enable = 'on';
                app.IsRecording = true;
                app.StatusLabel.Text = '录制数据中...';
                [app.Filenames{1}] = collectRosData();
                app.StatusLabel.Text = ['数据文件: ', app.Filenames{1}];
    
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
            app.RunButton.Enable = 'off';
        end
        function RunSLAM(app, ~)
                global existFlag
                existFlag = false
              % 设置参数
                driftNoiseParams.bias = 0.005; %rad/s
                driftNoiseParams.sh2 = 1E-4;
                driftNoiseParams.sp2 = 1E-4; 
                indDataSet = 1;   
                rms_drs = [];
                rms_ekfs = [];
%                 disp("SLAMing....")
%                 while ~existFlag
%                 % 运行主算法
%                     if ~evalin('base', 'exist(''filename'', ''var'')')
%                         if exist(fullfile(['' ...
%                                 './'], filename), 'file') == 2
%                             break;
%                         end
%                     end  
%                 end
                while ~existFlag 
                end
                disp("SLAM")
                [MF,PF,xs,loop_start,loop_end,wp,wm,m_b,t,pos_odo,pos_gt] = ...
                        magSLAMwithLoopClosures(filename,driftNoiseParams,app.MakePlots,app.VisualiseOutput,app.MakeVideo);
                disp("SLAMoff....")
                % 显示生成的figure
                figs = findall(0, 'Type', 'figure');
                foundFigure = false;
                    
                    for i = 1:length(figs)
                        if strcmp(figs(i).Name, 'Algorithm Results')
                            % 创建新的选项卡显示结果
                            app.ResultsTab = uitab(app.TabGroup, 'Title', '结果');
                            newAxes = uiaxes(app.ResultsTab);
                            newAxes.Position = [20 20 920 520];
                            
                            % 将figure的内容复制到新的UIAxes中
                            copyobj(figs(i).Children, newAxes);
                            delete(figs(i));
                            foundFigure = true;
                            break;
                        end
                    end
                        
                if ~foundFigure
                    app.StatusLabel.Text = '算法运行完成，但未找到结果显示。'; 
                else
                    app.StatusLabel.Text = '算法运行完成，结果显示在新选项卡中'; 
                    if app.MakePlots
                        [rms_dr,rms_ekf] = makePlotsSLAM(app.Filenames{indDataSet},MF,PF,xs,loop_start,loop_end,wp,wm,m_b,t,pos_odo,pos_gt,app.SavePlots);
                        rms_drs = [rms_drs ; rms_dr];
                        rms_ekfs = [rms_ekfs ; rms_ekf];
                    end
                end
                app.StartButton.Enable = 'on';
                
        end
        % Button pushed function: RunButton
        function RunButtonHandle(app, ~)
            global saveFlag
            saveFlag = true; % 设置保存数据标志
            app.StatusLabel.Text = '加载参数...';
            addpath('tools');
            addpath('src');
            rng(0,'twister');
            app.StatusLabel.Text = '参数加载完成';
            app.IsRecording = false;
            app.StatusLabel.Text = '运行算法...';
            app.StartButton.Enable = 'off';
            app.CancelButton.Enable = 'off';
            app.RunButton.Enable = 'off';
        end
        function RunButtonPushed(app, ~)
           try RunButtonHandle(app);
           catch
           end
           try
                RunSLAM(app);
           catch
           end
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
            
            % Create RunButton
            app.RunButton = uibutton(app.ButtonPanel, 'push');
            app.RunButton.Position = [490 20 200 40];
            app.RunButton.Text = '运行算法';
            app.RunButton.Enable = 'off';
            app.RunButton.FontSize = 12;
            app.RunButton.ButtonPushedFcn = createCallbackFcn(app, @RunButtonPushed, true);
            
            % Create SettingsPanel
            app.StatusPanel = uipanel(app.MainTab);
            app.StatusPanel.Title = '结果显示';
            app.StatusPanel.Position = [20 20 920 440];
            app.StatusPanel.BackgroundColor = [0.95 0.95 0.95];
            
            % Create FigureDisplayAxes
            app.FigureDisplayAxes = uiaxes(app.StatusPanel);
            app.FigureDisplayAxes.Position = [20 20 880 400];
            
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
            
            % Show the figure after components are created
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