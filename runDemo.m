%% Main file to run one-dimensional magnetic field SLAM algorithm
% References:
%
%   [1] Manon Kok and Arno Solin. Online One-Dimensional Magnetic Field SLAM 
%   with Loop-Closure Detection


%% Load data
addpath('tools')
addpath('src')
rng(0,'twister')


groundtruthVisual("groundtruth/result-0310-4.txt")
% clear, clc, close all
% 
% bagfile=""
[filename]=collectRosData;
% Filenames
% filename="data/square-mag-03-11-08-14.mat"; 
filenames{1}=filename; 

% Settings
makePlots = 1;
makeResults = 1;
runMC = 0;
makePlotsMC = 0;
doComparison = 0;
makeVideo = 0;
visualiseOutput = 1;
savePlots = 0;

%% Run main algorithm
if makeResults
    % Drift and noise settings
    driftNoiseParams.bias = 0.005; %rad/s
    driftNoiseParams.sh2 = 1E-4;
    driftNoiseParams.sp2 = 1E-4; 

    indDataSet = 1;   
    rms_drs = [];
    rms_ekfs = [];
    if makeResults
        [MF,PF,xs,loop_start,loop_end,wp,wm,m_b,t,pos_odo,pos_gt] = ...
            magSLAMwithLoopClosures(filenames{indDataSet},driftNoiseParams,makePlots,visualiseOutput,makeVideo);
        % Visualization
        if makePlots
            [rms_dr,rms_ekf] = makePlotsSLAM(filenames{indDataSet},MF,PF,xs,loop_start,loop_end,wp,wm,m_b,t,pos_odo,pos_gt,savePlots);
            rms_drs = [rms_drs ; rms_dr];
            rms_ekfs = [rms_ekfs ; rms_ekf];
        end
    end
end
