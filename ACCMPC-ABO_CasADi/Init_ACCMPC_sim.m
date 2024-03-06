clearvars;clc;close all;
addpath('.\casadi_3_6_3')
addpath(genpath('.\Functions\'))
addpath(genpath('.\DrivingCycles'))
V = SetVehicleParameters();

%% General settings

% === other settings ===
% whether to plot additional plots (comfort, calculation times, headway, etc.)
% createAdditionalPlots  = true;
% whether to create gifs of the receding horizon process of the MPCs
% OPTsettings.createGifs = false;
% whether to include a target vehicle (generated if not in use case)
OPTsettings.IncludeTV  = true;
% whether to use the simulation time from a use case instead of the one set below
useUseCaseTsim         = false;

% == use cases ==
%  1: accelerating from standstill to 80 km/h
%  2: braking to standstill from 80 km/h
%  3: several changes in the speed limit
%  4: approaching a stop
%  5: driving a route with several traffic lights
%  6: approaching and driving through several curves
%  7: climbing and descending a hill
%  8: driving behind an aggressive target vehicle
%  9: driving behind a less aggressive target vehicle
% 10: target vehicle cutting into the lane
% 11: long route scenario
% 12: scenario for the video
OPTsettings.useCaseNum = 0;


% time step and simulation time
OPTsettings.Ts     = 0.5;    % controller sample time (s) (MUST BE a multiple of .1 seconds!)
if ~useUseCaseTsim
    % if a custom simulation time is used
    % OPTsettings.t_sim  = 60; % total simulation time  (s)
    OPTsettings.t_sim  = 435; % WLTC-city simulation time  (s)
end

%% Processing

% Get other settings
OPTsettings = Settings(OPTsettings);
cutOffDist = OPTsettings.cutOffDist;

if OPTsettings.useCaseNum == 0 % custom use case
    % if TV is requested, it must be generated
    OPTsettings.generateTVMPC = true;
end

% transpose the target vehicle vectors if required
if isfield(OPTsettings,'s_tv')
    if size(OPTsettings.s_tv,1) < size(OPTsettings.s_tv,2) 
        OPTsettings.s_tv = OPTsettings.s_tv';
        OPTsettings.v_tv = OPTsettings.v_tv';
    end
end

% Target vehicle trajectory generation
if ~OPTsettings.IncludeTV
    % no target vehicle: s=inf, v=0
    if ~isfield(OPTsettings,'s_tv') || isempty(OPTsettings.s_tv)
        OPTsettings.s_tv = inf*ones(1+OPTsettings.t_sim/OPTsettings.Tvec(1),1);
        OPTsettings.v_tv =    zeros(1+OPTsettings.t_sim/OPTsettings.Tvec(1),1);
    end
else
    if OPTsettings.generateTVMPC
        % use an MPC to generate the target vehicle's trajectory
        fprintf ('=== GENERATING TARGET VEHICLE TRAJECTORY ===\n\n');
        % [OPTsettings.s_tv, OPTsettings.v_tv, TVbadExitMessages] = RunOpt_TVMPC(OPTsettings);
        [OPTsettings.s_tv, OPTsettings.v_tv, TVbadExitMessages] = Run_DrivingCycle(OPTsettings);
        % shift to the back of the vehicle
        OPTsettings.s_tv = OPTsettings.s_tv - OPTsettings.TVlength;
    end
end

%% Prepare Simulink Simulation

% Target = Run the Target Vehicle and the ABMPC in closed-loop in Simulink

% Acceleration-based MPC
fprintf ('\n=== ACCELERATION-BASED MPC ===\n\n');
run('RunInit_ABMPC');














%% Show results

%% Get data

% === Minimum velocity incentive ===
    % 
    % % get the minimum of all maximum speeds excluding stops
    % [s_velInc,v_velInc] = minPWA(OPTsettings.s_speedLim, OPTsettings.v_speedLim, OPTsettings.s_curv, OPTsettings.alpha_TTL.*(abs(OPTsettings.curvature)).^(-1/3));
    % 
    % % saturate slopes
    % [s_velInc,v_velInc] = SaturateSlopePWA(s_velInc,v_velInc,0.5);
    % 
    % % delete duplicates
    % s_minVel_ = s_velInc;
    % v_minVel_ = v_velInc;
    % pointsToKeep = ~diff(s_velInc)==0;
    % s_velInc = [s_velInc(pointsToKeep),s_minVel_(end)];
    % v_velInc = [v_velInc(pointsToKeep),v_minVel_(end)];
    % 
    % % simplify
    % [s_velInc,v_velInc] = SimplifyPWA(s_velInc,v_velInc);

%% Plot solutions

% Ts = OPTsettings.Tvec(1);
% plotVehicleVollowingPlots = isfield(OPTsettings,'s_tv') && (sum(OPTsettings.s_tv < 1e6) > 1);
% t_plot    = 0:Ts:OPTsettings.t_sim;
% t_plot_TV = t_plot;

%% states and control signals over time

%% construct plot for stops

% velocity and control signals over distance with distance-based
% constraints visualized and energy consumption

%% Distance over time
    
    %% acceleration and jerk over time
    
    %% Filtered acceleration and jerk over time
    
        %% Headway distance over velocity
        
        %% Headway distance over time  
        
        %% Headway time over time    
    
    %% Energy consumption over time
    
    %% Energy consumption over distance
    
    %% MPC loop times
    
    %% MPC solving times
    
    %% determine motor torque limits
    
    %% Acceleration constraint investigation
    
    %% Jerk constraint investigation
    
    %% running costs
