function [s_opt,v_opt,numSolverErrors] = Run_DrivingCycle(OPTsettings)
% PrecVehMPC Runs the target vehicle MPC over the simulation
%
%   Inputs:
%       OPTsettings : struct with settings for the MPC
%
%   Outputs
%       s_opt : travel distance array of the target vehicle over the simulation
%       v_opt : velocity array of the target vehicle over the simulation

%% Import Driving Cyle

load TO01_EAD.mat V_TO

% Resample to follow the Ts chosen
V_TO = resample(V_TO,1,5);
V_TO(V_TO<0.1) = 0;

%% Unpack settings

% optimization settings
N_hor           = OPTsettings.TV_N_hor;
solverToUse     = OPTsettings.solverToUse;
Ts              = OPTsettings.TV_Ts;
t_sim           = OPTsettings.t_sim;
s_0             = OPTsettings.TVinitDist;
v_0             = OPTsettings.TVinitVel;
a_minus1        = OPTsettings.a_minus1;

% get vehicle parameters
V = SetVehicleParameters();

%% Initialize Driving Cycle and create space vector

n_cycle = t_sim/Ts;
v_opt = zeros(n_cycle+1,1);
s_opt = zeros(n_cycle+1,1);
s_opt(1,1) = s_0;

for i = 2:n_cycle
    v_opt(i,1) = V_TO(i,1);
    s_opt(i,1) = s_opt(i-1,1) + Ts*V_TO(i,1);
end

v_opt(n_cycle+1,1) = v_opt(n_cycle,1);
s_opt(n_cycle+1,1) = s_opt(n_cycle,1);

numSolverErrors = 0;
end