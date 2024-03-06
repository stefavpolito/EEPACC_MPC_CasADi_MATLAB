clearvars;clc;close all;
addpath('.\casadi_3_6_3')
addpath(genpath('.\Functions\'))
addpath(genpath('.\DrivingCycles'))
V = SetVehicleParameters();

%% General settings
% 
% % === other settings ===
% % whether to plot additional plots (comfort, calculation times, headway, etc.)
% % createAdditionalPlots  = true;
% % whether to create gifs of the receding horizon process of the MPCs
% % OPTsettings.createGifs = false;
% % whether to include a target vehicle (generated if not in use case)
% OPTsettings.IncludeTV  = true;
% % whether to use the simulation time from a use case instead of the one set below
% useUseCaseTsim         = false;
% 
% % == use cases ==
% %  1: accelerating from standstill to 80 km/h
% %  2: braking to standstill from 80 km/h
% %  3: several changes in the speed limit
% %  4: approaching a stop
% %  5: driving a route with several traffic lights
% %  6: approaching and driving through several curves
% %  7: climbing and descending a hill
% %  8: driving behind an aggressive target vehicle
% %  9: driving behind a less aggressive target vehicle
% % 10: target vehicle cutting into the lane
% % 11: long route scenario
% % 12: scenario for the video
% OPTsettings.useCaseNum = 0;
% 
% % time step and simulation time
% OPTsettings.Ts     = 0.5;    % controller sample time (s) (MUST BE a multiple of .1 seconds!)
% if ~useUseCaseTsim
%     % if a custom simulation time is used
%     % OPTsettings.t_sim  = 60; % total simulation time  (s)
%     OPTsettings.t_sim  = 435; % WLTC-city simulation time  (s)
% end
% 
% %% Processing
% 
% % Get other settings
% OPTsettings = Settings(OPTsettings);
% cutOffDist = OPTsettings.cutOffDist;
% 
% if OPTsettings.useCaseNum == 0 % custom use case
%     % if TV is requested, it must be generated
%     OPTsettings.generateTVMPC = true;
% end
% 
% % transpose the target vehicle vectors if required
% if isfield(OPTsettings,'s_tv')
%     if size(OPTsettings.s_tv,1) < size(OPTsettings.s_tv,2) 
%         OPTsettings.s_tv = OPTsettings.s_tv';
%         OPTsettings.v_tv = OPTsettings.v_tv';
%     end
% end
% 
% % Target vehicle trajectory generation
% if ~OPTsettings.IncludeTV
%     % no target vehicle: s=inf, v=0
%     if ~isfield(OPTsettings,'s_tv') || isempty(OPTsettings.s_tv)
%         OPTsettings.s_tv = inf*ones(1+OPTsettings.t_sim/OPTsettings.Tvec(1),1);
%         OPTsettings.v_tv =    zeros(1+OPTsettings.t_sim/OPTsettings.Tvec(1),1);
%     end
% else
%     if OPTsettings.generateTVMPC
%         % use an MPC to generate the target vehicle's trajectory
%         fprintf ('=== GENERATING TARGET VEHICLE TRAJECTORY ===\n\n');
%         % [OPTsettings.s_tv, OPTsettings.v_tv, TVbadExitMessages] = RunOpt_TVMPC(OPTsettings);
%         [OPTsettings.s_tv, OPTsettings.v_tv, TVbadExitMessages] = Run_DrivingCycle(OPTsettings);
%         % shift to the back of the vehicle
%         OPTsettings.s_tv = OPTsettings.s_tv - OPTsettings.TVlength;
%     end
% end

%% Prepare Simulink Simulation

import casadi.*

T = 10; % Time horizon
N = 20; % number of control intervals

% Declare model variables
x1 = SX.sym('x1');
x2 = SX.sym('x2');
x = [x1; x2];
u = SX.sym('u');

% Model equations
xdot = [(1-x2^2)*x1 - x2 + u; x1];

% Objective term
L = x1^2 + x2^2 + u^2;

% Continuous time dynamics
f = Function('f', {x, u}, {xdot, L});

% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
M = 4; % RK4 steps per interval
DT = T/N/M;
X0 = MX.sym('X0', 2);
U = MX.sym('U');
X = X0;
Q = 0;
for j=1:M
   [k1, k1_q] = f(X, U);
   [k2, k2_q] = f(X + DT/2 * k1, U);
   [k3, k3_q] = f(X + DT/2 * k2, U);
   [k4, k4_q] = f(X + DT * k3, U);
   X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
   Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% "Lift" initial conditions
X0 = MX.sym('X0', 2);
w = {w{:}, X0};
lbw = [lbw; 0; 1];
ubw = [ubw; 0; 1];
w0 = [w0; 0; 1];

% Formulate the NLP
Xk = X0;
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)]);
    w = {w{:}, Uk};
    lbw = [lbw; -1];
    ubw = [ubw;  1];
    w0 = [w0;  0];

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], 2);
    w = {w{:}, Xk};
    lbw = [lbw; -0.25; -inf];
    ubw = [ubw;  inf;  inf];
    w0 = [w0; 0; 0];

    % Add equality constraint
    g = {g{:}, Xk_end-Xk};
    lbg = [lbg; 0; 0];
    ubg = [ubg; 0; 0];
end

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
options = struct('ipopt',struct('print_level',0),'print_time',false);
solver = nlpsol('solver', 'ipopt', prob, options);

s0 = MX.sym('s0',2);

lbw_sym = MX(lbw);
ubw_sym = MX(ubw);
lbw_sym(1:2) = s0;
ubw_sym(1:2) = s0;

sol_sym = solver('x0', w0, 'lbx', lbw_sym, 'ubx', ubw_sym,...
            'lbg', lbg, 'ubg', ubg);

% Mapping from initial state to control action

function_name = 'f';
f = Function(function_name,{s0},{sol_sym.x(3)});

file_name = 'f.casadi';
f.save(file_name);

lib_path = GlobalOptions.getCasadiPath();
inc_path = GlobalOptions.getCasadiIncludePath();
mex('-v',['-I' inc_path],['-L' lib_path],'-lcasadi', 'casadi_fun.c')

%%
open_system('ACCMPC_sfun')

