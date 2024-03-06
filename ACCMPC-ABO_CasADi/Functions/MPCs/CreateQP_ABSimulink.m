function [H,c,G,z_lb,z_ub,g_lb,g_ub] = CreateQP_ABSimulink(OPTsettings,n_x,n_u,s_0,v_0,s_est,v_est,s_tv_est,t_0,a_minus1)
% CreateQP_AB Creates the acceleration-based QP for qpOASES and HPIPM, i.e. it builds the matrices and vectors that define the optimization problem
%
%   Inputs:
%       OPTsettings : struct with settings for the optimization
%       n_x         : number of states in each time step
%       n_u         : number of controls in each time step
%       s_0         : initial travel distance for the QP
%       v_0         : initial velocity for the QP
%       s_est       : estimated travel distance over the horizon
%       v_est       : estimated velocity over the horizon
%       s_tv_est    : estimated travel distance of the target vehicle over the horizon
%       t_0         : initial time for the QP
%       a_minus1    : acceleration in the previous time step
%
%   Outputs:
%       H    : Hessian matrix (matrix of quadratic term in objective)
%       c    : vector defining the linear terms in the objective
%       G    : matrix defining the linear constraints
%       z_lb : vector with lower bounds of all optimization variables
%       z_ub : vector with upper bounds of all optimization variables
%       g_lb : vector with lower bounds of the constraints defined in G
%       g_ub : vector with upper bounds of the constraints defined in G

%% Extract settings

solverToUse = OPTsettings.solverToUse;
Tvec        = OPTsettings.Tvec;
W           = OPTsettings.W_AB;
tau_min     = OPTsettings.tau_min;
h_min       = OPTsettings.h_min;
N           = 20;
s_goal      = OPTsettings.s_goal;
Mb          = OPTsettings.Mb;

% get vehicle parameter struct
% V = SetVehicleParameters();
V = SetParametersInternally();

% extract weights
w_FC = W(1);
w_a = W(2);
w_j = W(3);
w_v = W(4);
w_h = W(5);
w_s = W(6);
w_f = W(7);

%% Initialize

% Estimate bounds on velocity from the route and comfort limits
[~,v_lim_max,v_stop_max,v_TL_max,v_curv_max,a_min_est,a_max_est,...
    j_min_est,j_max_est,tau_est] = EstimateBoundsInternally(OPTsettings, 0, s_est, v_est, t_0, N);

% Estimate minimum velocity to use as an incentive to travel
v_minIncentive = min(cat(2,v_lim_max,v_curv_max),[],2);

% initialize vectors and matrices
n_c   = n_x + N*30;
n_x_u = n_x + n_u;
n_z   = n_x_u*N;
H = zeros(n_z+n_x,n_z+n_x);
c = zeros(n_z+n_x,1);
G = zeros(n_c,n_z+n_x);
g_lb = zeros(n_c,1);
g_ub = g_lb;

% bounds on the state and control variables
s_min       = 0;                        % minimum distance (= pos relative to origin)  (m)
s_max       = s_goal;                   % maximum distance                             (m)
v_min       = 0;                        % minimum velocity                             (m/s)
v_max       = V.v_max;                  % maximum velocity                             (m/s)
a_min       = -8;                       % minimum acceleration                         (m/s^2)
a_max       = 8;                        % maximum acceleration                         (m/s^2)
slack_min   = 0;
slack_max   = inf;

%% Set bounds on z

% Initialize the bounds
z_lb = zeros(n_x_u*N+n_x,1);
z_ub = z_lb;

if solverToUse == 2
% HPIPM
    % Minimum and maximum bounds on states and controls
    z_lb(1:n_x_u:(n_x_u*N+n_x)) = s_min;       % s
    z_ub(1:n_x_u:(n_x_u*N+n_x)) = s_max;
    z_lb(2:n_x_u:(n_x_u*N+n_x)) = v_min;       % v
    z_ub(2:n_x_u:(n_x_u*N+n_x)) = v_max;
    z_lb(3:n_x_u:(n_x_u*N+n_x)) = a_min;       % a (prev)
    z_ub(3:n_x_u:(n_x_u*N+n_x)) = a_max;
    z_lb(4:n_x_u:(n_x_u*N+n_x)) = a_min;       % a (curr)
    z_ub(4:n_x_u:(n_x_u*N+n_x)) = a_max;
    z_lb(5:n_x_u:(n_x_u*N+n_x)) = slack_min;   % xi_v
    z_ub(5:n_x_u:(n_x_u*N+n_x)) = slack_max;
    z_lb(6:n_x_u:(n_x_u*N+n_x)) = slack_min;   % xi_h
    z_ub(6:n_x_u:(n_x_u*N+n_x)) = slack_max;
    z_lb(7:n_x_u:(n_x_u*N+n_x)) = slack_min;   % xi_s
    z_ub(7:n_x_u:(n_x_u*N+n_x)) = slack_max;
    z_lb(8:n_x_u:(n_x_u*N+n_x)) = slack_min;   % xi_f
    z_ub(8:n_x_u:(n_x_u*N+n_x)) = slack_max;
else
% qpOASES:
    % set lower and upper bounds of z (all infinite)
    if solverToUse == 1
        % dense
        z_lb = -inf*ones(N*n_u,1);
        z_ub = -z_lb;
    else
        % sparse
        z_lb = -inf*ones(N*(n_x+n_u)+n_x,1);
        z_ub = -z_lb;
    end
end

%% Set objective function and constraints (G and its bounds)

if solverToUse == 2
% HPIPM
    % indices of z (mod n_x_u + 1):
    scurr   = 1;        % distance                           s 
    vcurr   = 2;        % velocity                           v
    aprev   = 3;        % previous acceleration              a_prev
    acurr   = 4;        % acceleration                       a
    xi_v    = 5;        % velocity incentive slack variable  xi_v
    xi_h    = 6;        % headway slack variable             xi_h
    xi_s    = 7;        % safety-critical feasibility slack  xi_s
    xi_f    = 8;        % feasibility slack variable         xi_f
else
% qpOASES
    % indices of z (mod n_x_u + 1):   
    scurr   = 1;        % distance                           s 
    vcurr   = 2;        % velocity                           v
    aprev   = 3-n_x_u;  % previous acceleration              a_prev
    acurr   = 3;        % acceleration                       a_curr
    xi_v    = 4;        % velocity incentive slack variable  xi_v
    xi_h    = 5;        % headway slack variable             xi_h
    xi_s    = 6;        % safety-critical feasibility slack  xi_s
    xi_f    = 7;        % feasibility slack variable         xi_f
end

cstrInd = 1;
for kk = 0:N-1

    % k is the index for the estimates (1 to N), kk is the true time step index (0 to N-1)
    k = kk+1;

    % Get the time step at time k
    T = Tvec(k);

    % offset to simplify notation
    o = kk*n_x_u;

    %% === Objective function ===

    % objective function: fuel consumption

    % % Fitting of ICE MAP : FC(k) = k00 + k10*wICE(k) + k01*TICE(k)
    % ii = o+[vcurr,acurr];
    % H(ii,ii) = H(ii,ii) + w_FC * [ 2*V.k01*V.F2*V.R_w/V.tau_fd/tau_est(k)/V.eta_drive ,  0;
    %                                0                                                  ,  0];   % Quadratic term
    % c(ii) = c(ii) +  w_FC * [ V.k10/V.R_w*V.tau_fd*tau_est(k)              ; 
    %                           V.k01*V.lambda*V.m*V.R_w/V.tau_fd/tau_est(k)/V.eta_drive ];      % Linear term

    % Fitting of EFFICIENCY MAP : FC(k) = p00 + p10*V(k) + p01*Twheel(k)
    ii = o+[vcurr,acurr];
    H(ii,ii) = H(ii,ii) + w_FC * [ 2*V.p01*V.F2,  0;
                                   0           ,  0];   % Quadratic term
    c(ii) = c(ii) +  w_FC * [ V.p10              ; 
                              V.p01*V.lambda*V.m ];      % Linear term

    % objective function: acceleration term
    ii = o+acurr;
    H(ii,ii) = H(ii,ii) + 2*w_a;
    
    % objective function: jerk term
    if kk == 0
        ii = o+acurr;
        H(ii,ii) = H(ii,ii) + 2*w_j/T^2;
        c(ii) = c(ii) - 2*w_j/T*a_minus1;
    else
        ii = o + [acurr,aprev];
        H(ii,ii) = H(ii,ii) + 2*w_j/T^2*[1, -1; -1, 1];
    end

    % objective function: slack variables
    c(o+xi_v) = c(o+xi_v) + w_v;
    c(o+xi_h) = c(o+xi_h) + 1e2*w_h;
    H(o+xi_h,o+xi_h) = H(o+xi_h,o+xi_h) + 2*w_h;
    c(o+xi_s) = c(o+xi_s) + w_s;
    c(o+xi_f) = c(o+xi_f) + w_f;   

    %% === Constraints ===

    if solverToUse == 2
    % HPIPM

        % continuity of the state variables
        ii = o+[scurr,vcurr,acurr,n_x_u+[scurr,vcurr,aprev]];
        G(cstrInd,ii) = [1, T, .5*T^2, -1,  0,  0]; % continuity of distance
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;
        G(cstrInd,ii) = [0,  1,  T,  0, -1,  0]; % continuity of velocity
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;
        G(cstrInd,ii) = [0,  0,   1,  0,  0, -1]; % previous acceleration
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;

        % set initial states
        if kk == 0
            G(cstrInd,o+scurr) = 1; % distance
            g_lb(cstrInd) = s_0;
            g_ub(cstrInd) = s_0;
                cstrInd = cstrInd + 1;
            G(cstrInd,o+vcurr) = 1; % velocity
            g_lb(cstrInd) = v_0;
            g_ub(cstrInd) = v_0;
                cstrInd = cstrInd + 1;
            G(cstrInd,o+aprev) = 1; % acceleration (prev)
            g_lb(cstrInd) = a_minus1;
            g_ub(cstrInd) = a_minus1;
                cstrInd = cstrInd + 1;
        end
    else
    % qpOASES

        if solverToUse == 0 % sparse qpOASES: add dynamics

            % continuity of the state variables
            ii = o+[scurr,vcurr,acurr,n_x_u+[scurr,vcurr]];
            G(cstrInd,ii) = [1, T,  T^2, -1,  0]; % continuity of distance
            g_lb(cstrInd) = 0;
            g_ub(cstrInd) = 0;
                cstrInd = cstrInd + 1;
            G(cstrInd,ii) = [0,  1, T,  0, -1]; % continuity of velocity
            g_lb(cstrInd) = 0;
            g_ub(cstrInd) = 0;
                cstrInd = cstrInd + 1;
                
            % set initial states (s and v)
            if kk == 0
                G(cstrInd,o+scurr) = 1; % distance
                g_lb(cstrInd) = s_0;
                g_ub(cstrInd) = s_0;
                    cstrInd = cstrInd + 1;
                G(cstrInd,o+vcurr) = 1; % velocity
                g_lb(cstrInd) = v_0;
                g_ub(cstrInd) = v_0;
                    cstrInd = cstrInd + 1;
            end

        end

        % Minimum and maximum bounds on states and controls
            % acceleration is limited by ISO acceleration constraint
        G(cstrInd,o+scurr) = 1;         % s
        g_lb(cstrInd) = s_min;
        g_ub(cstrInd) = s_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+vcurr) = 1;         % v
        g_lb(cstrInd) = v_min;
        g_ub(cstrInd) = v_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+xi_v) = 1;          % xi_v
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+xi_h) = 1;          % xi_h
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+xi_s) = 1;          % xi_s
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+xi_f) = 1;          % xi_f
        g_lb(cstrInd) = slack_min;
        g_ub(cstrInd) = slack_max;
            cstrInd = cstrInd + 1;
    end

    % Move blocking
    if Mb(k) == 1
        % block the move: previous acceleration must be equal to current acceleration
        G(cstrInd,o+[aprev,acurr]) = [1, -1];
        g_lb(cstrInd) = 0;
        g_ub(cstrInd) = 0;
            cstrInd = cstrInd + 1;
    end

    % ISO acceleration limits
    G(cstrInd,o+[acurr,xi_f]) = [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = a_max_est(k);
        cstrInd = cstrInd + 1;
    G(cstrInd,o+[acurr,xi_f]) = [1,1];
    g_lb(cstrInd) = a_min_est(k);
    g_ub(cstrInd) = inf;
        cstrInd = cstrInd + 1;

    % ISO jerk limits
    if kk > 0 || solverToUse == 2
        % HPIPM or k > 0
        G(cstrInd,o+[aprev,acurr,xi_f]) = [-1,1,-1];
        g_lb(cstrInd) = -inf;
        g_ub(cstrInd) = T*j_max_est(k);
            cstrInd = cstrInd + 1;
        G(cstrInd,o+[aprev,acurr,xi_f]) = [-1,1,1];
        g_lb(cstrInd) = T*j_min_est(k);
        g_ub(cstrInd) = inf;
            cstrInd = cstrInd + 1;
    else
        % qpOASES and k == 0
        G(cstrInd,o+[acurr,xi_f]) = [1,-1];
        g_lb(cstrInd) = -inf;
        g_ub(cstrInd) = T*j_max_est(k)+a_minus1;
            cstrInd = cstrInd + 1;
        G(cstrInd,o+[acurr,xi_f]) = [1,1];
        g_lb(cstrInd) = T*j_min_est(k)+a_minus1;
        g_ub(cstrInd) = inf;
            cstrInd = cstrInd + 1;
    end
    
    % % Speed limit
    % G(cstrInd,o+[vcurr,xi_f]) = [1,-1];
    % g_lb(cstrInd) = -inf;
    % g_ub(cstrInd) =  v_lim_max(k);
    %     cstrInd = cstrInd + 1;
    % 
    % % max velocity for safe/comfortable curve negotitation
    % G(cstrInd,o+[vcurr,xi_f]) = [1,-1];
    % g_lb(cstrInd) = -inf;
    % g_ub(cstrInd) =  v_curv_max(k);
    %     cstrInd = cstrInd + 1;
    % 
    % % max velocity to simulate a stop constraint
    % G(cstrInd,o+[vcurr,xi_s]) = [1,-1];
    % g_lb(cstrInd) = -inf;
    % g_ub(cstrInd) =  v_stop_max(k);
    %     cstrInd = cstrInd + 1;
    % 
    % % max velocity to simulate a traffic light constraint
    % G(cstrInd,o+[vcurr,xi_s]) = [1,-1];
    % g_lb(cstrInd) = -inf;
    % g_ub(cstrInd) =  v_TL_max(k);
    %     cstrInd = cstrInd + 1;
    
    % minimum velocity incentive
    G(cstrInd,o+[vcurr,xi_v]) = [1,1];
    g_lb(cstrInd) = v_minIncentive(k);
    g_ub(cstrInd) = inf;
        cstrInd = cstrInd + 1;

    % safe headway distance
    G(cstrInd,o+[scurr,xi_s])= [1,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = s_tv_est(k) - h_min;
        cstrInd = cstrInd + 1;
    G(cstrInd,o+[scurr,vcurr,xi_s])= [1,tau_min,-1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = s_tv_est(k);
        cstrInd = cstrInd + 1;

    % desired headway distance policy
    T_hwp = 2;
    A_hwp = 2;
    G_hwp = -0.0246*T_hwp + 0.010819;
    G(cstrInd,o+[scurr,vcurr,xi_h])= [1, T_hwp + G_hwp*v_est(k), -1];
    g_lb(cstrInd) = -inf;
    g_ub(cstrInd) = s_tv_est(k) - A_hwp;
        cstrInd = cstrInd + 1;

end

% Final stage:
T = Tvec(end);
o = N*n_x_u;

% safe headway distance
G(cstrInd,o+scurr)= 1;
g_lb(cstrInd) = -inf;
g_ub(cstrInd) = s_tv_est(N) - h_min;
    cstrInd = cstrInd + 1;
G(cstrInd,o+[scurr,vcurr])= [1, tau_min];
g_lb(cstrInd) = -inf;
g_ub(cstrInd) = s_tv_est(N);
    cstrInd = cstrInd + 1;


    function V = SetParametersInternally()
    % SetVehicleParameters loads the vehicle parameters
    %
    %   Inputs:
    %       -
    %
    %   Outputs:
    %       V : struct containg all vehicle parameters
    
        % === VEHICLE - Vehicle Depentent Parameters ===
            % Vehicle body - DAILY
        V.m           = 2620;                       % vehicle mass                                	kg
        V.A_f         = 4.614;                      % Frontal area                                  m^2
        V.c_d         = 0.46;                       % Aerodynamic drag coefficient                  -
        V.L           = 3.52;                       % Wheel base length                             m
        V.h_g         = 1.2;                        % Height of the center of gravity               m
        V.WD_s_F      = 0.45;                       % Static weight distribution on  front axle     -
        V.L_f         = 1.584;
        V.L_r         = 1.936;
        % V.L_f         = V.WD_s_F*V.L;               % Distance between the CoG the front axle       m
        % V.L_r         = V.L - V.L_f;                % Static weight distribution on  front axle     m
        
            % Vehicle body - BMW I3
        % V.m           = 1443;                       % vehicle mass                                	kg
        % V.A_f         = 2.38;                       % Frontal area                                  m^2
        % V.c_d         = 0.29;                       % Aerodynamic drag coefficient                  -
        % V.L           = 2.57;                       % Wheel base length                             m
        % V.h_g         = 0.47;                       % Height of the center of gravity               m
        % V.WD_s_F      = 0.53;                       % Static weight distribution on  front axle     -
        % V.L_f         = V.WD_s_F*V.L;               % Distance between the CoG the front axle       m
        % V.L_r         = V.L - V.L_f;                % Static weight distribution on  front axle     m
    
            % Coast Down - DAILY Only
        V.F0         = 275;                         % F0 Coast Down                                 N
        V.F1         = 0;                           % F1 Coast Down                                 N/(m/s)
        V.F2         = 1.305072;                    % F2 Coast Down                                 N/(m/s)^2
    
            % ICE Fuel Consumption Map Fitting (Poly11) - DAILY
            % val(w_ICE,T_ICE) = p00 + p10*w_ICE + p01*T_ICE
        % F1C Coefficients (with 95% confidence bounds):
        % V.p00        =      -2.528;                 % Coefficient p00                               (g/s)
        % V.p10        =     0.01173;                 % Coefficient p10 multiplied by w_ICE           (g/s)/(rad/s)
        % V.p01        =     0.01325;                 % Coefficient p01 multiplied by T_ICE           (g/s)/(Nm)
        % F1A Coefficients (with 95% confidence bounds):
        % UNITS: FC[g/s] = Poly11 ( w_ICE[rad/s] , T_ICE[Nm] )
        V.k00 =      -2.134;  % (-2.457, -1.811)    % Coefficient p00                               (g/s)
        V.k10 =     0.01164;  % (0.01059, 0.01269)  % Coefficient p10 multiplied by w_ICE           (g/s)/(rad/s)
        V.k01 =     0.01041;  % (0.009481, 0.01134) % Coefficient p01 multiplied by T_ICE           (g/s)/(Nm)
    
            % Vehicle Efficiency Map Fitting (Poly23) - DAILY
            % val(v_act,T_wheel) = p00 + p10*v_act + p01*T_wheel + p20*v_act^2 + p11*v_act*T_wheel +  
            %                      p02*y^2 + p21*v_act^2*T_wheel + p12*v_act*T_wheel^2 + p03*T_wheel^3
        % FC in [g/s] Coefficients (with 95% confidence bounds):
        % V.p00 =      0.1709; % (0.1635, 0.1783)
        % V.p10 =    0.003006; % (0.002724, 0.003288)
        % V.p01 =   2.821e-05; % (1.381e-05, 4.261e-05)
        % V.p20 =  -1.681e-05; % (-1.921e-05, -1.442e-05)
        % V.p11 =   3.994e-05; % (3.948e-05, 4.04e-05)
        % V.p02 =   1.288e-07; % (1.209e-07, 1.366e-07)
        % V.p21 =   4.652e-08; % (4.342e-08, 4.961e-08)
        % V.p12 =   5.416e-10; % (4.217e-10, 6.615e-10)
        % V.p03 =  -1.764e-11; % (-1.899e-11, -1.629e-11)
        % FC in [g/s]  Coefficients (with 95% confidence bounds):
        % V.p00 =      0.6154; % (0.5887, 0.642)
        % V.p10 =     0.01082; % (0.009806, 0.01184)
        % V.p01 =   0.0001016; % (4.973e-05, 0.0001534)
        % V.p20 =  -6.053e-05; % (-6.916e-05, -5.19e-05)
        % V.p11 =   0.0001438; % (0.0001421, 0.0001454)
        % V.p02 =   4.636e-07; % (4.353e-07, 4.919e-07)
        % V.p21 =   1.675e-07; % (1.563e-07, 1.786e-07)
        % V.p12 =    1.95e-09; % (1.518e-09, 2.382e-09)
        % V.p03 =   -6.35e-11; % (-6.835e-11, -5.865e-11)
    
            % Vehicle Efficiency Map Fitting (Poly11) - DAILY
            % val(v_act,T_wheel) = p00 + p10*v_act + p01*T_wheel
        % FC in [kg/h]  Coefficients (with 95% confidence bounds):
        % V.p00 =        -4.239; % (-4.268, -4.21)
        % V.p10 =        0.1154; % (0.115, 0.1158)
        % V.p01 =      0.006351; % (0.006333, 0.006369)
        % FC in [g/s]  Coefficients (with 95% confidence bounds):
        % UNITS: FC[g/s] = Poly11 ( V[m/s] , TW[Nm] )
        V.p00 =        -1.178; % (-1.185, -1.17)
        V.p10 =        0.1154; % (0.115, 0.1158)
        V.p01 =      0.001764; % (0.001759, 0.001769)
    
            % Powertrain
        V.P_m_max     = 125   *1e3;                 % Peak motor power                              kW
        V.T_m_max     = 250;                        % Maximum motor torque                          Nm
        V.omega_m_r   = 4800  /60*2*pi;             % Rated motor speed                             rad/s
        V.omega_m_max = 11400 /60*2*pi;             % Maximum motor speed                           rad/s
    
            % Transmission - DAILY
        V.c_r         = 0.0107;                     % Rolling resistance coefficient                -
        V.R_w         = 0.361;                      % Wheel radius                                  m
        V.beta_gb     = 9.665;                      % Gearbox ratio                                 -
        V.beta_fd     = 1;                          % Final drive gearbox ratio (estimated)         -
        % V.phi         = V.beta_gb*V.beta_fd/V.R_w;  % transmission ratio (motor to wheel)           -
        V.phi         = 26.7729;
        % Added values for Daily
        V.upSpd       = [15,25,30,40,55,70,85]./(3.6*1.05);% Upshifting Threshold                          m/s
        V.downSpd     = [5,25,20,30,40,50,60]./3.6; % Downshifting Threshold                        m/s
        V.tau_gb      = [4.714, 3.314, 2.106, 1.667, 1.285, 1, 0.839, 0.667]; % Gearbox ratio DAILY -
        V.tau_fd      = 3.615;                      % Final drive ratio DAILY                       -
        V.eta_drive   = 0.94 * 0.94;                % Driveline Efficiency (Bearbox + FD)           -
            % Transmission - BMW I3
        % V.c_r         = 0.0064;                     % Rolling resistance coefficient                -
        % V.R_w         = 0.3498;                     % Wheel radius                                  m
        % V.beta_gb     = 9.665;                      % Gearbox ratio                                 -
        % V.beta_fd     = 1;                          % Final drive gearbox ratio (estimated)         -
        % V.phi         = V.beta_gb*V.beta_fd/V.R_w;  % transmission ratio (motor to wheel)           -
        
            % Battery
        V.U_N         = 353;                        % Nominal battery pack voltage                  V
        V.Q_N         = 94;                         % Nominal battery pack capacity                 V
        V.E_b_gross   = 33.2  *3.6e6;               % Gross battery pack energy content             J
        V.E_b_net     = 27.2  *3.6e6;               % Net battery pack energy content               J
    
            % Performance
        V.v_max       = 150   /3.6;                 % Top speed                                     m/s
        V.t_acc       = 7.3;                        % Acceleration time (0-100 m/s)                 s
        V.E_v         = 13.1  *3.6e6/1e5;           % Energy consumption                            J/m
        V.D_r         = 300   *1e3;                 % Driving range                                	m
    
            % Efficiency coefficients
        V.eta_i       = 0.95;                       % Inverter efficiency                           -
        V.eta_gb      = 0.985;                      % Gearbox efficiency                            -
        V.eta_fd      = 0.93;                       % Final drive efficiency                        -
        % V.eta_TF      = V.eta_gb*V.eta_fd;          % transmission efficiency (motor to wheel)      -
        V.eta_TF      = 0.9161;

        % === OTHER - Vehicle Independent Parameters ===
        V.lambda      = 1.05;                       % Rotatial inertia inclusion factor             -
        V.P_aux       = 250;                        % Auxilary power usage                          W
        V.mu          = 0.8;                        % 'average' friction coefficient (tire/road)    -
        V.rho_a       = 1.225;                      % Air density (at 15 degrees celsius)           kg/m^3
        V.g           = 9.81;                       % Gravitational acceleration                    m/s^2
        % V.zeta_a      = .5*V.c_d*V.rho_a*V.A_f;     % Abbreviation for the constant air             kg/m
                                                        % resistance force terms
        V.zeta_a      = 1.3;
    end

    
    function [slope_est, v_lim_max,v_stop_max,v_TL_max,v_curv_max,a_min_est,a_max_est,j_min_est,j_max_est,tau_est] = EstimateBoundsInternally(OPTsettings,MPCtype,s_est,v_est,t_0,N_hor)
    % EstimateRouteAndComfortBounds Estimates travel distance and velocity dependent constraint bounds based on estimates of the trajectory of the ego vehicle
    %
    %   Inputs:
    %       OPTsettings : struct with settings for the optimization
    %       MPCtype     : whether it is a FBMPC/ABMPC (0), BLMPC (1) or TVMPC (2)
    %       s_est       : estimated travel distance over the horizon
    %       v_est       : estimated velocity over the horizon
    %       t_0         : initial time for the QP
    %       N_hor       : number of steps in the horizon
    %
    %   Outputs:
    %       slope_est  : estimated slopes over the horizon
    %       v_lim_max  : estimated speed limits over the horizon
    %       v_stop_max : estimated stop forcing speeds over the horizon
    %       v_TL_max   : estimated traffic light stop forcing speeds over the horizon
    %       v_curv_max : estimated comfortable curve speeeds over the horizon
    %       a_min_est  : estimated minimum acceleration over the horizon
    %       a_max_est  : estimated maximum acceleration over the horizon
    %       j_min_est  : estimated minimum jerk over the horizon
    %       j_max_est  : estimated maximum jerk over the horizon
    
    %% Extract settings
    
        s_speedLim          = OPTsettings.s_speedLim;
        v_speedLim          = OPTsettings.v_speedLim;
        s_curv              = OPTsettings.s_curv;
        curvature           = OPTsettings.curvature;
        s_slope             = OPTsettings.s_slope;
        slope               = OPTsettings.slope;
        stopLoc             = OPTsettings.stopLoc;
        stopRefDist         = OPTsettings.stopRefDist;
        stopRefVelSlope     = OPTsettings.stopRefVelSlope;
        stopVel             = OPTsettings.stopVel;
        TLLoc               = OPTsettings.TLLoc;
        TLstopVel           = OPTsettings.TLstopVel;
        TLStopRegionSize    = OPTsettings.TLStopRegionSize;
        alpha_TTL           = OPTsettings.alpha_TTL;
        Tvec                = OPTsettings.Tvec;
        N_hor               = 20;
    
        if MPCtype == 1
            % get BLMPC comfort limits
            BL_a_LimLowVel  = OPTsettings.BL_a_LimLowVel;    
            BL_a_LimHighVel = OPTsettings.BL_a_LimHighVel;  
            BL_j_LimLowVel  = OPTsettings.BL_j_LimLowVel;
            BL_j_LimHighVel = OPTsettings.BL_j_LimHighVel;  
        elseif MPCtype == 2
            % get TVMPC comfort limits
            TV_a_LimLowVel  = OPTsettings.TV_a_LimLowVel;    
            TV_a_LimHighVel = OPTsettings.TV_a_LimHighVel;  
            TV_j_LimLowVel  = OPTsettings.TV_j_LimLowVel;
            TV_j_LimHighVel = OPTsettings.TV_j_LimHighVel;  
        end
    
        % if length(Tvec) < N_hor
        %     % target vehicle MPC is used
        %     Tvec = Tvec(1)*ones(1,N_hor);
        % end
    
    %% Estimate the current gear based on estimate of vehcile speed
    
        % gear
        tau_est = zeros(N_hor,1);
        for i = 1:N_hor
            [tau_est(i),~] = LUTgearInternally(v_est(i));
        end
    
    
    %% Estimate route based parameters
    
        % slope
        slope_est = zeros(N_hor,1);
        for i = 1:N_hor
            for j = 1:length(s_slope)
                if j == length(s_slope)
                    slope_est(i) = slope(end);
                    break;
                elseif s_est(i) >= s_slope(j) && s_est(i) < s_slope(j+1)
                    slope_est(i) = slope(j);
                    break;
                else
                    slope_est(i) = slope(1);
                    break;
                end
            end
        end
    
        % speed limit: get maximum velocity estimates
        v_lim_max = zeros(N_hor,1);
        for i = 1:N_hor
            for j = 1:length(s_speedLim)
                if j == length(s_speedLim)
                    v_lim_max(i) = s_speedLim(end);
                elseif s_est(i) >= s_speedLim(j) && s_est(i) < s_speedLim(j+1)
                    v_lim_max(i) = v_speedLim(j);
                    break;
                end
            end
        end
    
        % curve: get maximum velocity for comfortable curve negotiation  
        v_curv_max = zeros(N_hor,1);
        for i = 1:N_hor
            for j = 1:length(s_curv)
                if j == length(s_curv)
                    v_curv_max(i) = alpha_TTL*(abs(curvature(end)))^(-1/3);
                elseif s_est(i) > s_curv(j) && s_est(i) < s_curv(j + 1)
                    v_curv_max(i) = alpha_TTL*(abs(curvature(j)))^(-1/3);
                    break;
                end
            end
        end
    
        % stops: get maximum velocity estimates
        v_stop_max = 1e5*ones(N_hor,1);
        for i = 1:N_hor
            for j = 1:length(stopLoc)
                distToStop_j = abs(stopLoc(j) - s_est(i));
                if distToStop_j < stopRefDist
                    v_stop_max(i) = distToStop_j*stopRefVelSlope + stopVel;
                end
            end
        end
    
        % traffic lights: get maximum velocity estimates
        v_TL_max = 1e5*ones(N_hor,1);
        % check the full horizon
        for i = 1:N_hor
            for j = 1:size(TLLoc,1)
                if mod(t_0 + i*Tvec(i) - TLLoc(j,2), sum(TLLoc(j,3:4))) <  TLLoc(j,3) % check whether the traffic light is red
                    distToTL_j = TLLoc(j) - s_est(i);
                    if abs(distToTL_j) < stopRefDist
                        if distToTL_j < 0                                       % behind the traffic light
                            v_TL_max(i) = abs(distToTL_j)*stopRefVelSlope + TLstopVel;
                        elseif abs(distToTL_j) < TLStopRegionSize               % close in front of the traffic light
                            v_TL_max(i) = TLstopVel;
                        else                                                    % far in front of the traffic light
                            v_TL_max(i) = abs(distToTL_j-stopVel)*stopRefVelSlope + TLstopVel;
                        end
                    end
                end
            end
        end
    
    %% Estimate acceleration and jerk limits
    
        % Initialize arrays
        a_min_est = zeros(N_hor,1);
        a_max_est = a_min_est;
        j_min_est = a_min_est;
        j_max_est = a_min_est;
    
        % estimate acceleration and jerk bounds
        for i = 1:N_hor
            if MPCtype == 0
                % ISO limits
                if v_est(i) < 5
                    a_min_est(i) = -5;
                    a_max_est(i) =  4;
                    j_min_est(i) = -5;
                    j_max_est(i) =  5;
                elseif v_est(i) < 20
                    a_min_est(i) =   -5.5+v_est(i)/10;
                    a_max_est(i) = 14/3-2*v_est(i)/15;
                    j_min_est(i) =  -35/6+v_est(i)/6;
                    j_max_est(i) =   35/6-v_est(i)/6;
                else
                    a_min_est(i) = -3.5;
                    a_max_est(i) =  2;
                    j_min_est(i) = -2.5;
                    j_max_est(i) =  2.5;
                end
            elseif MPCtype == 1 % baseline limits
                if v_est(i) < 5
                    a_min_est(i) = -BL_a_LimLowVel;
                    a_max_est(i) =  BL_a_LimLowVel;
                    j_min_est(i) = -BL_j_LimLowVel;
                    j_max_est(i) =  BL_j_LimLowVel;
                elseif v_est(i) < 20
                    a_min_est(i) =  -( (4*BL_a_LimLowVel-BL_a_LimHighVel)/3  + (BL_a_LimHighVel - BL_a_LimLowVel)/15*v_est(i) );
                    a_max_est(i) =     (4*BL_a_LimLowVel-BL_a_LimHighVel)/3  + (BL_a_LimHighVel - BL_a_LimLowVel)/15*v_est(i);
                    j_min_est(i) =  -( (4*BL_j_LimLowVel-BL_j_LimHighVel)/3  + (BL_j_LimHighVel - BL_j_LimLowVel)/15*v_est(i) );
                    j_max_est(i) =     (4*BL_j_LimLowVel-BL_j_LimHighVel)/3  + (BL_j_LimHighVel - BL_j_LimLowVel)/15*v_est(i);
                else
                    a_min_est(i) = -BL_a_LimHighVel;
                    a_max_est(i) =  BL_a_LimHighVel;
                    j_min_est(i) = -BL_j_LimHighVel;
                    j_max_est(i) =  BL_j_LimHighVel;
                end
            else % target vehicle limits
                if v_est(i) < 5
                    a_min_est(i) = -TV_a_LimLowVel;
                    a_max_est(i) =  TV_a_LimLowVel;
                    j_min_est(i) = -TV_j_LimLowVel;
                    j_max_est(i) =  TV_j_LimLowVel;
                elseif v_est(i) < 20
                    a_min_est(i) =  -( (4*TV_a_LimLowVel-TV_a_LimHighVel)/3  + (TV_a_LimHighVel - TV_a_LimLowVel)/15*v_est(i) );
                    a_max_est(i) =     (4*TV_a_LimLowVel-TV_a_LimHighVel)/3  + (TV_a_LimHighVel - TV_a_LimLowVel)/15*v_est(i);
                    j_min_est(i) =  -( (4*TV_j_LimLowVel-TV_j_LimHighVel)/3  + (TV_j_LimHighVel - TV_j_LimLowVel)/15*v_est(i) );
                    j_max_est(i) =     (4*TV_j_LimLowVel-TV_j_LimHighVel)/3  + (TV_j_LimHighVel - TV_j_LimLowVel)/15*v_est(i);
                else
                    a_min_est(i) = -TV_a_LimHighVel;
                    a_max_est(i) =  TV_a_LimHighVel;
                    j_min_est(i) = -TV_j_LimHighVel;
                    j_max_est(i) =  TV_j_LimHighVel;
                end
            end
        end
    
    
            
            function [tau_measured,gear_measured] = LUTgearInternally(v_measured)
            % LUTgearshift Generate a transmission ratio to implement the effect of 
            % the gear shifting process when running the plant model (DAILY ONLY).
            %
            %   Inputs:
            %       v_measured : simulated measured velocity in the current step
            %   Outputs:
            %       tau_measured : simulated measured gearbox ratio in the current step
            
                %% Initialization
                % V = SetVehicleParameters();
                V = SetParametersInternally();       % vehicle parameters
            
                %% LUT implementation
                
                % Run M RK4 integration steps
                if v_measured < V.upSpd(1,1)
                    tau_measured = V.tau_gb(1,1); % First Gear
                    gear_measured = 1;
                elseif v_measured > V.upSpd(1,1) && v_measured < V.upSpd(1,2)
                    tau_measured = V.tau_gb(1,2); % Second Gear
                    gear_measured = 2;
                elseif v_measured > V.upSpd(1,2) && v_measured < V.upSpd(1,3)
                    tau_measured = V.tau_gb(1,3); % Third Gear
                    gear_measured = 3;
                elseif v_measured > V.upSpd(1,3) && v_measured < V.upSpd(1,4)
                    tau_measured = V.tau_gb(1,4); % Forth Gear
                    gear_measured = 4;
                elseif v_measured > V.upSpd(1,4) && v_measured < V.upSpd(1,5)
                    tau_measured = V.tau_gb(1,5); % Fifth Gear
                    gear_measured = 5;
                elseif v_measured > V.upSpd(1,5) && v_measured < V.upSpd(1,6)
                    tau_measured = V.tau_gb(1,6); % Sixth Gear
                    gear_measured = 6;
                elseif v_measured > V.upSpd(1,6) && v_measured < V.upSpd(1,7)
                    tau_measured = V.tau_gb(1,7); % Seventh Gear
                    gear_measured = 7;
                elseif v_measured > V.upSpd(1,7)
                    tau_measured = V.tau_gb(1,8); % Eight Gear
                    gear_measured = 8;
                else
                    tau_measured = 0;
                    gear_measured = 0;
                end
            
                function V = SetParametersInternally()
                % SetVehicleParameters loads the vehicle parameters
                %
                %   Inputs:
                %       -
                %
                %   Outputs:
                %       V : struct containg all vehicle parameters
                
                    % === VEHICLE - Vehicle Depentent Parameters ===
                        % Vehicle body - DAILY
                    V.m           = 2620;                       % vehicle mass                                	kg
                    V.A_f         = 4.614;                      % Frontal area                                  m^2
                    V.c_d         = 0.46;                       % Aerodynamic drag coefficient                  -
                    V.L           = 3.52;                       % Wheel base length                             m
                    V.h_g         = 1.2;                        % Height of the center of gravity               m
                    V.WD_s_F      = 0.45;                       % Static weight distribution on  front axle     -
                    V.L_f         = 1.584;
                    V.L_r         = 1.936;
                    % V.L_f         = V.WD_s_F*V.L;               % Distance between the CoG the front axle       m
                    % V.L_r         = V.L - V.L_f;                % Static weight distribution on  front axle     m
                    
                        % Vehicle body - BMW I3
                    % V.m           = 1443;                       % vehicle mass                                	kg
                    % V.A_f         = 2.38;                       % Frontal area                                  m^2
                    % V.c_d         = 0.29;                       % Aerodynamic drag coefficient                  -
                    % V.L           = 2.57;                       % Wheel base length                             m
                    % V.h_g         = 0.47;                       % Height of the center of gravity               m
                    % V.WD_s_F      = 0.53;                       % Static weight distribution on  front axle     -
                    % V.L_f         = V.WD_s_F*V.L;               % Distance between the CoG the front axle       m
                    % V.L_r         = V.L - V.L_f;                % Static weight distribution on  front axle     m
                
                        % Coast Down - DAILY Only
                    V.F0         = 275;                         % F0 Coast Down                                 N
                    V.F1         = 0;                           % F1 Coast Down                                 N/(m/s)
                    V.F2         = 1.305072;                    % F2 Coast Down                                 N/(m/s)^2
                
                        % ICE Fuel Consumption Map Fitting (Poly11) - DAILY
                        % val(w_ICE,T_ICE) = p00 + p10*w_ICE + p01*T_ICE
                    % F1C Coefficients (with 95% confidence bounds):
                    % V.p00        =      -2.528;                 % Coefficient p00                               (g/s)
                    % V.p10        =     0.01173;                 % Coefficient p10 multiplied by w_ICE           (g/s)/(rad/s)
                    % V.p01        =     0.01325;                 % Coefficient p01 multiplied by T_ICE           (g/s)/(Nm)
                    % F1A Coefficients (with 95% confidence bounds):
                    % UNITS: FC[g/s] = Poly11 ( w_ICE[rad/s] , T_ICE[Nm] )
                    V.k00 =      -2.134;  % (-2.457, -1.811)    % Coefficient p00                               (g/s)
                    V.k10 =     0.01164;  % (0.01059, 0.01269)  % Coefficient p10 multiplied by w_ICE           (g/s)/(rad/s)
                    V.k01 =     0.01041;  % (0.009481, 0.01134) % Coefficient p01 multiplied by T_ICE           (g/s)/(Nm)
                
                        % Vehicle Efficiency Map Fitting (Poly23) - DAILY
                        % val(v_act,T_wheel) = p00 + p10*v_act + p01*T_wheel + p20*v_act^2 + p11*v_act*T_wheel +  
                        %                      p02*y^2 + p21*v_act^2*T_wheel + p12*v_act*T_wheel^2 + p03*T_wheel^3
                    % FC in [g/s] Coefficients (with 95% confidence bounds):
                    % V.p00 =      0.1709; % (0.1635, 0.1783)
                    % V.p10 =    0.003006; % (0.002724, 0.003288)
                    % V.p01 =   2.821e-05; % (1.381e-05, 4.261e-05)
                    % V.p20 =  -1.681e-05; % (-1.921e-05, -1.442e-05)
                    % V.p11 =   3.994e-05; % (3.948e-05, 4.04e-05)
                    % V.p02 =   1.288e-07; % (1.209e-07, 1.366e-07)
                    % V.p21 =   4.652e-08; % (4.342e-08, 4.961e-08)
                    % V.p12 =   5.416e-10; % (4.217e-10, 6.615e-10)
                    % V.p03 =  -1.764e-11; % (-1.899e-11, -1.629e-11)
                    % FC in [g/s]  Coefficients (with 95% confidence bounds):
                    % V.p00 =      0.6154; % (0.5887, 0.642)
                    % V.p10 =     0.01082; % (0.009806, 0.01184)
                    % V.p01 =   0.0001016; % (4.973e-05, 0.0001534)
                    % V.p20 =  -6.053e-05; % (-6.916e-05, -5.19e-05)
                    % V.p11 =   0.0001438; % (0.0001421, 0.0001454)
                    % V.p02 =   4.636e-07; % (4.353e-07, 4.919e-07)
                    % V.p21 =   1.675e-07; % (1.563e-07, 1.786e-07)
                    % V.p12 =    1.95e-09; % (1.518e-09, 2.382e-09)
                    % V.p03 =   -6.35e-11; % (-6.835e-11, -5.865e-11)
                
                        % Vehicle Efficiency Map Fitting (Poly11) - DAILY
                        % val(v_act,T_wheel) = p00 + p10*v_act + p01*T_wheel
                    % FC in [kg/h]  Coefficients (with 95% confidence bounds):
                    % V.p00 =        -4.239; % (-4.268, -4.21)
                    % V.p10 =        0.1154; % (0.115, 0.1158)
                    % V.p01 =      0.006351; % (0.006333, 0.006369)
                    % FC in [g/s]  Coefficients (with 95% confidence bounds):
                    % UNITS: FC[g/s] = Poly11 ( V[m/s] , TW[Nm] )
                    V.p00 =        -1.178; % (-1.185, -1.17)
                    V.p10 =        0.1154; % (0.115, 0.1158)
                    V.p01 =      0.001764; % (0.001759, 0.001769)
                
                        % Powertrain
                    V.P_m_max     = 125   *1e3;                 % Peak motor power                              kW
                    V.T_m_max     = 250;                        % Maximum motor torque                          Nm
                    V.omega_m_r   = 4800  /60*2*pi;             % Rated motor speed                             rad/s
                    V.omega_m_max = 11400 /60*2*pi;             % Maximum motor speed                           rad/s
                
                        % Transmission - DAILY
                    V.c_r         = 0.0107;                     % Rolling resistance coefficient                -
                    V.R_w         = 0.361;                      % Wheel radius                                  m
                    V.beta_gb     = 9.665;                      % Gearbox ratio                                 -
                    V.beta_fd     = 1;                          % Final drive gearbox ratio (estimated)         -
                    % V.phi         = V.beta_gb*V.beta_fd/V.R_w;  % transmission ratio (motor to wheel)           -
                    V.phi         = 26.7729;
                    % Added values for Daily
                    V.upSpd       = [15,25,30,40,55,70,85]./(3.6*1.05);% Upshifting Threshold                          m/s
                    V.downSpd     = [5,25,20,30,40,50,60]./3.6; % Downshifting Threshold                        m/s
                    V.tau_gb      = [4.714, 3.314, 2.106, 1.667, 1.285, 1, 0.839, 0.667]; % Gearbox ratio DAILY -
                    V.tau_fd      = 3.615;                      % Final drive ratio DAILY                       -
                    V.eta_drive   = 0.94 * 0.94;                % Driveline Efficiency (Bearbox + FD)           -
                        % Transmission - BMW I3
                    % V.c_r         = 0.0064;                     % Rolling resistance coefficient                -
                    % V.R_w         = 0.3498;                     % Wheel radius                                  m
                    % V.beta_gb     = 9.665;                      % Gearbox ratio                                 -
                    % V.beta_fd     = 1;                          % Final drive gearbox ratio (estimated)         -
                    % V.phi         = V.beta_gb*V.beta_fd/V.R_w;  % transmission ratio (motor to wheel)           -
                    
                        % Battery
                    V.U_N         = 353;                        % Nominal battery pack voltage                  V
                    V.Q_N         = 94;                         % Nominal battery pack capacity                 V
                    V.E_b_gross   = 33.2  *3.6e6;               % Gross battery pack energy content             J
                    V.E_b_net     = 27.2  *3.6e6;               % Net battery pack energy content               J
                
                        % Performance
                    V.v_max       = 150   /3.6;                 % Top speed                                     m/s
                    V.t_acc       = 7.3;                        % Acceleration time (0-100 m/s)                 s
                    V.E_v         = 13.1  *3.6e6/1e5;           % Energy consumption                            J/m
                    V.D_r         = 300   *1e3;                 % Driving range                                	m
                
                        % Efficiency coefficients
                    V.eta_i       = 0.95;                       % Inverter efficiency                           -
                    V.eta_gb      = 0.985;                      % Gearbox efficiency                            -
                    V.eta_fd      = 0.93;                       % Final drive efficiency                        -
                    % V.eta_TF      = V.eta_gb*V.eta_fd;          % transmission efficiency (motor to wheel)      -
                    V.eta_TF      = 0.9161;
            
                    % === OTHER - Vehicle Independent Parameters ===
                    V.lambda      = 1.05;                       % Rotatial inertia inclusion factor             -
                    V.P_aux       = 250;                        % Auxilary power usage                          W
                    V.mu          = 0.8;                        % 'average' friction coefficient (tire/road)    -
                    V.rho_a       = 1.225;                      % Air density (at 15 degrees celsius)           kg/m^3
                    V.g           = 9.81;                       % Gravitational acceleration                    m/s^2
                    % V.zeta_a      = .5*V.c_d*V.rho_a*V.A_f;     % Abbreviation for the constant air             kg/m
                                                                    % resistance force terms
                    V.zeta_a      = 1.3;
                end
    
            end
    
    end

end

