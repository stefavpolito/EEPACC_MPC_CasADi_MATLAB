function optSol = RunOpt_ABMPC(OPTsettings)
% RunOpt_ABMPC Runs the acceleration-based MPC over the simulation
%
%   Inputs:
%       OPTsettings : struct with settings for the MPC
%
%   Output:
%       optSol      : struct containing the optimal solution and other relevant data

%% Import CasADi

import casadi.*

%% Unpack settings

% optimization settings
W               = OPTsettings.W_AB;
N_hor           = OPTsettings.N_hor;
b_fifthOrder    = OPTsettings.b_fifthOrder;
solverToUse     = OPTsettings.solverToUse;
Tvec            = OPTsettings.Tvec;
Mb              = OPTsettings.Mb;
t_sim           = OPTsettings.t_sim;
s_0             = OPTsettings.s_init;
v_0             = OPTsettings.v_init;
a_minus1        = OPTsettings.a_minus1;

% vehicle following settings
s_tv = OPTsettings.s_tv;
v_tv = OPTsettings.v_tv;

% get vehicle parameters
V = SetVehicleParameters();
% V = SetParametersInternally();

% determine number of steps for the simulation
Ts    = Tvec(1);
N_sim = t_sim/Ts;

%% Initialize matrices and create solver

% numbers of variables
if solverToUse == 2
% HPIPM
    n_x = 3; % [s; v; a_prev]
else
% qpOASES
    n_x = 2; % [s; v]
end
n_u = 5; % [a; xi_v; xi_h; xi_s; xi_f]
n_x_u = n_x + n_u;

% initialization of matrices: estimated values don't matter
s_est       = ones(N_hor,1);
v_est       = ones(N_hor,1);
s_tv_est    = ones(N_hor,1);
t_0         = 1;

% Get matrices and vectors for sparsity patterns
[H,c,G,~,~,g_lb,g_ub] = CreateQP_AB(OPTsettings,n_x,n_u,s_0,v_0,s_est,v_est,s_tv_est,t_0,a_minus1);

% Delete zero rows in G
zeroColsG = all(~G,2);
G(zeroColsG,:) = [];

% Fix lengths of bounds on G
g_lb = g_lb(1:size(G,1));
g_ub = g_ub(1:size(G,1));

% Transform to dense formulation if required
if solverToUse == 1

    % Create state space matrices
    A = zeros(N_hor,n_x,n_x);
    B = zeros(N_hor,n_x,n_u);
    D = zeros(N_hor,n_x);
    for k = 1:N_hor
        A(k,1:2,1:2) = [1, Tvec(k); 0, 1];
        B(k,1,1) = .5*Tvec(k)^2;
        B(k,2,1) = Tvec(k);
        D(k,:,:) = [0; 0];
    end

    [H,~,G,~,~,~,~] = TransformToDenseFormulation(N_hor,A,B,D,H,c,G,g_lb,g_ub,n_x,n_u,s_0,v_0);
end

% create problem structure and solver
if solverToUse == 2
% HPIPM
    % # tot. constraints = {# cols G} - {# state cont. constr.} - {# init. state constr.} - {# final stage constr.} - {# move blocking constr.}
    ng_num = (size(G,1)-(N_hor+1)*n_x - 2 - sum(Mb))/N_hor;
    optsHPIPM = struct;
    optsHPIPM.N = N_hor;
    optsHPIPM.nx = n_x*ones(N_hor+1,1);
    optsHPIPM.nu = [n_u*ones(N_hor,1);0];
    ng = [ng_num+n_x;ng_num*ones(N_hor-1,1);n_x-1];
    optsHPIPM.ng = ng + [Mb,0]'; % add move blocking constraints
    optsHPIPM.hpipm.iter_max = OPTsettings.ABMPCmaxIterHPIPM;
        % other optional options
        %     optsHPIPM.hpipm.res_g_max = 1e-10;
        %     optsHPIPM.hpipm.res_b_max = 1e-10;
        %     optsHPIPM.hpipm.res_d_max = 1e-10;
        %     optsHPIPM.hpipm.res_m_max = 1e-10;
        %     optsHPIPM.hpipm.mode = "robust";
        %     optsHPIPM.print_time = true;
        %     optsHPIPM.inf = 1e7;
    optsHPIPM.print_problem = false;
    optsHPIPM.print_time = false;
    optsHPIPM.print_in = false;
    optsHPIPM.print_out = false;
    optsHPIPM.hpipm.warm_start = true;
    optsHPIPM.error_on_fail = false;
    H = sparse(H);
    G = sparse(G);
    prob = struct('h', SX(H).sparsity(), 'a', SX(G).sparsity());
    QPsolver = conic('QPsolver','hpipm',prob,optsHPIPM);
else
% qpOASES
    opts = struct;
    opts.printLevel = 'low'; % options: none, low, medium, high, debug_iter, tabular
    opts.error_on_fail = false;
    prob = struct('h', SX(H).sparsity(), 'a', SX(G).sparsity());
    QPsolver = conic('QPsolver','qpoases',prob,opts);
end

%% MPC loop

% initialize arrays
s_opt     = zeros(t_sim/Ts,1);
v_opt     = s_opt;
Fm_opt    = s_opt;
Fb_opt    = s_opt;
xi_v_opt  = s_opt;
xi_h_opt  = s_opt;
xi_s_opt  = s_opt;
xi_f_opt  = s_opt;
E_opt     = s_opt;
a_opt     = s_opt;
DistHor   = s_opt;
cost      = s_opt;
tLoop     = s_opt;
tSolve    = s_opt;
cost_a    = s_opt;
cost_j    = s_opt;
cost_xi_v = s_opt;
cost_xi_h = s_opt;
cost_xi_s = s_opt;
cost_xi_f = s_opt;

% simulation time in the first step of each MPC loop (s)
t_0  = 0;

% Loop over MPC steps
for kk = 0:N_sim 
    k = kk+1;
    tLoopTic = tic();
    %% Measurements and estimations

    if kk == 0
        % current state is initial state at t0
        s_measured = s_0;
        v_measured = v_0;
        
        % Previous solution is unknown (assumption)
        s_opt_prev_sol = zeros(N_hor+1,1);
        v_opt_prev_sol = zeros(N_hor+1,1);

        % initial target vehicle measurement
        s_tv_measured = s_tv(k);
        v_tv_measured = 0;
        v_tv_prev     = v_tv_measured;
        a_tv_prev     = (v_tv_measured - v_tv_prev)/Ts;
    else
        % Previous states and controls
        s_prev  = s_opt(k-1);
        v_prev  = v_opt(k-1);
        Fm_prev = Fm_opt(k-1);
        Fb_prev = Fb_opt(k-1);

        % Get measurements from simulated nonlinear plant (integrator using 4 RK4 steps)
        [s_measured,v_measured] = RunPlantModel([s_prev,v_prev],[Fm_prev, Fb_prev],OPTsettings);

        % Estimate of the previous acceleration
        a_minus1 = (v_measured-v_prev)/Ts;

        % Headway distance measurement and calculations
        s_tv_measured = s_tv(k);
        v_tv_prev     = v_tv_measured;
        v_tv_measured = v_tv(k);
        a_tv_prev     = (v_tv_measured - v_tv_prev)/Ts;
    end

    % estimates of the ego vehicle's trajectory
    [s_est,v_est] = EstimateVehicleTrajectory(OPTsettings,0,s_measured,v_measured,a_minus1,s_opt_prev_sol,v_opt_prev_sol);

    % Estimate the target vehicle's distances
    [s_tv_est,v_tv_est] = EstimateVehicleTrajectory(OPTsettings,1,s_tv_measured,v_tv_measured,a_tv_prev);

    % save horizon distance
    DistHor(k) = s_est(end) - s_measured;

    %% Formulate the the QP

    [H,c,G,z_lb,z_ub,g_lb,g_ub] = CreateQP_AB(OPTsettings,n_x,n_u,s_measured,v_measured,s_est,v_est,s_tv_est,t_0,a_minus1);

    %% Postprocessing

    if solverToUse == 2
        % HPIPM: change inf values in bounds of G and z to 1e7
        for i = 1:length(g_lb)
            if abs(g_lb(i)) > 1e7
                g_lb(i) = sign(g_lb(i))*1e7;
            end
            if abs(g_ub(i)) > 1e7
                g_ub(i) = sign(g_ub(i))*1e7;
            end
        end
        for i = 1:length(z_lb)
            if abs(z_lb(i)) > 1e7
                z_lb(i) = sign(z_lb(i))*1e7;
            end
            if abs(z_ub(i)) > 1e7
                z_ub(i) = sign(z_ub(i))*1e7;
            end
        end
    end

    % Delete zero rows in G
    G(zeroColsG,:) = [];
        
    % Fix lengths of bounds on G
    g_lb = g_lb(1:size(G,1));
    g_ub = g_ub(1:size(G,1));

    % Transform optimization matrices to dense versions (if necessary)
    if solverToUse == 1
        % dense qpOASES
        [H,c,G,g_lb,g_ub,Psi,d] = TransformToDenseFormulation(N_hor,A,B,D,H,c,G,g_lb,g_ub,n_x,n_u,s_measured,v_measured);
    end
    
    %% Solve MPC and extract states and controls

    % solve the problem and extract the optimal solution
    tSolveTic = tic();
    if solverToUse == 2 
    % HPIPM 
        H = sparse(H);
        G = sparse(G);
        sol = QPsolver('h',H,'g',c,'a',G,'lbx',z_lb,'ubx',z_ub,'lba',g_lb,'uba',g_ub);
    else 
    % qpOASES
        sol = QPsolver('h',H,'g',c,'a',G,'lbx',z_lb,'ubx',z_ub,'lba',g_lb,'uba',g_ub);       
    end
    tSolve(k) = toc(tSolveTic);
    optSol.exitMessage(k) = ~QPsolver.stats().success;
    optSol.solverTime(k) = QPsolver.stats().t_proc_solver;
    z_opt = full(sol.x);
    
    % Transform dense z to sparse z to get the states as well
    if solverToUse == 1
        z_opt = Psi*z_opt+d;
    end

    % extract optimal state and control variables
    s_opt(k) = z_opt(1);
    v_opt(k) = z_opt(2);
    if solverToUse == 2 
    % HPIPM
        a_opt(k)    = z_opt(4);     
        xi_v_opt(k) = z_opt(5);
        xi_h_opt(k) = z_opt(6);
        xi_s_opt(k) = z_opt(7);
        xi_f_opt(k) = z_opt(8);
    else 
    % qpOASES
        a_opt(k)    = z_opt(3);     
        xi_v_opt(k) = z_opt(4);
        xi_h_opt(k) = z_opt(5);
        xi_s_opt(k) = z_opt(6);
        xi_f_opt(k) = z_opt(7);
    end

    % save previous solution's trajectory for approximations in the next step
    s_opt_prev_sol = z_opt(1:n_x_u:end);
    v_opt_prev_sol = z_opt(2:n_x_u:end);

    %% determine required forces

    % get the estimated road angle
    theta = InterpPWA(s_measured,OPTsettings.s_slope,OPTsettings.slope);

    % Total resistance force (sum of aerodynamic, rolling and grading resistance forces)
    F_r = -V.zeta_a*v_measured^2 - V.c_r*V.m*V.g*cos(theta) - V.m*V.g*sin(theta); 

    % Required force
    F_t_req = V.m*V.lambda*a_opt(k) - F_r;

    % Calculate rear wheel traction limit and total traction limit
    F_f_r_max = V.mu/V.L*( V.m*V.g*( V.L_f*cos(theta) + V.h_g*sin(theta) ) + V.h_g*( V.zeta_a*v_measured^2 + V.lambda*V.m*a_opt(k) ) );
    F_f_tot_max = V.mu*V.m*V.g*cos(theta);

    % Correct for traction force (friction) and motor limits (torque)
    if F_t_req < 0 
        % braking
        if v_measured < V.omega_m_r/V.phi
            F_m_min = -V.phi*V.T_m_max/V.eta_TF;
        else
            F_m_min = -V.P_m_max/V.eta_TF/v_measured;
        end
        Fm_opt(k) = max([F_t_req,F_m_min,-F_f_r_max]);
        Fb_opt(k) = max(F_t_req,-F_f_tot_max) - Fm_opt(k);
    else
        % propulsion
        if v_measured < V.omega_m_r/V.phi
            F_m_max = V.phi*V.T_m_max*V.eta_TF;
        else
            F_m_max = V.P_m_max*V.eta_TF/v_measured;
        end
        Fm_opt(k) = min([F_t_req,F_m_max,F_f_r_max]);
        Fb_opt(k) = 0;
    end
    
    % Get the 'corrected' acceleration
    a_opt(k) = (Fm_opt(k) + Fb_opt(k) + F_r)/V.m/V.lambda; % realized acceleration

    % Calculate other optimal quantities
    cost(k)  = full(sol.cost);
    tLoop(k) = toc(tLoopTic);
    t_0 = t_0 + Ts;

    if OPTsettings.createGifs
        %% plot a frame for the animation
        if kk == 0
            [EVtrajPlot,EVpredPlot,TVtrajPlot,TVpredPlot] = InitiateAnimation_MPC(k,s_opt,v_opt,s_opt_prev_sol,v_opt_prev_sol,s_tv_est,v_tv_est,OPTsettings,'ABMPC');
        else
            PlotAnimFrame_MPC(k,s_opt,v_opt,s_opt_prev_sol,v_opt_prev_sol,s_tv_est,v_tv_est,EVtrajPlot,EVpredPlot,TVtrajPlot,TVpredPlot)
        end
    end

end

% Calculate other optimal quantities
rpm_opt = (30/pi)*v_opt*V.phi;
Tm_opt  = Fm_opt./V.phi./(V.eta_TF.^sign(Fm_opt));
% Add phi = Rw/tau_FD/tau_GB(v_opt)
P_opt   = GetMotorPower_FifthOrderSurface(Fm_opt,rpm_opt,b_fifthOrder);
for k = 1:length(a_opt)
    E_opt(k) = Ts*sum(P_opt(1:k));
end

%% prepare output struct

% optimal jerk
j_opt = diff(a_opt)/Ts;

% optimized variables
optSol.s_opt        = s_opt;
optSol.v_opt        = v_opt;
optSol.Fm_opt       = Fm_opt;
optSol.Fb_opt       = Fb_opt;
optSol.xi_v_opt     = xi_v_opt;
optSol.xi_h_opt     = xi_h_opt;
optSol.xi_s_opt     = xi_s_opt;
optSol.xi_f_opt     = xi_f_opt;

% derived from optimal variables
optSol.P_opt    = P_opt;
optSol.E_opt    = E_opt;
optSol.a_opt    = a_opt;
optSol.j_opt    = j_opt;
optSol.Tm_opt   = Tm_opt;
optSol.rpm_opt  = rpm_opt;

optSol.tLoop    = tLoop;
optSol.tSolve   = tSolve;

% MPC data
optSol.H        = H;
optSol.G        = G;
optSol.DistHor  = DistHor;

% extract weights
w_a  = W(1);
w_j  = W(2);
w_v  = W(3);
w_h  = W(4);
w_s  = W(5);
w_f  = W(5);

% costs over the horizon
for k = 1:N_sim
    cost_a(k)    = w_a*sum(a_opt(1:k).^2);
    cost_j(k)    = w_j*sum(j_opt(1:k).^2);
    cost_xi_v(k) = w_v*sum(xi_v_opt(1:k));
    cost_xi_h(k) = w_h*sum(xi_h_opt(1:k));
    cost_xi_s(k) = w_s*sum(xi_s_opt(1:k));
    cost_xi_f(k) = w_f*sum(xi_f_opt(1:k));
end
optSol.cost_a    = cost_a;
optSol.cost_j    = cost_j;
optSol.cost_xi_v = cost_xi_v;
optSol.cost_xi_h = cost_xi_h;
optSol.cost_xi_s = cost_xi_s;
optSol.cost_xi_f = cost_xi_f;


end