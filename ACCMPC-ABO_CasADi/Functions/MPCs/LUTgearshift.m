function [tau_measured,gear_measured] = LUTgearshift(v_measured)
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


%% Code to try the setup of the LUT on known Driving Cycle
% tau_z=zeros(length(V_z),1);
% for i=1:length(V_z)
%     v_measured = V_z(i,1)/3.6;
%     [tau_z(i,1),gear_z(i,1)] = LUTgearshift(v_measured);
% end
% plot(T_z,V_z), hold on, plot(T_z,gear_z*10), plot(T_z,G_z*10)

%% Tipical Callback
%  [tau_measured,gear_measured] = LUTgearshift(v_measured);
