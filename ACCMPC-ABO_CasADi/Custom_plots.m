clearvars;clc;close all;
addpath('.\casadi_3_6_3')
addpath(genpath('.\Functions\'))
addpath(genpath('.\DrivingCycles'))
V = SetVehicleParameters();

%% Load Results

% load('savedABMPCsolEFFMAP.mat')
% load('savedABMPCsolICEMAP.mat')
load('savedABMPCsolnoFCopt.mat')
load('savedABMPCsolFCopt_2.mat')
load('TO_Urban_LEAD.mat')

%% Optimal Values - ICEMAP

% tau_est = zeros(length(ABMPCsolICEMAP.a_opt),1);
% gear_est = tau_est;
% phi_est = tau_est;
% TICE_opt = tau_est;
% wICE_opt = tau_est;
% FC_ICE = tau_est;
% FC_tot_ICE = tau_est;
% TWICE_act = tau_est;
% FCICE_act = tau_est;
% FCtot_act = tau_est;
% 
% for i=2:length(ABMPCsolICEMAP.a_opt)
% 
%     [tau_est(i,1), gear_est(i,1)] = LUTgearshift(ABMPCsolICEMAP.v_opt(i,1));
%     phi_est(i,1) = V.R_w / V.tau_fd / tau_est(i,1) ; 
%     TICE_opt(i,1) = max( 0 , (V.lambda*V.m*ABMPCsolICEMAP.a_opt(i,1) + ...
%                               V.F0 + ...
%                               V.F2*ABMPCsolICEMAP.v_opt(i,1)*ABMPCsolICEMAP.v_opt(i,1)) * phi_est(i,1) / V.eta_drive );
%     wICE_opt(i,1) = max( 800/30*pi , ABMPCsolICEMAP.v_opt(i,1) / phi_est(i,1));
%     % UNITS: FC[g/s] = Poly11 ( w_ICE[rad/s] , T_ICE[Nm] )
%     FC_ICE(i,1) = max( 0.25, V.k00 + ...
%                              V.k10 * wICE_opt(i,1) + ... 
%                              V.k01 * TICE_opt(i,1)); % [g/s]
%     FC_tot_ICE(i,1) = FC_tot_ICE(i-1,1) + FC_ICE(i,1)/1000*0.5; % [kg]
% 
% end
% 
% FE_ICE = max(FC_tot_ICE/0.835)/max(ABMPCsolICEMAP.s_opt/1000)*100 % [L/100km]
% FE_REF = TO_Urban.FE(end,1)

%% Optimal Values - EFFMAP

% tau_est = zeros(length(ABMPCsolEFFMAP.a_opt),1);
% TW_opt = tau_est;
% FC_EFF = tau_est;
% FC_tot_EFF = tau_est;
% 
% for i=2:length(ABMPCsolEFFMAP.a_opt)
% 
%     TW_opt(i,1) = max( 0 , (V.lambda*V.m*ABMPCsolEFFMAP.a_opt(i,1) + ...
%                             V.F0 + ...
%                             V.F2*ABMPCsolEFFMAP.v_opt(i,1)*ABMPCsolEFFMAP.v_opt(i,1)) * V.R_w );
%     % UNITS: FC[g/s] = Poly11 ( V[m/s] , TW[Nm] )
%     FC_EFF(i,1) = max( 0.25 , V.p00 + ...
%                               V.p10 * ABMPCsolEFFMAP.v_opt(i,1) + ...
%                               V.p01 * TW_opt(i,1)); % [g/s]
%     FC_tot_EFF(i,1) = FC_tot_EFF(i-1,1) + FC_EFF(i,1)/1000*0.5; % [kg]
% 
% end
% 
% FE_EFF = max(FC_tot_EFF/0.835)/max(ABMPCsolEFFMAP.s_opt/1000)*100 % [L/100km]
% FE_ref = TO_Urban.FE(end,1)

%% Optimal Values - with EFFMAP Optimization

tau_est = zeros(length(ABMPCsolFCopt.a_opt),1);
TW_opt = tau_est;
FC_EFF = tau_est;
FC_tot_EFF = tau_est;

for i=2:length(ABMPCsolFCopt.a_opt)

    TW_opt(i,1) = max( 0 , (V.lambda*V.m*ABMPCsolFCopt.a_opt(i,1) + ...
                            V.F0 + ...
                            V.F2*ABMPCsolFCopt.v_opt(i,1)*ABMPCsolFCopt.v_opt(i,1)) * V.R_w );
    % UNITS: FC[g/s] = Poly11 ( V[m/s] , TW[Nm] )
    FC_EFF(i,1) = max( 0.25 , V.p00 + ...
                              V.p10 * ABMPCsolFCopt.v_opt(i,1) + ...
                              V.p01 * TW_opt(i,1)); % [g/s]
    FC_tot_EFF(i,1) = FC_tot_EFF(i-1,1) + FC_EFF(i,1)/1000*0.5; % [kg]

end

FE_EFF = max(FC_tot_EFF/0.835)/max(ABMPCsolFCopt.s_opt/1000)*100 % [L/100km]
FE_ref = TO_Urban.FE(end,1)

%% Optimal Values - without EFFMAP Optimization

tau_est = zeros(length(ABMPCsolnoFCopt.a_opt),1);
TW_opt = tau_est;
FC_noEFF = tau_est;
FC_tot_noEFF = tau_est;

for i=2:length(ABMPCsolnoFCopt.a_opt)

    TW_opt(i,1) = max( 0 , (V.lambda*V.m*ABMPCsolnoFCopt.a_opt(i,1) + ...
                            V.F0 + ...
                            V.F2*ABMPCsolnoFCopt.v_opt(i,1)*ABMPCsolnoFCopt.v_opt(i,1)) * V.R_w );
    % UNITS: FC[g/s] = Poly11 ( V[m/s] , TW[Nm] )
    FC_noEFF(i,1) = max( 0.25 , V.p00 + ...
                                V.p10 * ABMPCsolnoFCopt.v_opt(i,1) + ...
                                V.p01 * TW_opt(i,1)); % [g/s]
    FC_tot_noEFF(i,1) = FC_tot_noEFF(i-1,1) + FC_noEFF(i,1)/1000*0.5; % [kg]

end

FE_noEFF = max(FC_tot_noEFF/0.835)/max(ABMPCsolnoFCopt.s_opt/1000)*100 % [L/100km]
FE_ref = TO_Urban.FE(end,1)


%% Baseline

load("DrivingCycles\TO01_EAD.mat")
V_TO=resample(V_TO,1,5);
V_TO(871,1) = 0;
tau_lead = zeros(length(V_TO),1);
gear_lead = tau_lead;
phi_lead = tau_lead;
TICE_lead = tau_lead;
wICE_lead = tau_lead;
TW_lead = tau_lead;
FC_lead = tau_lead;
FC_tot_lead = tau_lead;
a_TO = tau_lead;
s_TO = tau_lead;

for i=2:length(V_TO)
    
    [tau_lead(i,1), gear_lead(i,1)] = LUTgearshift(V_TO(i,1));
    a_TO(i,1) = (V_TO(i,1) - V_TO(i-1,1)) / 0.5 ;
    phi_lead(i,1) = V.R_w / V.tau_fd / tau_lead(i,1) ; 
    TICE_lead(i,1) = max( 0 , (V.lambda*V.m*a_TO(i,1) + ...
                              V.F0 + ...
                              V.F2*V_TO(i,1)*V_TO(i,1)) * phi_lead(i,1) / V.eta_drive );
    wICE_lead(i,1) = max( 800/30*pi , V_TO(i,1) / phi_lead(i,1));
    TW_lead(i,1) = max( 0 , (V.lambda*V.m*a_TO(i,1) + ...
                            V.F0 + ...
                            V.F2*V_TO(i,1)*V_TO(i,1)) * V.R_w );
    % UNITS: FC[g/s] = Poly11 ( V[m/s] , TW[Nm] )
    FC_lead(i,1) = max( 0.25 , V.p00 + ...
                               V.p10 * V_TO(i,1) + ...
                               V.p01 * TW_lead(i,1)); % [g/s]
    FC_tot_lead(i,1) = FC_tot_lead(i-1,1) + FC_lead(i,1)/1000*0.5; % [kg]
    s_TO(i,1) = s_TO(i-1,1) + V_TO(i,1) * 0.5;

end

FE_lead = max(FC_tot_lead/0.835)/max(s_TO/1000)*100 % [L/100km]
FE_ref = TO_Urban.FE(end,1)

%% Plots of ICE Map Version

% time = linspace(0,(length(ABMPCsolICEMAP.a_opt)-1)/2,length(ABMPCsolICEMAP.a_opt))';
% 
% figure()
% subplot(3,1,1)
% hold on
% title('Vehicle Speed [m/s]')
% plot(time,ABMPCsolICEMAP.v_opt)
% plot(time,V_TO)
% legend('opt','lead')
% subplot(3,1,2)
% hold on
% title('Travelled Distance [m]')
% plot(time,ABMPCsolICEMAP.s_opt)
% plot(time,s_TO)
% legend('opt','lead')
% subplot(3,1,3)
% hold on
% title('Longi Acceleration [m/s2]')
% plot(time,ABMPCsolICEMAP.a_opt)
% plot(time,a_TO)
% legend('opt','lead')
% 
% figure()
% subplot(3,1,1)
% hold on
% plot(time,gear_est)
% plot(time,gear_lead)
% plot(TO_Urban.TO_time,TO_Urban.gear)
% legend('opt','lead','model')
% subplot(3,1,2)
% hold on
% plot(time,tau_est)
% plot(time,tau_lead)
% legend('opt','lead')
% subplot(3,1,3)
% hold on
% plot(time,phi_est)
% plot(time,phi_lead)
% legend('opt','lead')
% 
% figure()
% subplot(3,1,1)
% hold on
% title('ICE Torque [Nm]')
% plot(time,TICE_opt)
% plot(time,TICE_lead)
% plot(TO_Urban.TO_time,TO_Urban.T_ICE)
% legend('opt','lead','model')
% subplot(3,1,2)
% hold on
% title('ICE Speed [rpm]')
% plot(time,wICE_opt*30/pi)
% plot(time,wICE_lead*30/pi)
% plot(TO_Urban.TO_time,TO_Urban.w_ICE)
% legend('opt','lead','model')
% subplot(3,1,3)
% hold on
% title('Fuel Rate [g/s]')
% plot(time,FC_ICE)
% plot(time,FC_lead)
% legend('opt','lead')
% 
% figure()
% hold on
% title('Total Fuel Consumption [L]')
% plot(time,FC_tot_ICE)
% plot(time,FC_tot_lead)
% plot(TO_Urban.TO_time,TO_Urban.FC)
% legend('opt','lead','model')

%% Comparison between Cost Functions

% time = linspace(0,(length(ABMPCsol.a_opt)-1)/2,length(ABMPCsol.a_opt))';
% 
% figure()
% subplot(3,1,1)
% hold on
% title('Vehicle Speed [m/s]')
% plot(time,V_TO)
% plot(time,ABMPCsolICEMAP.v_opt)
% plot(time,ABMPCsol.v_opt)
% legend('lead','ICEMAP','EFFMAP')
% subplot(3,1,2)
% hold on
% title('Travelled Distance [m]')
% plot(time,s_TO)
% plot(time,ABMPCsolICEMAP.s_opt)
% plot(time,ABMPCsol.s_opt)
% legend('lead','ICEMAP','EFFMAP')
% subplot(3,1,3)
% hold on
% title('Longi Acceleration [m/s2]')
% plot(time,a_TO)
% plot(time,ABMPCsolICEMAP.a_opt)
% plot(time,ABMPCsol.a_opt)
% legend('lead','ICEMAP','EFFMAP')
% 
% % figure()
% % subplot(3,1,1)
% % hold on
% % plot(time,gear_est)
% % plot(time,gear_lead)
% % legend('opt','lead')
% % subplot(3,1,2)
% % hold on
% % plot(time,tau_est)
% % plot(time,tau_lead)
% % legend('opt','lead')
% % subplot(3,1,3)
% % hold on
% % plot(time,phi_est)
% % plot(time,phi_lead)
% % legend('opt','lead')
% 
% figure()
% hold on
% title('Fuel Rate [g/s]')
% plot(time,FC_lead)
% plot(time,FC_ICE)
% plot(time,FC_EFF)
% legend('lead','ICEMAP','EFFMAP')
% 
% figure()
% hold on
% title('Total Fuel Consumption [L]')
% plot(time,FC_tot_lead)
% plot(time,FC_tot_ICE)
% plot(time,FC_tot_EFF)
% plot(TO_Urban.TO_time,TO_Urban.FC)
% legend('lead','ICEMAP','EFFMAP','model')

%% Comparison FC with/without EFFMAP Optimization

time = linspace(0,(length(ABMPCsolFCopt.a_opt)-1)/2,length(ABMPCsolFCopt.a_opt))';

figure()
subplot(3,1,1)
hold on
title('Vehicle Speed [m/s]')
plot(time,V_TO)
plot(time,ABMPCsolFCopt.v_opt)
plot(time,ABMPCsolnoFCopt.v_opt)
legend('lead','FCopt','noFCopt')
subplot(3,1,2)
hold on
title('Travelled Distance [m]')
plot(time,s_TO)
plot(time,ABMPCsolFCopt.s_opt)
plot(time,ABMPCsolnoFCopt.s_opt)
legend('lead','FCopt','noFCopt')
subplot(3,1,3)
hold on
title('Longi Acceleration [m/s2]')
plot(time,a_TO)
plot(time,ABMPCsolFCopt.a_opt)
plot(time,ABMPCsolnoFCopt.a_opt)
legend('lead','FCopt','noFCopt')

figure()
hold on
title('Fuel Rate [g/s]')
plot(time,FC_lead)
plot(time,FC_EFF)
plot(time,FC_noEFF)
legend('lead','FCopt','noFCopt')

figure()
hold on
title('Total Fuel Consumption [L]')
plot(time,FC_tot_lead)
plot(time,FC_tot_EFF)
plot(time,FC_tot_noEFF)
% plot(TO_Urban.TO_time,TO_Urban.FC)
legend('lead','FCopt','noFCopt')
