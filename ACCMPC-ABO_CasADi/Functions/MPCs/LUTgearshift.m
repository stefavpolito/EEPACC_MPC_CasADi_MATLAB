function [tau_measured,gear_measured] = LUTgearshift(v_measured)
% LUTgearshift Generate a transmission ratio to implement the effect of 
% the gear shifting process when running the plant model (DAILY ONLY).
%
%   Inputs:
%       v_measured : simulated measured velocity in the current step
%   Outputs:
%       tau_measured : simulated measured gearbox ratio in the current step

    %% Initialization

    V       = SetVehicleParameters();       % vehicle parameters

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
