%Author: Vishagen Ramasamy
%Date: -

% Copyright University of Southampton 2017.
% No warranty either expressed or implied is given to the results produced 
% by this software.  Neither the University, students or its employees 
%accept any responsibility for use of or reliance on results produced by 
%this software.

%% script that computes the filling of the tank(s)
%% Importing the pressure and temperature profile at the entrance of the delivery pipe from Dicken and Merida
%  and calulating the stagnation enthalpy and entropy

for j = 1:maxt
    time(j+1) = (j)*dt;                                                      % Time as filling proceeds
    for i = 1:tank_number
        % Select inlet pressure / temperature profile based on user input
        if blnUseStandardData == 1
            % Constant inlet pressure profile
            P_inlet(i,j+1) = ConstPressKPa;
        else
            % Read inlet pressure profile from specified file
            P_inlet(i, j+1) = interp1(InletPressureData(:,1),InletPressureData(:,3), dt*j);
        end
        
        if blnUseStandardData == 1
            %Temp_inlet(i,j+1) = interp1(time_in, temp_in, dt*j);   % Temperature profile after the first 1.35 seconds of the fill based on Dicken and Merida (2007)
            Temp_inlet(i, j+1) = ConstTempK;
        else
            % Read inlet temperature profile from specified file
            Temp_inlet(i, j+1) = interp1(InletTempData(:,1), InletTempData(:,3), dt*j);
        end
        
        h_inlet(i,j+1) = refpropm('H','T',Temp_inlet(i,j+1),'P',P_inlet(i,j+1),Fluid{i}, refpropdir);       % Returns  specific total enthalpy at the entrance of the delivery pipe
        entropy_inlet(i,j+1) = refpropm('S','T',Temp_inlet(i,j+1),'P',P_inlet(i,j+1),Fluid{i}, refpropdir); % Returns  specific entropy at the entrance of the delivery pipe
        
        %% Determine which pressure to use at the exit of the delivery pipe which is dependent upon the number of zones
        
        if l_d(i) > 3 && blnOneZone{i} == 0          % The length-to-diameter ratio of the tank(s) determines the number of zones
            Pressure_exit = P_gas_zone1{i}(j);   % The pressure at the exit of the delivery pipe is equal to the gas pressure in zone 1
        else
            Pressure_exit = P_gas(i,j);          % The pressure at the exit of the delivery pipe is equal to the gas pressure within the whole tank(s)
        end
        
        if (P_inlet(i,j+1) > Pressure_exit)  % condition for filling of tank(s)
            sound_exit(i,j+1) = refpropm('A','P',Pressure_exit,'S',entropy_inlet(i,j+1),Fluid{i}, refpropdir);     % Returns the speed of sound at the exit of the delivery pipe
            h_static_exit(i,j+1) = refpropm('H','P',Pressure_exit,'S',entropy_inlet(i,j+1),Fluid{i}, refpropdir);  % Returns the static enthalpy at the exit of the delivery pipe
            visc_exit(i,j+1) = refpropm('V','P',Pressure_exit,'S',entropy_inlet(i,j+1),Fluid{i}, refpropdir);      % Returns the viscosity at the exit of the delivery pipe
            mach_exit(i,j+1) = sqrt(2*(h_inlet(i,j+1)-h_static_exit(i,j+1))/sound_exit(i,j+1)^2);   % Calculates the Mach number at the exit of the delivery pipe
            
            %% If Mach number is greater than one, the exit pressure is greater than the pressure within the tank(s) and
            %  is incremetally increased and iterated in a while loop  until mach number is equal to one
            
            Inlet_entropy = entropy_inlet(i,j+1);
            Inlet_stagnation_enthalpy = h_inlet(i,j+1);
            P_guess =  Pressure_exit;
            
            if mach_exit(i,j+1) > 1
                Pressure_exit = find_exit_pressure(Inlet_stagnation_enthalpy,Inlet_entropy,Fluid{i},P_guess, refpropdir);  % Calls function that iterates the exit pressure until the exit Mach number is equal to one
                sound_exit(i,j+1) = refpropm('A','P',Pressure_exit,'S',Inlet_entropy,Fluid{i}, refpropdir);                % Returns the speed of sound at the exit of the delivery pipe for choking conditions
                h_static_exit(i,j+1) = refpropm('H','P',Pressure_exit,'S',Inlet_entropy,Fluid{i}, refpropdir);             % Returns the static enthalpy at the exit of the delivery pipe for choking conditions
                visc_exit(i,j+1) = refpropm('V','P',Pressure_exit,'S',Inlet_entropy,Fluid{i}, refpropdir);                 % Returns the viscosity at the exit of the delivery pipe for choking conditions
                mach_exit(i,j+1) = sqrt(2*(h_inlet(i,j+1)-h_static_exit(i,j+1))/sound_exit(i,j+1)^2);       % Confirms that the Mach number at the exit of the delivery pipe is one for choking conditions
            end
            
            %% Calculation of the mass flow rate into the tank(s)
            
            rho_exit(i,j+1) = refpropm('D','P',Pressure_exit,'S',Inlet_entropy,Fluid{i}, refpropdir);                  % Returns the density of the gas at the exit of the delivery pipe
            vel_exit(i,j+1) = mach_exit(i,j+1)*sound_exit(i,j+1);                                       % Calculates the velocity of the gas at the exit of the delivery pipe
            Re_exit_isentropic(i,j+1) = rho_exit(i,j+1) *d_inlet*vel_exit(i,j+1)/visc_exit(i,j+1);      % Calculates the isentropic Reynolds number at the exit of the delivery pipe
            cd(i,j+1) =  I + J/(Re_exit_isentropic(i,j+1)^(0.25));                                      % Calculates the discharge coefficient
            mfr(i,j+1) = cd(i,j+1)*rho_exit(i,j+1)*vel_exit(i,j+1)*A_inlet;                             % Calculates the mass flow rate of the gas into the tank(s)
            Re_entrance_actual(i,j+1)= 4*mfr(i,j+1)/(pi*d_inlet*visc_exit(i,j+1));                      % Calculates the actual Reynolds number at the entrance of the delivery pipe
            dM_inlet = mfr(i,j+1)*dt;                                                                   % Amount of mass of gas being fed into the tank at each time step
        end
        
        %% Heat transfer calculations & caluclations of the thermodynamic properties of the gas in the tank(s) when L/D is less than three
        if l_d(i) <= 3 | blnOneZone{i} == 1
            
            Nus(i,j+1) =   a_1*Re_entrance_actual(i,j+1)^(b_1);                   % Nusselt number and Reynolds number correlation for a single zone model (a_1 and b_1 are constants and are defined in the main script)
            k_gas(i,j+1) = refpropm('L','T',Temp_gas(i,j),'P',P_gas(i,j),Fluid{i}, refpropdir);  % Thermal conductivity of the gas in the tank(s) during the fill
            heat_coef_forced(i,j+1) = Nus(i,j+1)*k_gas(i,j+1)/d_tank(i);          % Calculation of the heat transfer coefficient for the 1 zone model during the fill
            
            if Inner_wall_boundary(i) == 1                                                                                                                        % Condition for isothermal inner wall
                Qsurf(i,j+1) = -dt*surf_area(i)*heat_coef_forced(i,j+1)*(Temp_gas(i,j)-Inner_temp_wall_isothermal(i));                                               % Heat transfer when the inner wall is set to isothermal conditions
            else                                                                                                                                                  % Condition for conjugate heat transfer
                Qsurf(i,j+1) = -dt*surf_area(i)*heat_coef_forced(i,j+1)*(Temp_gas(i,j)-Temp_wall{i}(1,j));                                                           % Heat transfer for conjugate heat transfer case
                Temp_wall{i}(1,j+1) = Temp_wall{i}(1,j)+ CFL_liner(i)*(Temp_wall{i}(2,j)- Temp_wall{i}(1,j) - Qsurf(i,j+1)*dr(i)/(cond_liner(i)*(surf_area(i)*dt))); % Calculates inner wall temperature for conjugate heat transfer case
                
                if Outer_wall_boundary(i)==1                                                                                                                                                                 % Condition for isothermal outer wall conditions
                    Temp_wall{i}(number_of_gridpoints(i),j+1) = Outer_temp_wall_isothermal(i);                                                                                                                   % Temperature at the outer wall for isothermal outer wall conditions
                else                                                                                                                                                                                         % Condition for adiabatic outer wall conditions
                    Temp_wall{i}(number_of_gridpoints(i),j+1) = Temp_wall{i}(number_of_gridpoints(i),j)+2*CFL_laminate(i)*(Temp_wall{i}(number_of_gridpoints(i)-1,j)-Temp_wall{i}(number_of_gridpoints(i),j));   % Temperature at the outer wall for adiabatic outer wall conditions
                end
                % Computation of the temperature of the struture of the tank(s)
                for k=2:number_of_gridpoints(i)-1
                    if (k>=2)&&(k<=int_pt_liner_laminate(i)-1)
                        Temp_wall{i}(k,j+1) = Temp_wall{i}(k,j)+CFL_liner(i)*(Temp_wall{i}(k+1,j)-2*Temp_wall{i}(k,j)+Temp_wall{i}(k-1,j));                                % Temperature across the liner
                    elseif (k>=int_pt_liner_laminate(i)+1)&&(k<=number_of_gridpoints(i)-1)
                        Temp_wall{i}(k,j+1) = Temp_wall{i}(k,j)+CFL_laminate(i)*(Temp_wall{i}(k+1,j)-2*Temp_wall{i}(k,j)+Temp_wall{i}(k-1,j));                             % Temperature across the laminate
                    else
                        Temp_wall{i}(k,j+1) = (cond_laminate(i)*(Temp_wall{i}(k+1,j)+CFL_laminate(i)*(Temp_wall{i}(k+2,j)-2*Temp_wall{i}(k+1,j)+Temp_wall{i}(k,j)))+cond_liner(i)*Temp_wall{i}(k-1,j+1))/(cond_liner(i)+cond_laminate(i)); % Temperature at the interface of the liner and the laminate
                    end
                end
            end
            m_gas(i,j+1) = m_gas(i,j)+ dM_inlet;                                               % Mass of gas in the tank(s) as the filling proceeds
            Ugas(i,j+1)=Ugas(i,j)+Qsurf(i,j+1)+h_inlet(i,j+1)*dM_inlet;                        % Internal energy of the gas in the tank(s) as the filling proceeds
            u_gas(i,j+1)= Ugas(i,j+1)/m_gas(i,j+1);                                            % Specific energy of the gas in the tank(s) as the filling proceeds
            rho_gas(i,j+1)=m_gas(i,j+1)/vol_tank(i);                                           % Density of the gas in the tank(s) as the filling proceeds
            P_gas(i,j+1) = refpropm('P','D',rho_gas(i,j+1),'U',u_gas(i,j+1),Fluid{i}, refpropdir);            % Pressure of the gas in the tank(s) as the filling proceeds
            Temp_gas(i,j+1) = refpropm('T','D',rho_gas(i,j+1),'U',u_gas(i,j+1),Fluid{i}, refpropdir);         % Temperature of the gas in the tank(s) as the filling proceeds
            
        else
            Re_compression(i,j+1) = Re_entrance_actual(i,j+1)*(d_inlet/d_tank(i));             % Reynolds number for zone 2
            
            Nus_zone1{i}(j+1) =  a_1*Re_entrance_actual(i,j+1)^(b_1);                          % Nusselt number and Reynolds number correlation for zone 1 (a_1 and b_1 are constants and are defined in the main script)
            Nus_zone2{i}(j+1) =  c_1*Re_compression(i,j+1)^(d_1);                              % Nusselt number and Reynolds number correlation for zone 1 (c_1 and d_1 are constants and are defined in the main script)
            
            k_gas_zone1{i}(j+1) =  refpropm('L','T',Temp_gas_zone1{i}(j),'P',P_gas_zone1{i}(j),Fluid{i}, refpropdir);    % Thermal conductivity of the gas in zone 1 during the fill
            k_gas_zone2{i}(j+1) =  refpropm('L','T',Temp_gas_zone2{i}(j),'P',P_gas_zone2{i}(j),Fluid{i}, refpropdir);    % Thermal conductivity of the gas in zone 1 during the fill
            
            heat_coef_forced_zone1{i}(j+1) =  Nus_zone1{i}(j+1)*k_gas_zone1{i}(j+1)/d_tank(i);                % Calculation of the heat transfer coefficient for zone 1 during the fill
            heat_coef_forced_zone2{i}(j+1) =  Nus_zone2{i}(j+1)*k_gas_zone2{i}(j+1)/l_zone2_total(i);         % Calculation of the heat transfer coefficient for zone 2 during the fill
            
            % Step 1. Equating the respecting values of zone 1 and zone 2 to
            % matrices with 'similar' names
            
            Vgas(1)=volume_zone1(i);
            Vgas(2)=volume_zone2(i);
            Mgas(1)=m_gas_zone1{i}(j);
            Mgas(2)=m_gas_zone2{i}(j);
            Ugas_twozone(1)=u_gas_zone1{i}(j)*Mgas(1);
            Ugas_twozone(2)=u_gas_zone2{i}(j)*Mgas(2);
            Asurf(1)=surface_area_zone1(i);
            Asurf(2)=surface_area_zone2(i);
            Tgas(1)=Temp_gas_zone1{i}(j);
            Tgas(2)=Temp_gas_zone2{i}(j);
            if Inner_wall_boundary(i) == 1
                Twall(1,1)=Inner_temp_wall_isothermal(i);
                Twall(2,1)=Inner_temp_wall_isothermal(i);
            else
                Twall(1,1)=Temp_wall_zone1{i}(1,j);
                Twall(2,1)=Temp_wall_zone2{i}(1,j);
            end
            % Step 2: apply the change of internal energy due to heat transfer and mass
            % input through nozzle:
            
            dM_inlet = mfr(i,j+1)*dt;
            
            Qsurf(1)=- dt*Asurf(1)*(heat_coef_forced_zone1{i}(j+1))*(Tgas(1)-Twall(1,1));
            Qsurf(2)=- dt*Asurf(2)*(heat_coef_forced_zone2{i}(j+1))*(Tgas(2)-Twall(2,1));
            
            Ugas_twozone(1)=Ugas_twozone(1)+Qsurf(1)+h_inlet(i,j+1)*dM_inlet;
            Ugas_twozone(2)=Ugas_twozone(2)+Qsurf(2);
            
            Mgas(1)=Mgas(1)+dM_inlet;
            Mgas(2)=Mgas(2);
            
            for m=1:2
                hgas(m)=refpropm('H','D',Mgas(m)/Vgas(m),'U',Ugas_twozone(m)/Mgas(m),Fluid{i}, refpropdir);
            end
            
            
            % Step 2: Find the amount of mass that needs to be transferred from zone 1
            % to zone 2 to equalise their pressure
            % (we assume forward Euler integration, therefore we use the specific
            % enthalpies h from the start of the timestep. We provide enthalpies for
            % both zones in case the flow gets reversed).
            
            dM_guess = dM_inlet*(volume_zone2(i)/(volume_zone1(i)+volume_zone2(i))) ;
            dM_12 = find_dM_12(hgas,Vgas,Mgas,Ugas_twozone,Fluid{i},dM_guess, refpropdir);                          % Calls function that iterates the mass transfer from zone 1 to zone 2 until pressure equilibrium is achieved
            
            % Step 3: Apply this change to the mass and update all properties:
            
            m_gas_zone1{i}(j+1)=Mgas(1)-dM_12;                                                                               % Mass of gas in zone 1 as filling proceeds
            m_gas_zone2{i}(j+1)=Mgas(2)+dM_12;                                                                               % Mass of gas in zone 2 as filling proceeds
            
            u_gas_zone1{i}(j+1)=(Ugas_twozone(1)-max(0,dM_12)*hgas(1)-min(0,dM_12)*hgas(2))/m_gas_zone1{i}(j+1);                     % Specific internal energy of the gas in zone 1
            u_gas_zone2{i}(j+1)=(Ugas_twozone(2)+max(0,dM_12)*hgas(1)+min(0,dM_12)*hgas(2))/m_gas_zone2{i}(j+1);                     % Specific internal energy of the gas in zone 2
            
            rho_gas_zone1{i}(j+1)=m_gas_zone1{i}(j+1)/volume_zone1(i);                                                       % Density of gas in zone 1
            rho_gas_zone2{i}(j+1)=m_gas_zone2{i}(j+1)/volume_zone2(i);                                                       % Density of gas in zone 2
            
            Temp_gas_zone1{i}(j+1)=refpropm('T','D',rho_gas_zone1{i}(j+1),'U', u_gas_zone1{i}(j+1),Fluid{i}, refpropdir);                   % Temperature of the gas in zone 1
            Temp_gas_zone2{i}(j+1)=refpropm('T','D',rho_gas_zone2{i}(j+1),'U', u_gas_zone2{i}(j+1),Fluid{i}, refpropdir);                   % Temperature of the gas in zone 2
            
            P_gas_zone1{i}(j+1)=refpropm('P','D',rho_gas_zone1{i}(j+1),'U', u_gas_zone1{i}(j+1),Fluid{i}, refpropdir);                      % Pressure of the gas in zone 1 (must be equal to Pressure of the gas in zone 2)
            P_gas_zone2{i}(j+1)=refpropm('P','D',rho_gas_zone2{i}(j+1),'U', u_gas_zone2{i}(j+1),Fluid{i}, refpropdir);                      % Pressure of the gas in zone 2 (must be equal to Pressure of the gas in zone 1)
            
            m_gas(i,j+1) = m_gas_zone1{i}(j+1) + m_gas_zone2{i}(j+1);                                                        % Total mass of gas in the tank
            rho_gas(i,j+1) = m_gas(i,j+1)/vol_tank(i);                                                                       % Density of the gas in the tank
            u_gas(i,j+1)= (u_gas_zone1{i}(j+1)*m_gas_zone1{i}(j+1)+ u_gas_zone2{i}(j+1)*m_gas_zone2{i}(j+1))/m_gas(i,j+1);   % Specific internal energy of the gas in the tank
            Ugas(i,j+1) = u_gas(i,j+1)*m_gas(i,j+1);                                                                         % Internal energy of the gas in the tank
            Temp_gas(i,j+1)  = refpropm('T','D',rho_gas(i,j+1),'U',u_gas(i,j+1),Fluid{i}, refpropdir);                                      % Returns the average temperature of gas in tank as filling proceeds
            P_gas(i,j+1)  = refpropm('P','D',rho_gas(i,j+1),'U',u_gas(i,j+1),Fluid{i}, refpropdir);                                         % Returns pressure of the gas in the cylinder (must be equal to P_gas_zone1 and P_gas_zone2)
            
            if Inner_wall_boundary(i) == 1                                                                                                                        % Condition for isothermal inner wall
                Qsurf_zone1{i}(j+1) = -dt*surface_area_zone1(i)* heat_coef_forced_zone1{i}(j+1)*(Temp_gas_zone1{i}(j+1)-Inner_temp_wall_isothermal(i));               % Heat transfer when the inner wall is set to isothermal conditions for zone 1
                Qsurf_zone2{i}(j+1) = -dt*surface_area_zone2(i)* heat_coef_forced_zone2{i}(j+1)*(Temp_gas_zone2{i}(j+1)-Inner_temp_wall_isothermal(i));               % Heat transfer when the inner wall is set to isothermal conditions for zone 2
            else                                                                                                                                                  % Condition for conjugate heat transfer
                Qsurf_zone1{i}(j+1) = -dt*surface_area_zone1(i)* heat_coef_forced_zone1{i}(j+1)*(Temp_gas_zone1{i}(j+1)-Temp_wall_zone1{i}(1,j));                     % Heat transfer for conjugate heat transfer case for zone 1
                Qsurf_zone2{i}(j+1) = -dt*surface_area_zone2(i)* heat_coef_forced_zone2{i}(j+1)*(Temp_gas_zone2{i}(j+1)-Temp_wall_zone2{i}(1,j));                     % Heat transfer for conjugate heat transfer case for zone 2
                
                Temp_wall_zone1{i}(1,j+1) = Temp_wall_zone1{i}(1,j)+ CFL_liner(i)*(Temp_wall_zone1{i}(2,j)- Temp_wall_zone1{i}(1,j) - Qsurf_zone1{i}(j+1)*dr(i)/(cond_liner(i)*(surface_area_zone1(i)*dt)));   % Calculates inner wall temperature for conjugate heat transfer case (zone 1)
                Temp_wall_zone2{i}(1,j+1) = Temp_wall_zone2{i}(1,j)+ CFL_liner(i)*(Temp_wall_zone2{i}(2,j)- Temp_wall_zone2{i}(1,j) - Qsurf_zone2{i}(j+1)*dr(i)/(cond_liner(i)*(surface_area_zone2(i)*dt)));   % Calculates inner wall temperature for conjugate heat transfer case (zone 2)
                
                if Outer_wall_boundary(i)==1                                                                                                                                                                   % Condition for isothermal outer wall conditions
                    Temp_wall_zone1{i}(number_of_gridpoints(i),j+1) = Outer_temp_wall_isothermal(i);                                                                                                             % Temperature at the outer wall for isothermal outer wall conditions (zone 1)
                    Temp_wall_zone2{i}(number_of_gridpoints(i),j+1) = Outer_temp_wall_isothermal(i);                                                                                                             % Temperature at the outer wall for isothermal outer wall conditions (zone 2)
                else                                                                                                                                                                                         % Condition for adiabatic outer wall conditions
                    Temp_wall_zone1{i}(number_of_gridpoints(i),j+1) = Temp_wall_zone1{i}(number_of_gridpoints(i),j)+2*CFL_laminate(i)*(Temp_wall_zone1{i}(number_of_gridpoints(i)-1,j)-Temp_wall_zone1{i}(number_of_gridpoints(i),j));   % Temperature at the outer wall for adiabatic outer wall conditions (zone 1)
                    Temp_wall_zone2{i}(number_of_gridpoints(i),j+1) = Temp_wall_zone2{i}(number_of_gridpoints(i),j)+2*CFL_laminate(i)*(Temp_wall_zone2{i}(number_of_gridpoints(i)-1,j)-Temp_wall_zone2{i}(number_of_gridpoints(i),j));   % Temperature at the outer wall for adiabatic outer wall conditions (zone 2)
                end
                % Computation of the temperature of the struture of the tank(s)
                for k=2:number_of_gridpoints(i)-1
                    if (k>=2)&&(k<=int_pt_liner_laminate(i)-1)
                        Temp_wall_zone1{i}(k,j+1) = Temp_wall_zone1{i}(k,j)+CFL_liner(i)*(Temp_wall_zone1{i}(k+1,j)-2*Temp_wall_zone1{i}(k,j)+Temp_wall_zone1{i}(k-1,j));                                % Temperature across the liner (zone 1)
                        Temp_wall_zone2{i}(k,j+1) = Temp_wall_zone2{i}(k,j)+CFL_liner(i)*(Temp_wall_zone2{i}(k+1,j)-2*Temp_wall_zone2{i}(k,j)+Temp_wall_zone2{i}(k-1,j));                                % Temperature across the liner (zone 2)
                    elseif (k>=int_pt_liner_laminate(i)+1)&&(k<=number_of_gridpoints(i)-1)
                        Temp_wall_zone1{i}(k,j+1) = Temp_wall_zone1{i}(k,j)+CFL_laminate(i)*(Temp_wall_zone1{i}(k+1,j)-2*Temp_wall_zone1{i}(k,j)+Temp_wall_zone1{i}(k-1,j));                             % Temperature across the laminate (zone 1)
                        Temp_wall_zone2{i}(k,j+1) = Temp_wall_zone2{i}(k,j)+CFL_laminate(i)*(Temp_wall_zone2{i}(k+1,j)-2*Temp_wall_zone2{i}(k,j)+Temp_wall_zone2{i}(k-1,j));                             % Temperature across the laminate (zone 2)
                    else
                        Temp_wall_zone1{i}(k,j+1) = (cond_laminate(i)*(Temp_wall_zone1{i}(k+1,j)+CFL_laminate(i)*(Temp_wall_zone1{i}(k+2,j)-2*Temp_wall_zone1{i}(k+1,j)+Temp_wall_zone1{i}(k,j)))+cond_liner(i)*Temp_wall_zone1{i}(k-1,j+1))/(cond_liner(i)+cond_laminate(i)); % Temperature at the interface of the liner and the laminate (zone 1)
                        Temp_wall_zone2{i}(k,j+1) = (cond_laminate(i)*(Temp_wall_zone2{i}(k+1,j)+CFL_laminate(i)*(Temp_wall_zone2{i}(k+2,j)-2*Temp_wall_zone2{i}(k+1,j)+Temp_wall_zone2{i}(k,j)))+cond_liner(i)*Temp_wall_zone2{i}(k-1,j+1))/(cond_liner(i)+cond_laminate(i)); % Temperature at the interface of the liner and the laminate (zone 2)
                    end
                end
            end
            
        end
        
    end
end