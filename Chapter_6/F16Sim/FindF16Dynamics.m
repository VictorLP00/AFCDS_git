%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 
% Edit: Ewoud Smeur (2021)
%================================================
clear;

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
%altitude = input('Enter the altitude for the simulation (ft)  :  ');
%velocity = input('Enter the velocity for the simulation (ft/s):  ');

altitude = 30000;
velocity = 900;
g_D = 9.80665*3.2808399; %[ft/s^2]
%x_a = 0;



x_a = 0;

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for Hifi model at desired altitude and velocity
%%
%disp('Trimming High Fidelity Model:');
fi_flag_Simulink = 1;
%[trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
%operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
%operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
%operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

%SS_hi = linearize('LIN_F16Block');

%% Find trim for lofi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the lofi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_lo = linearize('LIN_F16Block');



%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Direction
%%%%%%%%%%%%%%%%%%%%%%%

long_states = [3 5 7 8 11 13 14];
long_inputs = [1 2];
long_outputs = [3 5 7 8 11];

SS_long_lo = ss(SS_lo.A(long_states,long_states), SS_lo.B(long_states,long_inputs), SS_lo.C(long_outputs,long_states), SS_lo.D(long_outputs,long_inputs));
%SS_long_hi = ss(SS_hi.A(long_states,long_states), SS_hi.B(long_states,long_inputs), SS_hi.C(long_outputs,long_states), SS_hi.D(long_outputs,long_inputs));

SS_long_lo.StateName = SS_lo.StateName(long_states);
%SS_long_hi.StateName = SS_hi.StateName(long_states);

SS_long_lo.InputName= SS_lo.InputName(long_inputs);
%SS_long_hi.InputName= SS_hi.InputName(long_inputs);

%%%%%%%%%%%%%%%%%%%%
%% Lateral Direction
%%%%%%%%%%%%%%%%%%%%

lat_states = [4 6 7 9 10 12 13 15 16];
lat_inputs = [1 3 4];
lat_outputs = [4 6 7 9 10 12];

SS_lat_lo = ss(SS_lo.A(lat_states,lat_states), SS_lo.B(lat_states,lat_inputs), SS_lo.C(lat_outputs,lat_states), SS_lo.D(lat_outputs,lat_inputs));
%SS_lat_hi = ss(SS_hi.A(lat_states,lat_states), SS_hi.B(lat_states,lat_inputs), SS_hi.C(lat_outputs,lat_states), SS_hi.D(lat_outputs,lat_inputs));

SS_lat_lo.StateName = SS_lo.StateName(lat_states);
%SS_lat_hi.StateName = SS_hi.StateName(lat_states);

SS_lat_lo.InputName= SS_lo.InputName(lat_inputs);
%SS_lat_hi.InputName= SS_hi.InputName(lat_inputs);

%%%%%%%%%%%%%%%%
%% Matrix creation
%%%%%%%%%%%%%%%%


A_OL_long = SS_long_lo.A([3 4 2 5],[3 4 2 5 ]);
B_OL_long = SS_long_lo.B([3 4 2 5 ],2);
C_OL_long = SS_long_lo.C([3 4 2 5 ], [3 4 2 5]);
D_OL_long = SS_long_lo.D([3 4 2 5 ],2);

SS_long = ss(A_OL_long,B_OL_long,C_OL_long,D_OL_long);

% Eigenmotions long
[V_long,~]=eig(A_OL_long);
%Poles long
P_long=eig(A_OL_long);

A_OL_lat = SS_lat_lo.A([4 1 5 6], [4 1 5 6]);
B_OL_lat = SS_lat_lo.B([4 1 5 6], [1 2 3]);
C_OL_lat = SS_lat_lo.C([4 1 5 6], [4 1 5 6]);
D_OL_lat = SS_lat_lo.D([4 1 5 6], [1 2 3]);

SS_lat = ss(A_OL_lat,B_OL_lat,C_OL_lat,D_OL_lat);
%Eigenmotions lat
[V_lat,~] = eig(A_OL_lat);
%Poles lat
P_lat = eig(A_OL_lat);

%Eigenmotions

EigModes=[P_long([1, 3],:);  P_lat([1,3:4],:)]; %select the corresponding eigenmotions
Wn=zeros(5,1); Damping=zeros(5,1); Period=zeros(5,1); Timeconst=zeros(5,1); % make zero matrices for the different parameters

for i=1:5
        Wn(i,1)=sqrt((real(EigModes(i,1)))^2 + (imag(EigModes(i,1)))^2); %calculate natural frecuency
    
    if imag(EigModes(i,1))==0  %Calculations for aperiodic eigenmotion, because no imaginary part
        Period(i,1)  = 0;
        Timeconst(i,1)= -1 / EigModes(i,1);
        Damping(i,1) = - real(EigModes(i,1))/ Wn(i,1); 
    else 
        Period(i,1)  = 2*pi / imag(EigModes(i,1));
        Timeconst(i,1)= 0;
        Damping(i,1) = - real(EigModes(i,1))/ Wn(i,1);
    end 

end 
T_half = -log(0.5)./(Damping.*Wn); %calculation of time to half damp amplitude
%% Results
Modes_= {'Short period'; 'Phughoid'; 'Dutch roll'; 'Aperiodic roll'; 'Spiral'};
Parameters_={'Modes', 'Wn', 'Zeta', 'Period', 'Time to half',  'Timeconst'};
values = [ Wn Damping Period T_half Timeconst];

[y_SP, t_SP] = initial(SS_long,real(V_long(:,1)));     % Short Period
[y_Ph, t_Ph] = initial(SS_long,real(V_long(:,3)));     % Phugoid
[y_DR, t_DR] = initial(SS_lat,real(V_lat(:,1)));        % Dutch Roll
[y_AR, t_AR] = initial(SS_lat,real(V_lat(:,3)));        % Aperiodic Roll
[y_Sp, t_Sp] = initial(SS_lat,real(V_lat(:,4)));        % Spiral

close all

% Phugoid
figure
subplot(4,1,1)
plot(t_Ph, y_Ph(:,1))
title('Mode: Phugoid')
ylabel('V_{t} (ft/s)')      % Total velocity          
subplot(4,1,2)
plot(t_Ph, y_Ph(:,2))
ylabel('\alpha (deg)')      % Angle of attack
subplot(4,1,3)
plot(t_Ph, y_Ph(:,3))
ylabel('\theta (deg)')      % Pitch angle
subplot(4,1,4)
plot(t_Ph, y_Ph(:,4))
ylabel('q (deg/s)')         % Pitch rate 
xlabel('Time (s)')

% Short Period
figure
subplot(4,1,1)
plot(t_SP, y_SP(:,1))
title('Mode: Short Period')
ylabel('V_{t} (ft/s)')      % Total velocity          
subplot(4,1,2)
plot(t_SP, y_SP(:,2))
ylabel('\alpha (deg)')      % Angle of attack
subplot(4,1,3)
plot(t_SP, y_SP(:,3))
ylabel('\theta (deg)')      % Pitch angle
subplot(4,1,4)
plot(t_SP, y_SP(:,4))
ylabel('q (deg/s)')         % Pitch rate 
xlabel('Time (s)')

% Dutch Roll
figure
subplot(4,1,1)
plot(t_DR, y_DR(:,1))
title('Mode: Dutch Roll')
ylabel('\beta (deg)')       % Side slip angle
subplot(4,1,2)
plot(t_DR, y_DR(:,2))
ylabel('\phi (deg)')        % Roll angle
subplot(4,1,3)
plot(t_DR, y_DR(:,3))
ylabel('p (deg/s)')         % Roll rate
subplot(4,1,4)
plot(t_DR, y_DR(:,4))
ylabel('r (deg/s)')         % Yaw rate 
xlabel('Time (s)')

% Aperiodic Roll
figure
subplot(4,1,1)
plot(t_AR, y_AR(:,1))
title('Mode: Aperiodic Roll')
ylabel('\beta (deg)')       % Side slip angle
subplot(4,1,2)
plot(t_AR, y_AR(:,2))
ylabel('\phi (deg)')        % Roll angle
subplot(4,1,3)
plot(t_AR, y_AR(:,3))
ylabel('p (deg/s)')         % Roll rate
subplot(4,1,4)
plot(t_AR, y_AR(:,4))
ylabel('r (deg/s)')         % Yaw rate 
xlabel('Time (s)')

% Spiral
figure
subplot(4,1,1)
plot(t_Sp, y_Sp(:,1))
title('Mode: Spiral')
ylabel('\beta (deg)')       % Side slip angle
subplot(4,1,2)
plot(t_Sp, y_Sp(:,2))
ylabel('\phi (deg)')        % Roll angle
subplot(4,1,3)
plot(t_Sp, y_Sp(:,3))
ylabel('p (deg/s)')         % Roll rate
subplot(4,1,4)
plot(t_Sp, y_Sp(:,4))
ylabel('r (deg/s)')         % Yaw rate 
xlabel('Time (s)')

%% All Poles
% figure(1); 
% pzmap(SS_hi, 'r', SS_lo, 'b');
% title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nAll Poles\n Blue = lofi Red = hifi.', altitude, velocity);
% title(title_string);
% sgrid;
% 
% %% Longitudinal Poles
% %%
% figure(2); 
% pzmap(SS_long_hi, 'r', SS_long_lo, 'b');
% title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
% title(title_string);
% sgrid;
% 
% %% Lateral Poles
% %%
% figure(3); 
% pzmap(SS_lat_hi, 'r', SS_lat_lo, 'b');
% title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLateral Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
% title(title_string);
% sgrid;
% 
% %% Bode plot longitudinal 
% 
% % Choose an input and 
% input = 2;
% output = 3;
% 
% omega = logspace(-2,2,100);
% 
% figure
% bode(SS_long_hi(output,input),omega)
% hold on;
% bode(SS_long_lo(output,input),omega)
% legend('hifi','lofi')
% 
% %% Bode plot lateral 
% 
% % Choose an input and 
% input = 2;
% output = 3;
% 
% omega = logspace(-2,2,100);
% 
% figure
% bode(SS_lat_hi(output,input),omega)
% hold on;
% bode(SS_lat_lo(output,input),omega)
% legend('hifi','lofi')