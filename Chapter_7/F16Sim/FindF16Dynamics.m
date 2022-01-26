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

altitude = 15000;
velocity = 500;
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


A_OL_long = SS_long_lo.A([3 4 2 5 6 7],[3 4 2 5 6 7]);
B_OL_long = SS_long_lo.B([3 4 2 5 6 7],[1 2]);

A_OL_lat = SS_lat_lo.A([4 1 5 6 7 8], [4 1 5 6 7 8]);

%%  3.1/3.2
A_4 = SS_long_lo.A([3 4 2 5],[3 4 2 5]);
B_4 = SS_long_lo.A([3 4 2 5],7);
C_4 = SS_long_lo.C([3 4 2 5],[3 4 2 5]);
D_4 = SS_long_lo.D([3 4 2 5],2);


%% Build 2 state space model for short period motion
A_longitude = SS_long_lo.A([4 5],[4 5]);
B_longitude = SS_long_lo.A([4 5],7);
C_longitude = SS_long_lo.C([4 5],[4 5]);
D_longitude = SS_long_lo.D([4 5],1);

SS_shortP = ss(A_longitude,B_longitude,C_longitude,D_longitude);
TF_shortP = tf(SS_shortP);

%% Construct TF for 4 state space model
TF2 = tf(SS_long_lo);
[numerator_long,denominator_long] = ss2tf(A_4,B_4,C_4,D_4);

%% Plot response to step input

TF_long_pitchr = minreal(tf(numerator_long(4,:), denominator_long));

Timespan = 0: 0.01: 400;

% figure
% step(-TF_shortP(2),'r',Timespan);
% hold
% step(-TF_long_pitchr,'b',Timespan);
% legend('2-state model','4-state model')
% title('Response of pitch rate to step input')
% xlabel('Time[s]')
% ylabel('Pitch rate q [deg/s]')
% hold off

%% 3.3

Wnsp = 0.03*velocity *0.3048; % Natural frequency for the short period mode
theta2a = 1/(0.75*Wnsp);        % Time constant 
damp_sp = 0.5 ;                % Damping ratio of short period mode

% Calculation of closed loop poles
CL_poles = [complex(-Wnsp*damp_sp,Wnsp*sqrt(1-(damp_sp)^2)); complex(-Wnsp*damp_sp,-Wnsp*sqrt(1-(damp_sp)^2)) ];

% Pole placement in order to compute gain matrix K
K = place(A_longitude,B_longitude,CL_poles);

% Create the system with the pole placement as computed
A_poleplacement = A_longitude-B_longitude*K;

%State-Space with pole placement
SS_poleplacement = ss(A_poleplacement,B_longitude,C_longitude,D_longitude);

% Find Transfer Function
TF_poleplacement = minreal(tf(SS_poleplacement));

[dummy,denominator_pp] = ss2tf(A_poleplacement,B_longitude,C_longitude,D_longitude);

% Standard form --> Denominator = s^2 + 2*damping_ratio* natural frequency
% * s + natural frequency^2 
Wn = sqrt(denominator_pp(3));
damp_new = denominator_pp(2)/(2*Wn);


% Elevator deflection due to gusts
vertical_gust = 4.572; % [m/s]
gust_angle = atan(vertical_gust/(velocity*0.3048)); % gust angle from geometrical relations

delta_elavator = K(1)*gust_angle;
if delta_elavator <25
    disp(delta_elavator)

else
    disp('Gains not within legislation')

end

%% 3.5
Timespan = 0:0.01:5;

% Create lead lag filter]
theta2b = numerator_long(2, 2)/numerator_long(2, 3);

lead_lag_filter = tf([theta2a 1],[theta2b 1]);

% Apply lead lag filter to system
TF_final = minreal(TF_poleplacement*lead_lag_filter);

% Plotting to compare system with and without LL filter
% Without filter
[response_woF,dummy] = step(-TF_poleplacement(2),Timespan);

% With filter
[response_wF,dummy] = step(-TF_final(2),Timespan);

figure
plot(Timespan,response_woF,'b')
hold
plot(Timespan,response_wF,'r')
title('Response of pitch rate to step input')
legend('pole placement without lead-lag filter', 'pole placement with lead-lag filter')
xlabel('Time [s]')
ylabel('Pitch rate q [deg/s]')
hold off
