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

altitude = 5000;
velocity = 300;
g_D = 9.80665*3.2808399; %[ft/s^2]
h_airport = 3000;
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


% Trim values
trim_thrust = trim_thrust_lo;
trim_velocity = velocity;
trim_alt = altitude;
trim_elevator = trim_control_lo(1);

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

%% Create 5 state space model (set up for flare controller)
A_5 = SS_long_lo.A([1 2 3 4 5],[1 2 3 4 5]);
B_5 = SS_long_lo.A([1 2 3 4 5],[6 7]);
C_5 = SS_long_lo.C([1 2 3 4 5],[1 2 3 4 5]);
D_5 = SS_long_lo.D([1 2 3 4 5],[1 2]);

ss_lon_5 = ss(A_5,B_5,C_5,D_5);

% Set up GS
GS_angle = 3*(pi/180); % GLide slope angle [rad]
runway_distance = 10*velocity + (2000/tan(GS_angle));
runway_distance2 = (2000/tan(GS_angle));

% Set up flare geometry
x1 = 1100; %[ft]
h_dot = -velocity*sin(GS_angle);
x2 = 6.5*velocity-x1; % Check if 2< vertical speed at touchdown <3
h_flare = x2*tan(GS_angle);
tau = -h_flare/h_dot;

%sim GSandFlare




