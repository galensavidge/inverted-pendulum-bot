% Script to simulate the behavior of an inverted pendulum bot using a
% feedback controller.
%
% Created by Galen Savidge, 3/23/2019

clear all
close all

% Simulation parameters
dt = 0.001; % Seconds
sim_time = 1; % Seconds
pitch_desired = pi/3; % Radians

% Initialize simulated physical system
r_wheel = 0;
l_wheel = 0;
pitch = pi/8; % Radians
w_pitch = 0; % Rad/s
w_pitch_dot = 0; % Rad/s^2
yaw = 0;
w_yaw = 0;
w_yaw_dot = 0;
bot_height = 0.15; % Meters
bot_width = 0.05;
bot_depth = 0.02;
wheel_radius = 0.01;
bot_mass = 0.1; % In kg

g = -9.80665; % In m/s^2

I_y = bot_mass*(bot_height^2 + bot_depth^2)/12; % Inertia about y axis
I_z = bot_mass*(bot_width^2 + bot_depth^2)/12; % Inertia about z axis

num_steps = sim_time/dt;
t = 0:dt:(sim_time-dt);

% Feedback controller
Kp = 5;
Ki = 3;
Kd = .15;
last_err = pitch - pitch_desired;
integrator = 0;

err_hist = zeros(1, num_steps);
pitch_hist = zeros(1, num_steps);
grav_torque_hist = zeros(1, num_steps);

for i = 1:num_steps
    % Controller
    err = pitch - pitch_desired;
    integrator = integrator + Ki*err*dt;
    c = Kp*err + Kd*(err - last_err)/dt + integrator;
    err_hist(i) = 180*err/pi;
    last_err = err;
    
    wheel_force = -c;
    
    % Distance from center of wheel to center of rotation
    r = bot_height/2 - wheel_radius;
    
    grav_torque = bot_mass*g*r*cos(pitch); % Gravity
    wheel_torque = wheel_force*r/sin(pitch);
    w_pitch_dot = (grav_torque + wheel_torque)/I_y;
    
    w_pitch = w_pitch + w_pitch_dot*dt;
    % w_yaw = w_yaw + w_yaw_dot*dt;
    
    pitch = pitch + w_pitch*dt;
    
    pitch_hist(i) = 180*pitch/pi;
    grav_torque_hist(i) = grav_torque;
end

figure(1)
plot(t, pitch_hist)
title('Pitch over time')
xlabel('Time (s)')
ylabel('Pitch (degrees)')
grid on

figure(2)
plot(t, grav_torque_hist)
title('Gravity torque over time')
xlabel('Time (s)')
ylabel('Gravity torque (N)')
grid on

figure(3)
plot(t, err_hist)
title('Error over time')
xlabel('Time (s)')
ylabel('Error (degrees)')
grid on