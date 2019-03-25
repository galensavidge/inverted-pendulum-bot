% Script to simulate the behavior of an inverted pendulum bot using a
% feedback controller.
%
% Created by Galen Savidge, 3/23/2019

clear all
close all

% Simulation parameters
dt = 0.01; % Seconds
sim_time = 1; % Seconds
pitch_desired = -pi/6; % Radians

% Initialize simulated physical system
r_wheel = 0;
l_wheel = 0;
pitch = -pi/3; % Radians
w = 0; % Rad/s
w_dot = 0; % Rad/s^2
yaw = 0;
w_yaw = 0;
w_yaw_dot = 0;
bot_height = 0.15; % Meters
bot_width = 0.05;
bot_depth = 0.02;
wheel_radius = 0.01;
W = 0; % Midpoint between wheels position, meters
v_W = 0; % Wheel velocity, m/s
bot_mass = 0.1; % In kg

g = -9.80665; % In m/s^2

% Distance from center of wheel to center of rotation
r = bot_height/2 - wheel_radius;

num_steps = sim_time/dt;
t = 0:dt:(sim_time-dt);

% Feedback controller
Kp = 100;
Ki = 20;
Kd = 5;
last_err = pitch - pitch_desired;
integrator = 0;

% History for graphing
pitch_hist = zeros(1, num_steps);
W_hist = zeros(1, num_steps);
w_W_hist = zeros(1, num_steps);

% Set up real time plot
figure(1)
axis([-0.25 0.25 -0.25 0.25])
axis equal
hold on
grid on

for i = 1:num_steps
    % Controller
    err = pitch - pitch_desired;
    integrator = integrator + Ki*err*dt;
    c = Kp*err + Kd*(err - last_err)/dt + integrator;
    last_err = err;
    
    v_W_dot = -c;
    
    % Find pitch
    w_dot = v_W_dot*cos(pitch)/r - g*sin(pitch)/r;
    w = w + w_dot*dt;
    pitch = pitch + w*dt;
    pitch_hist(i) = 180*pitch/pi;
    
    % Find wheel position
    v_W = v_W + v_W_dot*dt;
    W = W + v_W*dt;
    
    w_W_hist(i) = -v_W/wheel_radius;
    
    % Find body position
    B = [W - r*sin(pitch); wheel_radius + r*cos(pitch)];
    
    figure(1)
    clf
    axis([-0.25 0.25 -0.25 0.25])
    axis equal
    hold on
    grid on
    draw_rectangle_rotated(B, bot_depth, bot_height, pitch, 'red')
    % plot(B(1), B(2), '*')
    pause(0.001)
end

figure(2)
plot(t, pitch_hist)
title('Pitch over time')
xlabel('Time (s)')
ylabel('Pitch (degrees)')
grid on

figure(3)
plot(t, w_W_hist)
title('Wheel speed over time')
xlabel('Time (s)')
ylabel('Wheel speed (rad/s)')
grid on