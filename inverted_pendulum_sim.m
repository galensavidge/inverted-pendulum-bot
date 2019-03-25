% Script to simulate the behavior of an inverted pendulum bot using a
% feedback controller.
%
% Created by Galen Savidge, 3/23/2019

clear all
close all

% Simulation parameters
dt = 0.01; % Seconds
sim_time = 10; % Seconds
B_desired = 1; % Meters
% pitch_desired = 0; % Radians

% Graphing parameters
body_color = [0.1 0.1 0.8];
wheel_color = [0.8 0.1 0.1];

% Initialize simulated physical system
r_wheel = 0;
l_wheel = 0;
pitch = pi/6; % Radians
w = 0; % Rad/s
w_dot = 0; % Rad/s^2
yaw = 0;
w_yaw = 0;
w_yaw_dot = 0;
bot_height = 0.15; % Meters
bot_width = 0.05;
bot_depth = 0.02;
wheel_radius = 0.02;
W = 0; % Midpoint between wheels position, meters
v_W = 0; % Wheel velocity, m/s
bot_mass = 1; % In kg

g = -9.80665; % In m/s^2

% Distance from center of wheel to center of rotation
r = bot_height/2;

num_steps = sim_time/dt;
t = 0:dt:(sim_time-dt);

% High level feedback controller
K_pos = 0.15;
Kp_pos = 1*K_pos;
Ki_pos = 0*K_pos;
Kd_pos = 1*K_pos;
max_pos_err = 0.5;

B = [W - r*sin(pitch); wheel_radius + r*cos(pitch)];
last_pos_err = B(1) - B_desired;

pos_integrator = 0;

% Run high level controller for the first step
pos_err = B(1) - B_desired;

pos_integrator = pos_integrator + Ki_pos*pos_err*dt;
c_pos = Kp_pos*pos_err + Kd_pos*(pos_err - last_pos_err)/dt + pos_integrator;
last_pos_err = pos_err;

pitch_desired = c_pos;

% Low level feedback controller
K_pitch = 50;
Kp_pitch = 1*K_pitch;
Ki_pitch = 0.02*K_pitch;
Kd_pitch = 0.05*K_pitch;

last_pitch_err = pitch - pitch_desired;
pitch_integrator = 0;

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
xlabel('X_I (Meters)')
ylabel('Y_I (Meters)')
title('Inverted Pendulum Bot Simulation')

for i = 1:num_steps
    % High level controller
    pos_err = B(1) - B_desired;
    pos_integrator = pos_integrator + Ki_pos*pos_err*dt;
    c_pos = Kp_pos*pos_err + Kd_pos*(pos_err - last_pos_err)/dt + pos_integrator;
    last_pos_err = pos_err;
    
    pitch_desired = c_pos;
    
    % Low level controller
    pitch_err = pitch - pitch_desired;
    pitch_integrator = pitch_integrator + Ki_pitch*pitch_err*dt;
    c_pitch = Kp_pitch*pitch_err + Kd_pitch*(pitch_err - last_pitch_err)/dt + pitch_integrator;
    last_pitch_err = pitch_err;
    
    v_W_dot = -c_pitch;
    
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
    cla
    draw_rectangle_rotated(B, bot_depth, bot_height, pitch, body_color)
    plot(B(1), B(2), 'w*')
    rectangle('Position',[W(1) - wheel_radius, 0 , 2*wheel_radius, 2*wheel_radius],...
    'Curvature',[1,1], 'FaceColor',wheel_color);
    plot([B(1)-1 B(1)+1], [0, 0], 'k')
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