% Script to simulate the behavior of an inverted pendulum bot using a
% feedback controller.
%
% Created by Galen Savidge, 3/23/2019

clear all
close all

% Simulation parameters
dt = 0.01; % Seconds
sim_time = 5; % Seconds
B_desired = 0; % Desired position of center of mass, meters
max_speed = 1; % Bot top speed, m/s
timescale = 0.5;

make_gif = 0;
filename = 'inverted_pendulum_side.gif';
gif_timescale = 0.2;

% Graphing parameters
body_color = [0.1 0.1 0.8];
wheel_color = [0.8 0.1 0.1];

% Bot physical attributes
bot_mass = 1; % In kg
bot_height = 0.15; % 15 cm, ~6 in
bot_width = 0.075; % 7.5 cm, ~3 in
bot_depth = 0.025; % 2.5 cm, ~1 in
wheel_radius = 0.038; % 1.5 in
wheel_max_w = 100*2*pi/60; % Rad/s
r = bot_height/2; % Center of wheel to center of mass

% Initialize simulated physical system
v_r = 0; % Right wheel speed, m/s
v_l = 0; % Left wheel speed, m/s
pitch = pi/4; % Radians
w = 0; % Rad/s
w_dot = 0; % Rad/s^2
yaw = 0;
w_yaw = 0;
W = 0; % Distance traveled by wheels, meters
v_W = 0; % Wheel velocity, m/s

% Constants
g = -9.80665; % In m/s^2

% Position controller (modified PD)
K_pos = 0.2;
Kp_pos = 1.5*K_pos;
Kd_pos = 1*K_pos;

% Run high level controller for the first step
B = [W - r*sin(pitch); wheel_radius + r*cos(pitch)];
pos_err = B(1) - B_desired;
pos_err_p = sign(pos_err)*min(abs(pos_err), max_speed*Kd_pos/Kp_pos);
c_pos = Kp_pos*pos_err_p;
last_pos_err = pos_err;

pitch_desired = c_pos;

% Pitcch controller (standard PID)
K_pitch = 25;
Kp_pitch = 1*K_pitch;
Ki_pitch = 0.02*K_pitch;
Kd_pitch = 0.05*K_pitch;

last_pitch_err = pitch - pitch_desired;
pitch_integrator = 0;

% History for graphing
num_steps = sim_time/dt;
t = 0:dt:(sim_time-dt);
pitch_hist = zeros(1, num_steps);
v_W_hist = zeros(1, num_steps);
w_W_hist = zeros(1, num_steps);

% Set up gif capture
if make_gif == 1
    frame_time = dt/gif_timescale;
end

% Set up real time plot
f = figure(1);
axis([-0.25 0.25 -0.25 0.25])
axis equal
hold on
grid on
xlabel('X_I (Meters)')
ylabel('Y_I (Meters)')
title('Inverted Pendulum Bot Simulation')

pause

for i = 1:num_steps
    % High level controller
    pos_err = B(1) - B_desired;
    pos_err_p = sign(pos_err)*min(abs(pos_err), max_speed*Kd_pos/Kp_pos);
    c_pos = Kp_pos*pos_err_p + Kd_pos*(pos_err - last_pos_err)/dt;
    last_pos_err = pos_err;
    
    pitch_desired = c_pos;
    
    % Low level controller
    pitch_err = pitch - pitch_desired;
    pitch_integrator = pitch_integrator + Ki_pitch*pitch_err*dt;
    c_pitch = Kp_pitch*pitch_err + Kd_pitch*(pitch_err - last_pitch_err)/dt + pitch_integrator;
    last_pitch_err = pitch_err;
    
    % Find wheel speeds
    v_W_dot = -c_pitch;
    
    % Find pitch
    w_dot = v_W_dot*cos(pitch)/r - g*sin(pitch)/r;
    w = w + w_dot*dt;
    pitch = pitch + w*dt;
    pitch_hist(i) = 180*pitch/pi;
    
    % Find wheel speed
    v_W = v_W + v_W_dot*dt;
    w_W = -(v_W/wheel_radius + w);
%     w_W = sign(w_W)*min(5, abs(w_W)); % Cap wheel speed
%     v_W = -(w_W + w)*wheel_radius;
    
    % Find distance
    W = W + v_W*dt;
    
    v_W_hist(i) = v_W;
    w_W_hist(i) = w_W*60/(2*pi);
    
    % Find body position
    B = [W - r*sin(pitch); wheel_radius + r*cos(pitch)];
    
    % Plot simulation side view
    figure(1)
    cla
    draw_rectangle_rotated(B, bot_depth, bot_height, pitch, body_color)
    plot(B(1), B(2), 'w*')
    rectangle('Position',[W(1) - wheel_radius, 0 , 2*wheel_radius, 2*wheel_radius],...
    'Curvature',[1,1], 'FaceColor', wheel_color);
    plot([B(1)-1 B(1)+1], [0, 0], 'k')
    
    % Capture to <filename>
    if make_gif == 1
        frame = getframe(f);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if i == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_time);
        end
    end
    
    pause(max(dt/timescale-0.01, 0.0001))
end

figure(2)
plot(t, pitch_hist)
title('Pitch over time')
xlabel('Time (s)')
ylabel('Pitch (degrees)')
grid on

figure(3)
plot(t, v_W_hist)
title('Wheel translational speed over time')
xlabel('Time (s)')
ylabel('Wheel speed (m/s)')
grid on

figure(4)
plot(t, w_W_hist)
title('Wheel angular velocity over time')
xlabel('Time (s)')
ylabel('Wheel speed (RPM)')
grid on