function draw_rectangle_rotated(center, w, h, theta, color)
% Draws a rectangle rotated about the z axis by theta.
% center: vector for the center of the rectangle
% w: width
% h: height
% theta: angle in radians
% color: color of the rectangle
% Created by Galen Savidge, 3/24/2019

c = center(1:2);

R = [cos(theta) -sin(theta);
     sin(theta)  cos(theta)];

v1 = R*[w/2; h/2] + c;
v2 = R*[-w/2; h/2] + c;
v3 = R*[-w/2; -h/2] + c;
v4 = R*[w/2; -h/2] + c;

V = [v1 v2 v3 v4];
patch(V(1,:), V(2,:), color)