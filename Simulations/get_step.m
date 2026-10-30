function [x_dot] = get_step(state)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    state
end

arguments (Output)
    x_dot
end

mu = 3.986004418e14;

x_dot = zeros(6, 1);
x_dot(1:3) = state(4:6);


x_dot(4:6) = -1.0 * mu / (norm(state(1:3))^3) * state(1:3);

%disp(x_dot)
end