function out = trajectory(u,P)

% input(1*1):time
% output(6*1):desired trajectory and desired yaw angle

t = u(end);
b1_d = [1 0 0];
x_d = [2.0 + sin(t) -1 + 2*cos(t) 5.0]; 

out = [x_d b1_d];
end
