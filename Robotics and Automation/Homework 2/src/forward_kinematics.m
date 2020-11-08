function [X,dX] = forward_kinematics(q, dq, J)

global l1 l2
l=l1;
% Dekartove kordinate
x = l*cos(q(2));%  X
y = l*sin(q(2));%  Y
z = l+q(1);%  Z

X = [ x; y; z];

% Brzina u Dekartovim kordinatama
dX = J*dq; 