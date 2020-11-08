function [J] = matrix_kin(q)

global l1

l = l1;
J11 = 0;
J12 = -l*sin(q(2));
J21 = 0;
J22 = l*cos(q(2));
J31 = 1;
J32 = 0;
J = [J11 J12;...
     J21 J22;...
     J31 J32];
