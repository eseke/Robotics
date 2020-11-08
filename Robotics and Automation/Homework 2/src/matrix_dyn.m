function [H, C, G] = matrix_dyn(q, dq,m,l,g)

%global m l I g 

% matrica inercije H
H11 = 2*m;
H12 = 0;
H21 = 0;
H22 = m*l^2/3;

H = [H11 H12;...
     H21 H22];

% matrica brzinskih efekata
C11 = 0;
C12 = 0;
C21 = 0;
C22 = 0;

C = [C11 C12;...
     C21 C22];

% gravitaciona matrica
G11 = 2*m*g;
G21 = 0;

G = [G11;...
     G21];
