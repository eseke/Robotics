function dydt_Q_4 = int_2DoF(t, Q_4,m,l,g,Tau,TauF)

% Function that defines model of 2DOF robot manipulator
%         [tout,Xout] = ode45(@int_2DoF,[t t+dt], q);
%         X = Xout(end,:)';

% calculates H, C, G
% H = [2*m 0;0 m*l^2/3];
% C = [0 0;0 0];
% G = [2*m*g;0];
 
% dydtQ_2 = H\(Tau - C*[Q_4(3);Q_4(4)]-G-TauF);

% dydt_Q_4(1:2) = Q_4(3:4);
% dydt_Q_4(3:4) = dydtQ_2;
dydt_Q_4 =[Q_4(3);Q_4(4);[2*m 0;0 m*l^2/3]\(Tau -[2*m*g;0]-TauF)];