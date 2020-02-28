function dxdt = jed_kretanja_z2(t,x,g,m1,m2,l1,l2)

M = moment(t);

dxdt = zeros(4,1);

dxdt(1)= x(3); %izvod x po t = vx
dxdt(2)= x(4); %izvod y po t = vy
dxdt(3)=4*M/5/m1/l1/l1-0*(m1/2+m2)*4*cos(x(1))/5/m1/l1;
dxdt(4)=-2*g*cos(x(2))/l2;
