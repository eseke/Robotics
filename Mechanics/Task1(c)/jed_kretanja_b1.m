function dxdt = jed_kretanja_b1(t,x,m,g,l,ksi,M0,om,b)
M=moment1(t,M0,om);
c = fc(t,x,l,ksi);
f=ogr(t,x);
dxdt = zeros(4,1);

dxdt(1)= x(3); %izvod x po t = vx
dxdt(2)= x(4); %izvod y po t = vy
dxdt(3)=-f(1)*(c+f(2)/m*(m*g-b*x(4)+M(1)+f(2)/f(1)*(b*x(3)-M(1))))/(f(1)^2+f(2)^2);   %-(c+f(2)*(f(1)^2*(-b*x(4)+M(1))+m*g*f(1)^2+f(2)*f(1)*(-M(1)+b*x(3))-m*f(2)*c)/(m*(f(1)^2+f(2)^2)))/f(1);
dxdt(4)=(f(1)^2*(-b*x(4)+M(1))+m*g*f(1)^2+f(2)*f(1)*(-M(1)+b*x(3))-m*f(2)*c)/(m*(f(1)^2+f(2)^2));