function dxdt = jed_kretanja(t,x,g,l,ksi)

c = fc(t,x,l,ksi);
f=ogr(t,x);
dxdt = zeros(4,1);

dxdt(1)= x(3); %izvod x po t = vx
dxdt(2)= x(4); %izvod y po t = vy
%dxdt(3)=-(c+f(2)*dxdt(4))/f(1); %izvod vx po t = vx sa tackom
%dxdt(4)=g+f(2)*dxdt(3)/f(1); %izvod vy po t = vy sa tackom
dxdt(3)=(-c-f(2)*g)*f(1)/((f(1))^2+(f(2))^2);
dxdt(4)=g+f(2)*(-c-f(2)*g)/((f(1))^2+(f(2))^2);