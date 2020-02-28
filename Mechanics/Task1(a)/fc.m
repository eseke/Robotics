function c = fc(t,x,l,ksi)

f=ogr(t,x);
%c = 2*((x(3))^2+(x(4))^2)+2*ksi*(f(1)*x(3)+f(2)*x(4))+ksi*ksi*(0);
c=[x(3) x(4)]*[0 2;2 0]*[x(3);x(4)]+2*ksi*(f(1)*x(3)+f(2)*x(4));