
b=0.1; % const otpora
g = 9.81;
l = 1;
m = 1;
w0 = sqrt(g/l);
wrez = sqrt(w0^2-b^2/(2*m^2));
om = [0.1 0.2 0.5 0.8 0.9 0.95 1 1.05 1.1 1.2 1.5 1.7 1.9 1.95 2 2.5 3 4];

A = zeros(1,18);

for i = 1:18
    
A(i) = glavni_b1(wrez*om(i));

end

figure (1) 
plot(om,A)
xlabel('frekvencija prinudne sile [1/rad]');
ylabel('amplituda oscilovanja [m]');
grid on
axis equal
title('Amplitudska karakteristika za sinusni momenat');

for i = 1:18
A(i) = glavni_b2(wrez*om(i));

end

figure (2) 
plot(om,A)
xlabel('frekvencija prinudne sile [1/rad]');
ylabel('amplituda oscilovanja [m]');
grid on
axis equal
title('Amplitudska karakteristika za momenat povorki cetvrtki');
