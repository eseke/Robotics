function a = glavni_b2(om)

%% celija za konstante

g = 9.81; % m/s^2
l = 1; % m
m = 1; % kg
ksi=10000;
%om=pi/6; %rad;
M0=1; %[N*m];
b=0.1; % const otpora

%% celija za domen resavanja



x0 = 0.8; % m
y0 = 0.6; % m
vx0 = 0; % m/s
vy0 = 0; % m/s
pocetni = [x0,y0,vx0,vy0]; % vektor pocetnih uslova
tstart = 0; % s
tend =300; % s
N = 2000; % broj tacka
t = linspace(tstart,tend,N); 
% diskretizacija vremenske ose, t postaje ekvidistantan niz brojeva, od 0
% do 20 u N tacaka

%% resavanje dif. jedancina
options = odeset; 
[t,x] = ode15s(@jed_kretanja_b2,t,pocetni,options,m,g,l,ksi,M0,om,b); 
%[nezavisna promenljiva, zavisna promenljiva] = ode_po_izboru(@na funkciju,nezavina promenljiva, pocetni uslovi, options, pozivni parametri po istom redu kao i u funkciji)
% ode23 - Euler metoda - linearne dif. jedancine
% ode45 - Runge-Kutta metoda - primerenija za linearne dif. jedancine
% ode15s - Adams metoda - nelinearne jedncine

a=max(abs(x(:,1)));