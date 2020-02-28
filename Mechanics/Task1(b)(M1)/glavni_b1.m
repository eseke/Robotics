%% celija za inicijalizaciju

clear all % brise sve iz memorije
close all % gasi sve grafike u otvorenim p
clc % brise sve iz command window

%% celija za konstante

g = 9.81; % m/s^2
l = 1; % m
m = 1; % kg
ksi=10000;
om=pi/6; %rad;
M0=1; %[N*m];
b=0.1; % const otpora
% om=0.4; %rad;
% M0=5; %[N*m];
% b=0.2; % const otpora

%% celija za domen resavanja

x0 = 0.6; % m
y0 = 0.8; % m
vx0 = 0; %m/s
vy0 = 0; % m/s
pocetni = [x0,y0,vx0,vy0]; % vektor pocetnih uslova
tstart = 0; % s
tend =200; % s
N = 4000; % broj tacka
t = linspace(tstart,tend,N); 
% diskretizacija vremenske ose, t postaje ekvidistantan niz brojeva, od 0
% do 20 u N tacaka

%% resavanje dif. jedancina
options = odeset; 
[t,x] = ode15s(@jed_kretanja_b1,t,pocetni,options,m,g,l,ksi,M0,om,b); 
%[nezavisna promenljiva, zavisna promenljiva] = ode_po_izboru(@na funkciju,nezavina promenljiva, pocetni uslovi, options, pozivni parametri po istom redu kao i u funkciji)
% ode23 - Euler metoda - linearne dif. jedancine
% ode45 - Runge-Kutta metoda - primerenija za linearne dif. jedancine
% ode15s - Adams metoda - nelinearne jedncine


syms teta;
teta = atan(x(:,1)./x(:,2));
syms omega;
omega=(x(:,3)./abs(x(:,3))).*sqrt((x(:,3).^2+x(:,4).^2)./(x(:,1).^2+(x(:,2))));
syms S;
syms lambda;
syms C;
C = 2*(x(:,3).^2+x(:,4).^2)+2*ksi*(2*x(:,1).*x(:,3)+2*x(:,2).*x(:,4));

lambda=-m*(C+2*x(:,2)*g)./(2*(x(:,1).^2+x(:,2).^2));
S=lambda(:,1).*sqrt((2*x(:,1)).^2+(2*x(:,2)).^2);

%% crtanje grafika


figure(1) % otvaranje prozora za grafik 1
plot(t,x(:,1)); % x(:,) ekvivalentno svi redovi prve kolone, : = svi
xlabel('vreme t [s]');
ylabel('x koordinata [m]');
grid on
title('Grafik zavisnosti x koordinate od vremena');


figure(2) % otvaranje prozora za grafik 1
plot(t,x(:,2)); % x(:,) ekvivalentno svi redovi prve kolone, : = svi
xlabel('vreme t [s]');
ylabel('y koordinata [m]');
grid on
title('Grafik zavisnosti y koordinte od vremena');


figure(3) % otvaranje prozora za grafik 1
plot(t,x(:,3)); % x(:,) ekvivalentno svi redovi prve kolone, : = svi
xlabel('vreme t [s]');
ylabel('vx koordinata [m/s]');
grid on
title('Grafik zavisnosti brzine po x osi od vremena');


figure(4) % otvaranje prozora za grafik 1
plot(t,x(:,4)); % x(:,) ekvivalentno svi redovi prve kolone, : = svi
xlabel('vreme t [s]');
ylabel('vy koordinata [m/s]');
grid on
title('Grafik zavisnosti brzine po y osi od vremena');


figure(5) %zavisnost ugla otklona od vremena
plot(t,teta(:,1));
xlabel('vreme t[s]');
ylabel('ugao [rad]');
grid on
title('Zavisnosti ugla otklona u funkciji vremena');


figure(6) %zavisnost ugaone brzine od vremena
plot(t,omega(:,1));
xlabel('vreme t[s]');
ylabel('ugaona brzina [rad/s]');
grid on
title('Grafik zavisnosti ugaone brzine od vremena');


figure (7) %trajektorija
plot(x(:,1),x(:,2));
xlabel('x koordinata [m]');
ylabel('y koordinata [m]');
grid on
title('Grafik trajektorije klatna');


figure (8) %zavisnost ugaone brzine od ugla otkolona
plot(teta(:,1),omega(:,1));
xlabel('ugao [rad]');
ylabel('ugaona brzina [rad/s]');
grid on
axis equal
title('Grafik zavisnosti ugaone brzine od ugla otkolona');


figure (9) %zavisnost sile ogranicenja od vremena
plot(t,S(:,1));
xlabel('vreme [s]');
ylabel('sila ogranicenja [N]');
grid on
title('Grafik zavisnosti sile ogranicenja od vremena');


figure (10) %zavisnost ogranicenja od vremena
plot(t,x(:,1).^2+x(:,2).^2-l^2);
xlabel('vreme [s]');
ylabel('ogranicenje /f [m]');
grid on
axis equal
title('Grafik zavisnosti ogranicenja od vremena');