%% celija za inicijalizaciju

clear all % brise sve iz memorije
close all % gasi sve grafike u otvorenim p
clc % brise sve iz command window

%% celija za konstante

g = 9.81; % m/s^2
l1 = 1; % m
l2 = 1; % m
m1 = 1; % kg
m2 = 1; %kg

%% celija za domen resavanja

teta10 = 0; % m
teta30 = 0; % m
omega10 = 0; % m/s
omega30 = 0; % m/s
pocetni = [teta10,teta30,omega10,omega30]; % vektor pocetnih uslova
tstart = 0; % s
tend = 100; % s
N = 2000; % broj tacka
t = linspace(tstart,tend,N); 
% diskretizacija vremenske ose, t postaje ekvidistantan niz brojeva, od 0
% do 20 u N tacaka

%% resavanje dif. jedancina
options = odeset; 
[t,x] = ode15s(@jed_kretanja_z2,t,pocetni,options,g,m1,m2,l1,l2); 
%[nezavisna promenljiva, zavisna promenljiva] = ode_po_izboru(@na funkciju,nezavina promenljiva, pocetni uslovi, options, pozivni parametri po istom redu kao i u funkciji)
% ode23 - Euler metoda - linearne dif. jedancine
% ode45 - Runge-Kutta metoda - primerenija za linearne dif. jedancine
% ode15s - Adams metoda - nelinearne jedncine

teta2 = x(:,2) - x(:,1);
omega2 = x(:,4) - x(:,3);


%% crtanje grafika
 % otvaranje prozora za grafik 1
plot(t,x(:,1)); 
xlabel('vreme t [s]');
ylabel('ugao \theta1 [rad]');
grid on
title('Grafik zavisnosti ugla \theta1 od vremena');


figure(2)
plot(t,teta2);
xlabel('vreme t [s]');
ylabel('ugao \theta2 [rad]');
grid on
title('Grafik zavisnosti ugla \theta2 od vremena');


figure(3) 
plot(t,x(:,3)); 
xlabel('vreme t [s]');
ylabel('ugaona brzina donjeg stapa [rad/s]');
grid on
title('Grafik zavisnosti ugaone brzine od vremena');


figure(4)
plot(t,omega2);
xlabel('vreme t [s]');
ylabel('ugaona brzina gornjeg stapa [rad/s]');
grid on
title('Grafik zavisnosti ugaone brzine gornjeg stapa od vremena');


figure(5)
plot(l1*cos(x(:,1))+l2*cos(x(:,2)),l1*sin(x(:,1))+l2*sin(x(:,2)));
xlabel('x koordinata [m]');
ylabel('y koordinata [m]');
grid on
title('Trajektorija vrha gornjeg stapa');

