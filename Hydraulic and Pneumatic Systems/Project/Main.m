clear all;
close all;
clc;

%% inicijalizacija param
Bp = 30; %da
K = 0; %da
Ap = 6.5e-3; %da
Mt = 8; %da
Vt = 4.4e-3; %da
Kc = 1.8e-12; %da
Ctp = 2e-13; %da
Kq = 1; %da
Kt = 3.86e-3;  %da
Kf = 2.5e3; %da
R = 10; %da 
r = 0.005; %da
Be = 13e8; %da
Um = 10; %da
L = 0.15; %da
T = 1.5;% da
m = 20;%da
k = 7e5;%da
b = 150;%da
Mi = 0.02;%da
g = 10;
Kkr = 3729;
Tkr = 26.736e-3;
Umax = 24;

%% matrica A
a21 = -(K+2*k/3)/Mt;
a22 = -(Bp+b)/Mt;
a23 = Ap/Mt;
a24 = 2*k/3/Mt;
a25 = b/Mt;
a32 = -Ap*4*Be/Vt;
a33 = -(Ctp+Kc)*4*Be/Vt;
a51 = 2*k/3/m;
a52 = b/m;
a54 = -14*k/3/m;
a55 = -b/m;
a56 = 4*k/m;
a74 = 8*k/m;
a76 = -12*k/m;  
A = [0 1 0 0 0 0 0; a21 a22 a23 a24 a25 0 0; 0 a32 a33 0 0 0 0;0 0 0 0 1 0 0;a51 a52 0 a54 a55 a56 0; 0 0 0 0 0 0 1;0 0 0 a74 0 a76 0];

%% matrica B
b31 = Kq*4*Be*Kt/(Vt*r*R*Kf);
B = [ 0; 0; b31; 0; 0; 0; 0];
%% matrica C
C = [1 1 1 1 1 1 1]';

%% matrica E

e51 = Mi*g;
e71 = Mi*g/2;
E = [0; 0; 0; 0; e51; 0; e71];
%% Izvrsavanje simulacije

Tsim = 170; %Vreme simulacije
sim('Pr_model')


%% Crtanje grafika
close all

t = Ocek.time(1:10:length(Ocek.time));

figure()
x1ref = Ocek.signals.values(1:10:length(Ocek.time),3);
plot(t,x1ref)
title('Zeljeno kretanje objekta upravljanja')
xlabel('Vreme [s]')
ylabel('Rastojanje od pocetnog polozaja [m]')

figure()
x1_br_ref = Ocek.signals.values(1:10:length(Ocek.time),2);
plot(t,x1_br_ref)
title('Zeljena brzina objekta upravljanja')
xlabel('Vreme [s]')
ylabel('Brzina objekta [m/s]')

figure()
x1_ub_ref = Ocek.signals.values(1:10:length(Ocek.time),1);
plot(t,x1_ub_ref)
title('Zeljeno ubrzanje objekta upravljanja')
xlabel('Vreme [s]')
ylabel('Ubrzanje objekta [m/s^2]')

figure()
x1ref = Ocek.signals.values(1:10:length(Ocek.time),3);
x1 = Stanja.signals.values(1:10:length(Ocek.time),4);
plot(t,x1ref)
hold all
plot(t,x1)
title('Kretanje objekta upravljanja(telo mase m)')
xlabel('Vreme [s]')
ylabel('Rastojanje od pocetnog polozaja [m]')
legend('Referentno kretanje','Ostvareno kretanje')

figure()
Xp = Stanja.signals.values(1:10:length(Ocek.time),1);
plot(t,Xp)
title('Kretanje klipnjace')
xlabel('Vreme [s]')
ylabel('Rastojanje od pocetnog polozaja [m]')

figure()
Xp_br = Stanja.signals.values(1:10:length(Ocek.time),2);
plot(t,Xp_br)
title('Brzina klipnjace')
xlabel('Vreme [s]')
ylabel('Brzina [m/s]')

figure()
Pl = Stanja.signals.values(1:10:length(Ocek.time),3);
plot(t,Pl)
title('Pritisak radnog fluida u cilindru')
xlabel('Vreme [s]')
ylabel('Pritisak [Pa]')

figure()
X1_br = Stanja.signals.values(1:10:length(Ocek.time),5);
X1_br_ref = Ocek.signals.values(1:10:length(Ocek.time),2);

plot(t,X1_br_ref)
hold all
plot(t,X1_br)
title('Brzina objekta upravljanja')
xlabel('Vreme [s]')
ylabel('Brzina [m/s^2]')
legend('Referentna','Ostvarena')

figure()
Xp = Stanja.signals.values(1:10:length(Ocek.time),6);
plot(t,Xp)
title('Kretanje tela mase m/2')
xlabel('Vreme [s]')
ylabel('Rastojanje od pocetnog polozaja [m]')

figure()
Xp_br = Stanja.signals.values(1:10:length(Ocek.time),7);
plot(t,Xp_br)
title('Brzina tela mase m/2')
xlabel('Vreme [s]')
ylabel('Brzina [m/s]')

figure()
Uu = Uprv.signals.values(1:10:length(Ocek.time),5);
plot(t,Uu)
title('Ukupno upravljanje')
xlabel('Vreme [s]')
ylabel('Napon [V]')

figure()
Uu = Uprv.signals.values(1:10:length(Ocek.time),1);
plot(t,Uu)
title('Proporcijalna komponenta upravljanja pre prilagodjenja aktuatoru')
xlabel('Vreme [s]')
ylabel('Napon [V]')

figure()
Uu = Uprv.signals.values(1:10:length(Ocek.time),2);
plot(t,Uu)
title('Integralna komponenta upravljanja pre prilagodjenja aktuatoru')
xlabel('Vreme [s]')
ylabel('Napon [V]')

figure()
Uu = Uprv.signals.values(1:10:length(Ocek.time),3);
plot(t,Uu)
title('Diferencijalna komponenta upravljanja pre prilagodjenja aktuatoru')
xlabel('Vreme [s]')
ylabel('Napon [V]')

%% Racunanje trazenih parametara

Fmax = max(2*k/3*(Stanja.signals.values(1:length(Stanja.time),1)-Stanja.signals.values(1:length(Stanja.time),4))+b*(Stanja.signals.values(1:length(Stanja.time),2)-Stanja.signals.values(1:length(Stanja.time),5)));
h = max(Stanja.signals.values(1:length(Stanja.time),1));
v = max(Stanja.signals.values(1:length(Stanja.time),2));