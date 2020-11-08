close all;

figure(1)
plot(Ps.t,Ps.q_ref(1,:),Ps.t,Ps.q(1,:),'r','LineWidth',2)
grid;
title('Unutrasnja kordinata q1-pozicija prvog zgloba');
xlabel('vreme [s]');
ylabel('pomeraj [m]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(2)
plot(Ps.t,Ps.q_ref(2,:),Ps.t,Ps.q(2,:),'r','LineWidth',2)
grid;
title('Unutrasnja kordinata q2-pozicija drugog zgloba');
xlabel('vreme [s]');
ylabel('ugao [dg]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(3)
plot(Ps.t,Ps.dq_ref(1,:),Ps.t,Ps.dq(1,:),'r','LineWidth',2)
grid;
title('Unutrasnja kordinata dq1-brzina prvog zgloba');
xlabel('vreme [s]');
ylabel('brzina [m/s]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(4)
plot(Ps.t,Ps.dq_ref(2,:),Ps.t,Ps.dq(2,:),'r','LineWidth',2)
grid;
title('Unutrasnja kordinata dq2-brzina drugog zgloba');
xlabel('vreme [s]');
ylabel('ugaona brzina [dg/s]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(5)
plot(Ps.t,Ps.ddq_ref(1,:),Ps.t,Ps.ddq(1,:),'r','LineWidth',2)
grid;
title('Unutrasnja kordinata ddq1-ubrzanje prvog zgloba');
xlabel('vreme [s]');
ylabel('ubrzanje [m/s^2]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(6)
plot(Ps.t,Ps.ddq_ref(2,:),Ps.t,Ps.ddq(2,:),'r','LineWidth',2)
grid;
title('Unutrasnja kordinata ddq2-ubrzanje drugog zgloba');
xlabel('vreme [s]');
ylabel('ugaono ubrzanje [dg/s^2]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(7)
plot(Ps.t,Ps.X_ref(1,:),Ps.t,Ps.X(1,:),'r','LineWidth',2)
grid;
title('Spoljasnja kordinata x-pozicija na x osi');
xlabel('vreme [s]');
ylabel('pozicija [m]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(8)
plot(Ps.t,Ps.X_ref(2,:),Ps.t,Ps.X(2,:),'r','LineWidth',2)
grid;
title('Spoljasnja kordinata y-pozicija na y osi');
xlabel('vreme [s]');
ylabel('pozicija [m]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(9)
plot(Ps.t,Ps.X_ref(3,:),Ps.t,Ps.X(3,:),'r','LineWidth',2)
grid;
title('Spoljasnja kordinata z-pozicija na z osi');
xlabel('vreme [s]');
ylabel('pozicija [m]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(10)
plot(Ps.t,Ps.dX_ref(1,:),Ps.t,Ps.dX(1,:),'r','LineWidth',2)
grid;
title('Spoljasnja kordinata dx-brzina na x osi');
xlabel('vreme [s]');
ylabel('brzina [m/s]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(11)
plot(Ps.t,Ps.dX_ref(2,:),Ps.t,Ps.dX(2,:),'r','LineWidth',2)
grid;
title('Spoljasnja kordinata dy-brzina na y osi');
xlabel('vreme [s]');
ylabel('brzina [m/s]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(12)
plot(Ps.t,Ps.dX_ref(3,:),Ps.t,Ps.dX(3,:),'r','LineWidth',2)
grid;
title('Spoljasnja kordinata dz-brzina na z osi');
xlabel('vreme [s]');
ylabel('brzina [m/s]');
legend('referentana vrednost','ostvarena vrednost');
%set(gca,'FontSize',20)

figure(13)
plot(Ps.t,Ps.Tau(1,:),'LineWidth',2)
grid;
title('Ostvarena pogonska sila prvog zgloba');
xlabel('vreme [s]');
ylabel('sila [N]');
%set(gca,'FontSize',20)

figure(14)
plot(Ps.t,Ps.Tau(2,:),'LineWidth',2)
grid;
title('Ostvareni pogonski moment drugog zgloba');
xlabel('vreme [s]');
ylabel('moment sile [Nm]');
%set(gca,'FontSize',20)

figure(15)
subplot(2,1,1)
plot(Ps.t,Ps.Tau_FF(1,:),'LineWidth',2)
grid;
title('feedforward vrednost upravljanja za prvi zglob');
xlabel('vreme [s]');
ylabel('sila [N]');
%set(gca,'FontSize',20)
subplot(2,1,2)
plot(Ps.t,Ps.Tau_FF(2,:),'LineWidth',2)
grid;
title('feedforward vrednost upravljanja za drugi zglob');
xlabel('vreme [s]');
ylabel('Momenat sile [Nm]');
%set(gca,'FontSize',20)

figure(16)
subplot(2,1,1)
plot(Ps.t,Ps.Tau_FB(1,:),'LineWidth',2)
grid;
title('feedback vrednost upravljanja za prvi zglob');
xlabel('vreme [s]');
ylabel('Sila [N]');
%set(gca,'FontSize',20)
subplot(2,1,2)
plot(Ps.t,Ps.Tau_FB(2,:),'LineWidth',2)
grid;
title('feedback vrednost upravljanja za drugi zglob');
xlabel('vreme [s]');
ylabel('Momenat sile [Nm]');
%set(gca,'FontSize',20)

% figure(11)
% subplot(2,1,1)
% plot(Ps.t,Ps.struja_rot(1,:),'LineWidth',2)
% grid;
% title('vrednost struje motora za prvi zglob');
% xlabel('vreme [s]');
% ylabel('struja [A]');
% subplot(2,1,2)
% plot(Ps.t,Ps.struja_rot(2,:),'LineWidth',2)
% grid;
% title('vrednost struje motora za drugi zglob');
% xlabel('vreme [s]');
% ylabel('struja [A]');

% figure(12)
% plot(Ps.X_ref(1,:),Ps.X_ref(2,:),Ps.X(1,:),Ps.X(2,:),'r','LineWidth',2)
% grid;
% title('Kretanje robota u X-Z ravni');
% xlabel('X [m]');
% ylabel('Z [m]');
% legend('referentana vrednost','ostarena vrednost');
% 
% figure(13)
% plot(Ps.t,Ps.dX_ref(1,:),Ps.t,Ps.dX(1,:),'r','LineWidth',2)
% grid;
% title('Spoljasnja kordinata dx-x komponenta brzine');
% xlabel('vreme [s]');
% ylabel('brzina [m/s]');
% legend('referentana vrednost','ostvarena vrednost');
% 
% figure(14)
% plot(Ps.t,Ps.dX_ref(2,:),Ps.t,Ps.dX(2,:),'r','LineWidth',2)
% grid;
% title('Spoljasnja kordinata dz-z komponenta brzine');
% xlabel('vreme [s]');
% ylabel('brzina [m/s]');
% legend('referentana vrednost','ostvarena vrednost');
% 
% figure(15)
% plot(Ps.t,Ps.ddq_ref(2,:),Ps.t,Ps.ddq(2,:),'r','LineWidth',2)
% grid;
% title('ubrzanje');
% xlabel('vreme [s]');
% ylabel('brzina [m/s]');
% legend('referentana vrednost','ostvarena vrednost');

