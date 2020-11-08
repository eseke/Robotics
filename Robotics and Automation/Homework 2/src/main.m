clear all; close all; clc

global m I l l1 l2 g Fint  Tau  F m1 m2 TauF

%%
parametri_simulacije;
I = I1;
l = l1;
m = m1;
%%
planiranje_trajektorije;

%% controller
PD_parametri;
i=1;

%glavna petlja
while (t<T)
   t = i*dt;
   %odredjivanje momenanta eksternih sila koje trebaju da savladaju zglobovi
   if t>0.2*T && t< 0.25*T
     TauF = [0;-0.4*Fint(1)];
   else
         TauF = [0;0];
   end
   
   
   %izracunavanje matrica J
   J = matrix_kin(q);
   J_ref = matrix_kin(q_ref(:,i));
   
   %izracunavanje referentnih i trenutnih vrednosti u spoljasnjim koordinatama
   [X_ref, dX_ref] = forward_kinematics(q_ref(:,i),dq_ref(:,i),J_ref);
   [X, dX] = forward_kinematics(q,dq,J);
   
   %izracunavanje matrica dinamike
   %  H, C, G matrice estimacije
   H = [2*m 0;0 m*l^2/3];
   C = [0 0;0 0];
   G = [2*m*g;0];
   
   %projketovanje kontrolera
   [Tau, Tau_FF, Tau_FB] = kontroler(q,q_ref,dq,dq_ref,ddq,ddq_ref,Kp1,Kd1,H,C,G,i,TauF); 
    
    %numericka integracija
    Q_4 = [q; dq];
    options = odeset('RelTol',1e-2,'AbsTol',1e-3,'MaxOrder',3);
    [tout,Q_4_out] = ode45(@(tout,Q_4_out) int_2DoF(t, Q_4,m,l,g,Tau,TauF),[t t+dt], Q_4, options);
    Q_4 = Q_4_out(end,:)';
    q = Q_4(1:2);
    dq = Q_4(3:4);
    ddq=H\(Tau  - C*[dq(1); dq(2)]-G-TauF);%ubrzanje za sledecu iteraciju
    
   write_in_memory;
   i = i+1;
end
for i = 1:31421
    if(mod(i,20)==0)
        figure(1)
          plot3([0 0 Ps.X(1,i)], [0 0 Ps.X(2,i)],[0 Ps.X(3,i) Ps.X(3,i)],...
               '--rs','LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
          axis equal;
          axis([0 1.2*l 0 1.2*l 0 2.2*l]);%definisanje osa bez obzira na broj koordinata  
          title('2DoF robot')
          view(135,15)
          xlabel('x position[m]')
          ylabel('y position[m]')
          zlabel('z position[m]')
          grid on
    end
end
iscrtavanje;