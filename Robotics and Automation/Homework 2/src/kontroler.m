function [Tau, Tau_FF, Tau_FB] = kontroler(q, q_ref,dq,dq_ref,ddq, ddq_ref, Kp1, Kd1, H, C, G, i,TauF)

for j=1:2
   % feedforward  - computed torque metoda
   Tau_FF(j,1) = G(j,1)+C(j,1)*dq_ref(j,i)+H(j,j)*ddq_ref(j,i);
   
   % feedback PD control
   Tau_FB(j,1) = Kp1(j)*(q_ref(j,i)-q(j)) + Kd1(j)*(dq_ref(j,i)-dq(j));
   
   Tau(j,1) = Tau_FF(j,1) + Tau_FB(j,1);
end
end