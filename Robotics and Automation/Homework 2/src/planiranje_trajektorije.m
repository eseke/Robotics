%planiranje trajektorije robota 

%parametri kretanja
A = [0; 0*pi/180];
B = [0.3; 45*pi/180];
C = [0.5; 90*pi/180];
Ta = 0;
Tb = 7.864;
Tc = T;

Tab_acc = 0.01;
ddqab_max = [3.82;10];%*pi/180];

k = length(0:dt:T);
ddq_ref = zeros(2,k);
dq_ref = zeros(2,k);
q_ref = zeros(2,k);
lengthT = T/dt;
for i = 1:lengthT+1
    if i<=Tab_acc/dt
        ddq_ref(:,i) = ddqab_max;
        dq_ref(:,i) = ddqab_max.*(i-1)*dt;
        q_ref(:,i) = A + 0.5.*ddq_ref(:,i)*((i-1)*dt)^2;
    
    elseif i<=(Tb-Tab_acc)/dt
        ddq_ref(:,i) = 0;
        dq_ref(:,i) = ddqab_max.*Tab_acc;
        q_ref(:,i) = A + ddqab_max*Tab_acc.*((i-1)*dt-0.5*Tab_acc);
    
    elseif i<=Tb/dt
        ddq_ref(:,i) = -ddqab_max;
        dq_ref(:,i) = ddqab_max.*(Tb-(i-1)*dt);
        q_ref(:,i) = B - 0.5*ddqab_max.*(Tb-(i-1)*dt)^2;
    else
        t1 = (i/(lengthT+1)-Tb/dt/(lengthT+1))/(T-Tb)*T;
        q_sk = (-2*t1^3+3*t1^2);
        dq_sk = -6*t1^2+6*t1;
        ddq_sk = -12*t1+6;
        
        q_ref(:,i) = B+(C-B)*q_sk;
        dq_ref(:,i) = (C-B)*dq_sk/(Tc-Tb);
        ddq_ref(:,i) = (C-B)*ddq_sk/(Tc-Tb)^2;
    end   
end

q1_ref=q_ref(1,:);
q2_ref=q_ref(2,:);
dq1_ref=dq_ref(1,:);
dq2_ref=dq_ref(2,:);
ddq1_ref=ddq_ref(1,:);
ddq2_ref=ddq_ref(2,:);

