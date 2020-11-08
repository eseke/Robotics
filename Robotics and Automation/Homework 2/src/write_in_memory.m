
Ps.t(i) = i*dt;                 % vrema [s]    
Ps.q(:,i) = q.*[1;180/pi];        % pozicje zglobova [dg]
Ps.dq(:,i) = dq.*[1;180/pi];      % brzine zglobova [dg/s]
Ps.ddq(:,i) = ddq;      % brzine zglobova [dg/s]
Ps.X(:,i) = X;                  % Dekartove kordinate [m]
Ps.X_ref(:,i) = X_ref;          % referenntne Dekartove koedinate [m]
Ps.dX(:,i)=dX;                  %brzine u dekartovim kordinatama
Ps.dX_ref(:,i)=dX_ref;

Ps.q_ref(:,i) = q_ref(:,i).*[1;180/pi];   % referentna pozicija zglobova [dg]
Ps.dq_ref(:,i) = dq_ref(:,i).*[1;180/pi]; % referentna brzina zglobova [dg/s]
Ps.ddq_ref(:,i) = ddq_ref(:,i); % referentna brzina zglobova [dg/s]

Ps.Tau(:,i)=Tau(:);           %  ostvareni moment koji reduktor predaje sistemu [Nm]
%Ps.Fint(:,i) = F;            % spoljasnja sila [Nm]
Ps.Tau_FF(:,i)= Tau_FF(:,1);     %  upravljacki signal-struja [A] FF komponenta
Ps.Tau_FB(:,i)= Tau_FB(:,1);     % upravljacki signal-struja [A] FB komponenta