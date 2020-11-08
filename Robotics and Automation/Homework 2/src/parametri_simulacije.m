global m1 m2 I1 I2 l1 l2 g Fint

t = 0;          % inicijalizacija vremena [s]
dt = 0.001;     % korak simulacije [s]
T = 31.42;          % vreme simulacije [s]
lengthT = T/dt; % broj tacaka simulacije
i = 1;          % brojac
g = 9.81;       % g
Fint = [2; 1];  % spoljasnja sila [N]


%% Parametri segmenata
l1 = 0.5;       % [m] duzina segmenta 1
l2 = 0.5;       % [m] duzina segmenta 2
m1 = 2;         % [kg] masa segmenta 1
m2 = 2;         % [kg] masa segmenta 2
I1 = m1*l1^2/3;      % [kgm^2] moment inercije segmenta 1
I2 = m2*l2^2/3;      % [kgm^2] moment inercije segmenta 1
l = l1;
%% pocetno stanje
q = [0.0; 0*pi/180]; % pozicija
dq = [0; 0];                % brzina
ddq = [0; 0];               % ubrzanje


