% Medida Links y Ángulos
AB = 0.130;
BE = 0.420;     % crank length (cm)
DE = 0.330;     % 
AD = 0.350;    % 
EG = 0.430;    % 
FG = 0.430;    % 
BF = 0.430;     %
CF = 0.390;
CD = 0.330;
GH = 0.230;
 
theta_10= pi-60*pi/180;
theta_1 = 0;

% Medida de ángulo mínimo y máximo para la simulación
theta_min=244.825*pi/180;
theta_max=150*pi/180;

% Tiempo total del movimiento
T_movimiento=0.85;

% Masa de los links (kg)
m2=0.171;
m3=0.546;
m4=0.431;
m5=0.467;
m6=0.537;
m7=0.536;
m8=0.757;

% Masa pierna (kg)
mP=15; 

% Momento de inercia de los links (kg*m^2)
I2=337.288*10^-6;
I3=10085.406*10^-6;
I4=5005.719*10^-6;
I5=6328.187*10^-6;
I6=9587.042*10^-6;
I7=9525.159*10^-6;
I8=27095.132*10^-6;

%%
% Cálculo del centro de inercia de los links 
xbar2=[AB/2;0];
xbar3=[BE/2;0];
xbar4=[DE/2;0];
xbar5=[CF/2;0];
xbar6=[BF/2;0];
xbar7=[EG/2;0];
xbar8=[FG+GH-31.4677/100;0];

% Cálculo del peso de los links (N)
gravity=9.81;
W2=[0;-m2*gravity];
W3=[0;-m3*gravity];
W4=[0;-m4*gravity];
W5=[0;-m5*gravity];
W6=[0;-m6*gravity];
W7=[0;-m7*gravity];
W8=[0;-m8*gravity];
Peso_F=mP*gravity;

Fuerza_H=[0;-Peso_F];

%%

N = 100;
% N= 1;
[t3,t4,t5,t6,t7,t8] = deal(zeros(1,N));

% initial guesses for Newton-Raphson algorithm
theta3 = pi+25*pi/180; theta4 = pi+75*pi/180; theta5 = pi-40*pi/180;
theta6 = pi-70*pi/180; theta7= pi-65*pi/180; theta8= pi-150*pi/180; % tn means theta_n