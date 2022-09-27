% Medida Links y Ángulos
% optimizado(20%): 0.2134    0.1650    0.6600    0.6199    0.3300    0.2944    0.1980    0.2622
% optimizado(50%): 0.2411    0.1813    0.7604    0.6529    0.4181    0.3541    0.2836    0.3750

des = [0.5;0.5;0];

figure('units','normalized','outerposition',[0 0 1 1])

Lab = 0.2411;
Lbc = 0.1813;
Lad = 0.7604;
Lae = 0.6529;
Lcd = 0.4181;
Lce = 0.3541;
Ldf = 0.2836;
Lef = 0.3750;

% Lab = 0.2;
% Lbc = 0.15;
% Lad = 0.6;
% Lae = 0.6;
% Lcd = 0.3;
% Lce = 0.3;
% Ldf = 0.3;
% Lef = 0.3;
 
theta_1 = -pi/2;
alfa_1 = 0;

% Medida de ángulo mínimo y máximo para la simulación
theta_min=-135*pi/180;  %<= ERROR de 0.18%
theta_max=-46*pi/180;
% theta_min=-134*pi/180;
% theta_max=-46*pi/180;

% Tiempo total del movimiento
T_movimiento=0.85;

% Masa de los links (kg)
m1=0.307;
m2=0.237;
m3=0.913;
m4=0.787;
m5=0.513;
m6=0.439;
m7=0.463;
m8=0.356;

% Masa pierna (kg)
mP=15; 

% Momento de inercia de los links (kg*m^2)
I1=1831.34*10^-6;
I2=863.749*10^-6;
I3=46825.704*10^-6;
I4=30037.941*10^-6;
I5=8388.253*10^-6;
I6=5258.716*10^-6;
I7=6174.484*10^-6;
I8=2845.246*10^-6;

%%
% Cálculo del centro de inercia de los links 
xbar1=[Lab/2;0];
xbar2=[Lbc/2;0];
xbar3=[Lad/2;0];
xbar4=[Lae/2;0];
xbar5=[Lcd/2;0];
xbar6=[Lce/2;0];
xbar7=[Lef/2;0];
xbar8=[Ldf/2;0];

% Cálculo del peso de los links (N)
gravity=9.81;
W1=[0;-m1*gravity];
W2=[0;-m2*gravity];
W3=[0;-m3*gravity];
W4=[0;-m4*gravity];
W5=[0;-m5*gravity];
W6=[0;-m6*gravity];
W7=[0;-m7*gravity];
W8=[0;-m8*gravity];
Peso_F=mP*gravity;

Fuerza_P=[0;-Peso_F];

%%

N = 100;
% N= 1;
[t3,t4,t5,t6,t7,t8] = deal(zeros(1,N));

% initial guesses for Newton-Raphson algorithm

theta3 = -126.08*pi/180; theta4 = -74.11*pi/180; theta5 = -148.76*pi/180;
theta6 = -40.48*pi/180; theta7= -165.77; theta8= -21.78*pi/180;
 % tn means theta_n
