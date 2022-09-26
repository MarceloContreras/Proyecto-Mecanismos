% Dimensiones
Lab = 0.2;
Lbc = 0.15;
Lad = 0.6;
Lae = 0.6;
Lcd = 0.3;
Lce = 0.3;
Ldf = 0.22;
Lef = 0.25;
    
% AB,BC,AD,AE,CE,DF,EF

k = 200; % Num de pasos

% Parametros iniciales
omega2 = 2;
omega1 = 0;
theta1 = -pi/2;
alpha2 = 0;
alpha1 = 0;

T1 = linspace(-120,-56,k);

des = [0.5;0.5;0]; % Pos de end-effector deseada
        
% Valor inicial en deg
theta3 = -121.7*pi/180; theta4 = -84.39*pi/180; theta5 = -142.81*pi/180;
theta6 = -63.28*pi/180; theta7= -160.3*pi/180; theta8= -50.98*pi/180; 
