%% MECANISMO 1 PROYECTO

%% Inicialización de variables
clear all; close all;clc;

% Medida Links y Ángulos
% optimizado(20%): 0.2134    0.1650    0.6600    0.6199    0.3300    0.2944    0.1980    0.2622
% optimizado(50%): 0.2411    0.1813    0.7604    0.6529    0.4181    0.3541    0.2836    0.3750

Lab = 0.2411;
Lbc = 0.1813;
Lad = 0.7604;
Lae = 0.6529;
Lcd = 0.4181;
Lce = 0.3541;
Ldf = 0.2836;
Lef = 0.3750;

% Valores inicales:
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
T_movimiento=1.05;

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
gravity=0;
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

x_init=[Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef,theta_1];
vH=[];
A_path=[];

t=linspace(0,T_movimiento,N); % k valores equidistantes entre 0 y 0.75
T=T_movimiento;
Amplitud_w2=(theta_max-theta_min)/T;

% Matrices representativas a usar
Z2 = [0 ,0; 0 ,0];
U2=[1 0; 0 1];
Z12=[0 0];
Z21=[0;0];

for k=1:N
    theta2(k)=Amplitud_w2*t(k) - Amplitud_w2*T/(2*pi)*sin(2*pi*t(k)/T)+theta_min;
    w2(k)=Amplitud_w2*(1-cos(2*pi*t(k)/T));
    alfa_2(k)=Amplitud_w2*2*pi/T*sin(2*pi*t(k)/T);

    % Analisis de Posición
    x_x=[x_init,theta2(k),theta3,theta4,theta5,theta6,theta7,theta8];
    [xA, xB, xC, xD, xE, xF,x_g1,x_g2,x_g3,x_g4,x_g5,x_g6,x_g7,x_g8,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo1(x_x);
    
    A_path = [A_path xA];
    t3(k) = theta3;
    t4(k) = theta4;
    t5(k) = theta5;
    t6(k) = theta6;
    t7(k) = theta7;
    t8(k) = theta8;

    % Analisis de Velocidad
    x_v=[x_init,theta2(k),theta3,theta4,theta5,theta6,theta7,theta8,w2(k)];
    [w3(k),w4(k),w5(k),w6(k),w7(k),w8(k),v_g1(:,k),v_g2(:,k),v_g3(:,k),v_g4(:,k),v_g5(:,k),v_g6(:,k),v_g7(:,k),v_g8(:,k),~,~,~,~,vA(:,k)] =  find_vel_mecanismo1([x_v]);
    modulo_velocidad(k)= norm(vA(:,k));
    
    % Analisis de Aceleración
    x_a=[x_v,w3(k),w4(k),w5(k),w6(k),w7(k),w8(k),alfa_2(k)];
    [alfa_3(k),alfa_4(k),alfa_5(k),alfa_6(k),alfa_7(k),alfa_8(k),a_g1(:,k),a_g2(:,k),a_g3(:,k),a_g4(:,k),a_g5(:,k),a_g6(:,k),a_g7(:,k),a_g8(:,k),aA(:,k)]=find_acel_mecanismo1(x_a);
    
    % a_tangencial(k)= dot(v_H(:,k),a_H(:,k))/modulo_velocidad(k);
    
    % Analisis de Fuerza
    
    % Cálculo de s
    [~,~,~,s1B,s1A,~] = LinkCG(Lab,0,0,xbar1,-pi/2);
    [~,~,~,s2C,s2B,~] = LinkCG(Lbc,0,0,xbar2,theta2(k));
    [~,~,~,s3D,s3A,~] = LinkCG(Lad,0,0,xbar3,t3(k));
    [~,~,~,s4E,s4A,~] = LinkCG(Lae,0,0,xbar4,t4(k));
    [~,~,~,s5D,s5C,~] = LinkCG(Lcd,0,0,xbar5,t5(k));
    [~,~,~,s6E,s6C,~] = LinkCG(Lce,0,0,xbar6,t6(k));
    [~,~,~,s7F,s7E,~] = LinkCG(Lef,0,0,xbar7,t7(k));
    [~,~,~,s8F,s8D,~] = LinkCG(Ldf,0,0,xbar8,t8(k));

    % Matrices y cálculo de las Fuerzas
    
    % X = [F8F, F8D, F7F, F7E, F6E, F6C, F5D, F5C, F4E, F4A, F3D, F3A, F2C, FB, F1A, FF, T2, T1]
             
    % Matrices representativas a usar
Z2 = [0 ,0; 0 ,0];
U2=[1 0; 0 1];
Z12=[0 0];
Z21=[0;0];
    
        A_Fuerzas = [U2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,U2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,U2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,U2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,U2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,U2,U2,Z2,Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,U2,U2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,-U2,U2,Z2,Z21,Z21;
                 s8F',s8D',Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12,Z12,  s7F',s7E',Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12,Z12,  Z12,Z12,  s6E',s6C',Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12,Z12,  Z12,Z12,  Z12,Z12,  s5D',s5C',Z12,Z12,  Z12,Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  s4E',s4A',Z12,Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  s3D',s3A',Z12, Z12, Z12, Z12,0,0;
                 Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  s2C',s2B',Z12, Z12,1,0;
                 Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,Z12,  Z12,-s1B',s1A',Z12,0,1;
                 U2,Z2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,U2,Z21,Z21;
                 Z2,Z2,Z2,U2,U2,Z2,Z2,Z2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z21,Z21;
                 Z2,U2,Z2,Z2,Z2,Z2,U2,Z2,Z2,Z2,U2,Z2,Z2,Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,U2,Z2,U2,Z2,Z2,Z2,Z2,U2,Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,U2,Z2,U2,Z2,Z2,U2,Z2,Z21,Z21];
             
    B_Fuerzas = [m8*a_g8(:,k)-W8;
                 m7*a_g7(:,k)-W7;
                 m6*a_g6(:,k)-W6;
                 m5*a_g5(:,k)-W5;
                 m4*a_g4(:,k)-W4;
                 m3*a_g3(:,k)-W3;
                 m2*a_g2(:,k)-W2;
                 m1*a_g1(:,k)-W1;
                 I8*alfa_8(k);
                 I7*alfa_7(k);
                 I6*alfa_6(k);
                 I5*alfa_5(k);
                 I4*alfa_4(k);
                 I3*alfa_3(k);
                 I2*alfa_2(k);
                 I1*alfa_1;
                 Z21;
                 Z21;
                 Z21;
                 Z21;
                 -Fuerza_P];
           
    C_Fuerzas=pinv(A_Fuerzas)*B_Fuerzas;
        
    % Asignación valores obtenidos de las Fuerzas
    T2(k)=C_Fuerzas(33);
    T1(k)=C_Fuerzas(34);

   %% 
    P1 = InertialPower(m1,I1,v_g1(:,k),a_g1(:,k), 0, alfa_1);
    P2 = InertialPower(m2,I2,v_g2(:,k),a_g2(:,k), w2(k), alfa_2(k));
    P3 = InertialPower(m3,I3,v_g3(:,k),a_g3(:,k), w3(k), alfa_3(k));
    P4 = InertialPower(m4,I4,v_g4(:,k),a_g4(:,k), w4(k), alfa_4(k));
    P5 = InertialPower(m5,I5,v_g5(:,k),a_g5(:,k), w5(k), alfa_5(k));
    P6 = InertialPower(m6,I6,v_g6(:,k),a_g6(:,k), w6(k), alfa_6(k));
    P7 = InertialPower(m7,I7,v_g7(:,k),a_g7(:,k), w7(k), alfa_7(k));
    P8 = InertialPower(m8,I8,v_g8(:,k),a_g8(:,k), w8(k), alfa_8(k));
    
    U1 = m1*gravity*v_g1(2,k);
    U2 = m2*gravity*v_g2(2,k);
    U3 = m3*gravity*v_g3(2,k);
    U4 = m4*gravity*v_g4(2,k);
    U5 = m5*gravity*v_g5(2,k);
    U6 = m6*gravity*v_g6(2,k);
    U7 = m7*gravity*v_g7(2,k);
    U8 = m8*gravity*v_g8(2,k);
    
    PKin(k) = P1 + P2 + P3 + P4 + P5 + P6 + P7 + P8 - 8.5*(U1 + U2 + U3 + U4 + U5 + U6 + U7 + U8); % Inertial power
    
    PF = dot(Fuerza_P,vA(:,k)); % Power from external force FP
    PW1 = dot(W1,v_g1(:,k)); % Power from Weight 1
    PW2 = dot(W2,v_g2(:,k)); % Power from Weight 2
    PW3 = dot(W3,v_g3(:,k)); % Power from Weight 3
    PW4 = dot(W4,v_g4(:,k)); % Power from Weight 4
    PW5 = dot(W5,v_g5(:,k)); % Power from Weight 5
    PW6 = dot(W6,v_g6(:,k)); % Power from Weight 6
    PW7 = dot(W7,v_g7(:,k)); % Power from Weight 7
    PW8 = dot(W8,v_g8(:,k)); % Power from Weight 8
    PT = T2(k) * w2(k); % Power from driving torque T2
    PT1 = T1(k) * 0; % Power from reaction torque T1
    PExt(k) = PF + PW1 + PW2 + PW3+ PW4 + PW5 + PW6 + PW7 + PW8 + PT + PT1; % Total external power
end

%% INICIO PLOTEOS

 %Ploteo de energía externa y cinética
    plot(t,PKin,'o','Color',[153/255 153/255 153/255])
    hold on
    plot(t,PExt,'Color',[0 110/255 199/255],'LineWidth',2)
    title('External vs. Kinetic Power')
    xlabel('Tiempo (s)')
    ylabel('Power (W)')
    legend('Kinetic','External')
    grid on
    xlim([0 T_movimiento])