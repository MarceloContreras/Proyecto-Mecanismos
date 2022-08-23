%% MECANISMO 1 PROYECTO

%% Inicialización de variables
clear all; close all;clc;

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

x_init=[Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef,theta_1];
vH=[];
A_path=[];

t=linspace(0,T_movimiento,N); % k valores equidistantes entre 0 y 0.75
T=T_movimiento;
Amplitud_w2=(theta_max-theta_min)/T;

for k=1:N
    theta2(k)=Amplitud_w2*t(k) - Amplitud_w2*T/(2*pi)*sin(2*pi*t(k)/T)+theta_min;
    w2(k)=Amplitud_w2*(1-cos(2*pi*t(k)/T));
    alfa_2(k)=Amplitud_w2*2*pi/T*sin(2*pi*t(k)/T);
%     if k==1 | k==N
%         Fuerza_H=[0;0];
%     end
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
    
    % Simulacion
    plot_mecanismo1_proyecto(xA,xB,xC,xD,xE,xF,A_path);
    pause(0.01)

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
    
    % Matrices representativas a usar
    Z2 = [0 ,0; 0 ,0];
    U2=[1 0; 0 1];
    Z12=[0 0];
    Z21=[0;0];

    % Matrices y cálculo de las Fuerzas
    
    % X = [F8F, F8D, F7F, F7E, F6E, F6C, F5D, F5C, F4E, F4A, F3D, F3A, F2C, FB, F1A, FF, T2, T1]
             
    A_Fuerzas = [U2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2, Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,U2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2, Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,U2,U2,Z2,Z2,Z2,Z2,Z2,Z2,Z2, Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,U2,U2,Z2,Z2,Z2,Z2,Z2, Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,U2,U2,Z2,Z2,Z2, Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,U2,U2,Z2, Z2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,U2, U2,Z2,Z2,Z21,Z21;
                 Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,Z2,-U2,U2,Z2,Z21,Z21;
                 s8F',s8D', Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12, Z12,  s7F',s7E', Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12, Z12,  Z12, Z12,  s6E',s6C', Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12, Z12,  Z12, Z12,  Z12, Z12,  s5D',s5C', Z12, Z12,  Z12, Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  s4E',s4A', Z12, Z12,  Z12, Z12, Z12, Z12,0,0;
                 Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  s3D',s3A', Z12, Z12, Z12, Z12,0,0;
                 Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  s2C',s2B',Z12, Z12,1,0;
                 Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12, Z12,  Z12,-s1B',s1A',Z12,0,1;
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
    
    T2(k)=C_Fuerzas(33);
    T1(k)=C_Fuerzas(34);
end

%% INICIO PLOTEOS
 
% %% PLOTEOS POSICIÓN
% figure(2)
%  plot(t,t8)
%  title('Theta8 vs  Tiempo')
%  xlabel('Time (s)')
%  ylabel('Possition (rad/s)')
%  xlim([0 T_movimiento])
%  grid
%  
%  figure(3)
%  plot((theta_min-theta2)*180/pi,A_path(2,:))
%  title('A_Y vs  Theta2')
%  xlabel('Time (s)')
%  ylabel('Possition (m)')
%  xlim(flip([0 -theta_max*180/pi+theta_min*180/pi]))
%  grid
%  
%  figure(4)
%  plot((theta_min-theta2)*180/pi,A_path(1,:)-A_path(1,1))
%  title('A_X vs Theta2')
%  xlabel('Time (s)')
%  ylabel('Possition (m)')
%  xlim(flip([0 -theta_max*180/pi+theta_min*180/pi]))
%  grid
%  
%  %%  PLOTEOS VELOCIDAD
%  
%  figure(5)
%  plot(t,vA(2,:))
%  title('vA_Y vs  Tiempo')
%  xlabel('Time (s)')
%  ylabel('Velocity (m/s)')
%  xlim([0 T_movimiento])
%  grid
%  
%  figure(6)
%  plot(t,vA(1,:))
%  title('vA_X vs  Tiempo')
%  xlabel('Time (s)')
%  ylabel('Velocity (m/s)')
%  xlim([0 T_movimiento])
%  grid
% 
%  figure(7)
%  plot(t,w8)
%  title('w8 vs  Tiempo')
%  xlabel('Time (s)')
%  ylabel('Angular velocity (rad/s)')
%  xlim([0 T_movimiento])
%  grid
%  
%   figure(8)
%  plot(t,modulo_velocidad)
%  title('Modulo Velocidad vs  Tiempo')
%  xlabel('Time (s)')
%  ylabel('Velocity (m/s)')
%  xlim([0 T_movimiento])
%  grid
%  
%   %%  PLOTEOS ACELERACIÓN
%  
%  figure(9)
%  plot(t,aA(1,:),'r')
%  title('Aceleracion X vs  Tiempo')
%  xlabel('Time (s)')
%  ylabel('Velocity (m/s)')
%  xlim([0 T_movimiento])
%  grid
%  
%   figure(10)
%  plot(t,aA(2,:))
%  title('Aceleracion Y vs  Tiempo')
%  xlabel('Time (s)')
%  ylabel('Velocity (m/s)')
%  xlim([0 T_movimiento])
%  grid
%  
%    figure(11)
%  plot(t,alfa_8)
%  title('Alfa 8 vs Tiempo')
%  xlabel('Time (s)')
%  ylabel('Acceleration (rad/s^2)')
%  xlim([0 T_movimiento])
%  grid
%  
%   %%  PLOTEOS FUERZAS
%   
%    figure(12)
%  plot(t,T2)
%  title('Torque vs Tiempo')
%  xlabel('Time (s)')
%  ylabel('Torque (N.m)')
%  xlim([0 T_movimiento])
%  grid
%  

for m=1:N
    acel_modulo(m) = norm(aA(:,m));

end



    