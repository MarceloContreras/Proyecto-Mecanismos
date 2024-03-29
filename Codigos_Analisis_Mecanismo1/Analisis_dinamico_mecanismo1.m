%% MECANISMO 1 PROYECTO

%% Inicialización de variables
clear all; close all;clc;

load_param_mech1();

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

%% PLOTS

% PLOTS DE POSICION

% figure(2) % Plot angulo
% plot(t,t8), grid, xlim([0 T_movimiento])
% title('\theta_{8} vs Time'), xlabel('Time (s)'), ylabel('Position (rad)')
% 
% figure(3) % Plot pos_x vs ang
% plot((theta_min-theta2)*180/pi,A_path(2,:)), grid, xlim(flip([0 -theta_max*180/pi+theta_min*180/pi]))
% title('a_{Y} vs \theta_{8}'), xlabel('Position (rad)'), ylabel('Position (m)')
% 
% figure(4) % Plot pos_y vs ang
% plot((theta_min-theta2)*180/pi,A_path(1,:)-A_path(1,1)), grid, xlim(flip([0 -theta_max*180/pi+theta_min*180/pi]))
% title('a_{X} vs \theta_{8}'), xlabel('Position (rad)'), ylabel('Position (m)')
%  
% % PLOTS DE VELOCIDAD
% 
% figure(5) % Plot vel_x
% plot(t,vA(1,:)), grid, xlim([0 T_movimiento])
% title('vA_{X} vs Time'), xlabel('Time (s)'), ylabel('Velocity (m/s)')
%  
% figure(6) % Plot vel_y
% plot(t,vA(2,:)), grid, xlim([0 T_movimiento])
% title('vA_Y vs  Time'), xlabel('Time (s)'), ylabel('Velocity (m/s)')
%  
% figure(7) % Plot ang_vel_8
% plot(t,w8), grid, xlim([0 T_movimiento])
% title('\omega_{8} vs  Time'), xlabel('Time (s)'), ylabel('Angular velocity (rad/s)')
% 
% figure(8)
% plot(t,modulo_velocidad), grid, xlim([0 T_movimiento])
% title('Velocity module vs  Time'), xlabel('Time (s)'), ylabel('Velocity (m/s)')
%  
% %  PLOTEOS ACELERACIÓN
% 
% figure(9)
% plot(t,aA(1,:),'r'), grid, xlim([0 T_movimiento])
% title('a_{x} vs  Time'), xlabel('Time (s)'), ylabel('Acceleration (m/s^2)')
%  
% figure(10)
% plot(t,aA(2,:)), grid, xlim([0 T_movimiento])
% title('a_{y} vs  Time'), xlabel('Time (s)'), ylabel('Acceleration (m/s^2)')
%  
% figure(11)
% plot(t,alfa_8), grid, xlim([0 T_movimiento])
% title('\alpha_{8} vs Time'), xlabel('Time (s)'), ylabel('Acceleration (rad/s^2)')
%  
% %  PLOTEOS FUERZAS
%   
% figure(12)
% plot(t,T2), grid, xlim([0 T_movimiento])
% title('\tau vs Time'), xlabel('Time (s)'), ylabel('Torque (N.m)')


%%

for m=1:N
    acel_modulo(m) = norm(aA(:,m));
end



    