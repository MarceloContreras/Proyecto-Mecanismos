%% MECANISMO 2 PROYECTO

%% Inicialización de variables
clear all; close all;clc;

load_param_mech2();
load('exp_data.mat')

x_init=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];
vH=[];
H_path=[];

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
    [xA, xB, xC, xD, xE, xF, xG, xH,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo2(x_x);
    
    H_path = [H_path xH];
    t3(k) = theta3;
    t4(k) = theta4;
    t5(k) = theta5;
    t6(k) = theta6;
    t7(k) = theta7;
    t8(k) = theta8;
    
    % Simulacion
    plot_mecanismo2_proyecto(xA,xB,xC,xD,xE,xF,xG,xH,H_path);
    pause(0.01)

    % Analisis de Velocidad
    x_v=[x_init,theta2(k),theta3,theta4,theta5,theta6,theta7,theta8,w2(k)];
    [w3(k),w4(k),w5(k),w6(k),w7(k),w8(k),v_g2(:,k),v_g3(:,k),v_g4(:,k),v_g5(:,k),v_g6(:,k),v_g7(:,k),v_g8(:,k),v_H(:,k),v_H1(:,k),v_H2(:,k)] =  find_vel_mecanismo2([x_v,xbar8(1)]);
    modulo_velocidad(k)= norm(v_H(:,k));
    
    % Analisis de Aceleración
    x_a=[x_v,w3(k),w4(k),w5(k),w6(k),w7(k),w8(k),alfa_2(k),xbar8(1)];
    [alfa_3(k),alfa_4(k),alfa_5(k),alfa_6(k),alfa_7(k),alfa_8(k),a_g2(:,k),a_g3(:,k),a_g4(:,k),a_g5(:,k),a_g6(:,k),a_g7(:,k),a_g8(:,k),a_H(:,k),a_H1(:,k),a_H2(:,k)]=find_acel_aceleracion2(x_a);
    modulo_aceleracion(k)= norm(a_H(:,k));
    % a_tangencial(k)= dot(v_H(:,k),a_H(:,k))/modulo_velocidad(k);
    
    % Analisis de Fuerza
    
    % Cálculo de s
    [~,~,~,s2A,s2B,~] = LinkCG(AB,0,0,xbar2,theta2(k));
    [~,~,~,s3B,s3E,~] = LinkCG(BE,0,0,xbar3,t3(k));
    [~,~,~,s4D,s4E,~] = LinkCG(DE,0,0,xbar4,t4(k));
    [~,~,~,s5F,s5C,~] = LinkCG(CF,0,0,xbar5,t5(k));
    [~,~,~,s6F,s6B,~] = LinkCG(BF,0,0,xbar6,t6(k));
    [~,~,~,s7G,s7E,~] = LinkCG(EG,0,0,xbar7,t7(k));
    [~,~,~,s8H,s8F,~] = LinkCG(FG+GH,0,0,xbar8,t8(k));
    [~,~,~,~,s8G,~] = LinkCG(GH,0,0,xbar8,t8(k));
    
    % Matrices representativas a usar
    Z2 = [0 ,0; 0 ,0];
    U2=[1 0; 0 1];
    Z12=[0 0];
    Z21=[0;0];
    
    % Matrices y cálculo de las Fuerzas
    A_Fuerzas=[U2, U2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z21; 
               Z2, Z2, U2, Z2, Z2, Z2, U2, Z2, Z2, Z2, Z2, Z2, Z2, Z21; 
               Z2, Z2, Z2, Z2, Z2, U2, Z2, U2, Z2, Z2, Z2, Z2, Z2, Z21; 
               Z2, Z2, Z2, Z2, U2, Z2, Z2, Z2, Z2, U2, Z2, Z2, Z2, Z21; 
               Z2, Z2, Z2, U2, Z2, Z2, Z2, Z2, Z2, Z2, U2, Z2, Z2, Z21;
               Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, U2, Z2, Z2, Z2, U2, Z21;
               Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, U2, -U2, Z21;
               s2A', s2B', Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, 1;
               Z12, Z12, s3B', Z12, Z12, Z12, s3E', Z12, Z12, Z12, Z12, Z12, Z12, 0;
               Z12, Z12, Z12, Z12, Z12, s4D', Z12, s4E', Z12, Z12, Z12, Z12, Z12, 0;
               Z12, Z12, Z12, Z12, s5C', Z12, Z12, Z12, Z12, s5F', Z12, Z12, Z12, 0;
               Z12, Z12, Z12, s6B', Z12, Z12, Z12, Z12, Z12, Z12, s6F', Z12, Z12, 0;
               Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, s7E', Z12, Z12, Z12, s7G', 0;
               Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, Z12, s8F', -s8G', 0;
               Z2, U2, U2, U2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z21;
               Z2, Z2, Z2, Z2, Z2, Z2, U2, U2, U2, Z2, Z2, Z2, Z2, Z21;
               Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, Z2, U2, U2, U2, Z2, Z21];
           
    B_Fuerzas=[m2*a_g2(:,k) - W2; m3*a_g3(:,k) - W3; m4*a_g4(:,k) - W4; 
               m5*a_g5(:,k) - W5; m6*a_g6(:,k) - W6; m7*a_g7(:,k) - W7; m8*a_g8(:,k) - W8 - Fuerza_H;
               I2*alfa_2(k); I3*alfa_3(k); I4*alfa_4(k); I5*alfa_5(k);
               I6*alfa_6(k); I7*alfa_7(k); I8*alfa_8(k) - dot(s8H,Fuerza_H); Z21; Z21; Z21];
           
    C_Fuerzas=A_Fuerzas\B_Fuerzas;
    
    % Asignación valores obtenidos de las Fuerzas
    Fuerza_G(1,k)=C_Fuerzas(25);
    Fuerza_G(2,k)=C_Fuerzas(26);
    T3(k)=C_Fuerzas(27);
    
    
    P2 = InertialPower(m2,I2,v_g2(:,k),a_g2(:,k), w2(k), alfa_2(k));
    P3 = InertialPower(m3,I3,v_g3(:,k),a_g3(:,k), w3(k), alfa_3(k));
    P4 = InertialPower(m4,I4,v_g4(:,k),a_g4(:,k), w4(k), alfa_4(k));
    P5 = InertialPower(m5,I5,v_g5(:,k),a_g5(:,k), w5(k), alfa_5(k));
    P6 = InertialPower(m6,I6,v_g6(:,k),a_g6(:,k), w6(k), alfa_6(k));
    P7 = InertialPower(m7,I7,v_g7(:,k),a_g7(:,k), w7(k), alfa_7(k));
    P8 = InertialPower(m8,I8,v_g8(:,k),a_g8(:,k), w8(k), alfa_8(k));
    PKin(k) = P2 + P3 + P4 + P5 + P6 + P7 + P8; % Inertial power
    PF = dot(Fuerza_H,v_H(:,k)); % Power from external force FP
    PW2 = dot(W2,v_g2(:,k)); % Power from Weight 2
    PW3 = dot(W3,v_g3(:,k)); % Power from Weight 3
    PW4 = dot(W4,v_g4(:,k)); % Power from Weight 4
    PW5 = dot(W5,v_g5(:,k)); % Power from Weight 5
    PW6 = dot(W6,v_g6(:,k)); % Power from Weight 6
    PW7 = dot(W7,v_g7(:,k)); % Power from Weight 7
    PW8 = dot(W8,v_g8(:,k)); % Power from Weight 8
    PT = T3(k) * w2(k); % Power from driving torque T2
    PExt(k) = PF + PW2 + PW3+ PW4 + PW5 + PW6 + PW7 + PW8 + PT; % Total external power
end

%% PLOTS
 
% PLOTEOS POSICIÓN

figure(2) % Plot angulo tht_8
plot(t,t8), grid, xlim([0 T_movimiento])
title('\theta_{8} vs  Time'), xlabel('Time (s)'), ylabel('Position (rad)')

figure(3) % Plot pos h_y
plot(t,H_path(2,:)-H_path(2,1)), grid, xlim([0 T_movimiento])
title('H_{y} vs  \theta_{2}'), xlabel('Position (rad)'), ylabel('Position (m)')

figure(4) % Plot pos h_x
plot(t,H_path(1,:)-H_path(1,1)), grid, xlim([0 T_movimiento])
title('H_{x} vs \theta_{2}'), xlabel('Time (s)'), ylabel('Position (m)')

%  PLOTEOS VELOCIDAD

figure(5) % Plot vel_y
plot(t,v_H(2,:)), grid, xlim([0 T_movimiento])
title('vH_{y} vs Time'), xlabel('Time (s)'), ylabel('Velocity (m/s)')

figure(6) % Plot vel_x
plot(t,v_H(1,:)), grid, xlim([0 T_movimiento])
title('vH_{x} vs Time'), xlabel('Time (s)'), ylabel('Velocity (m/s)')

figure(7) % Plot w_8
plot(t,w8), grid, xlim([0 T_movimiento])
title('\omega_{8} vs Time'), xlabel('Time (s)'), ylabel('Angular velocity (rad/s)')

figure(8) % Plot |v|
plot(t,sqrt(v_H(1,:).^2 + v_H(2,:).^2)), grid, xlim([0 T_movimiento])
title('Velocity module vs Time'), xlabel('Time (s)'), ylabel('Velocity (m/s)')
hold on
plot(t,sqrt(v_H(1,:).^2 + v_H(2,:).^2))
plot(Libro1FINAL1(:,1),Libro1FINAL1(:,4)/100)
hold off
legend('Mech 1','Mech 2','Experimental data')

%%  PLOTEOS ACELERACIÓN

figure(9) % Plot acc_x
plot(t,a_H(1,:),'g'), grid, xlim([0 T_movimiento])
title('acc_{x} vs  Time'), xlabel('Time (s)'), ylabel('Acceleration (m/s^2)')

figure(10) % Plot acc_y
plot(t,a_H(2,:),'g'), grid, xlim([0 T_movimiento])
title('acc_{y} vs  Time'), xlabel('Time (s)'), ylabel('Acceleration (m/s^2)')

figure(11) % Plot alpha_8
plot(t,alfa_8,'g'), grid, xlim([0 T_movimiento])
title('\alpha_{8} vs Time'), xlabel('Time (s)'), ylabel('Acceleration (rad/s^2)')


%  PLOTEOS FUERZAS

figure(12)
plot(t,T2), grid, xlim([0 T_movimiento])
title('\tau vs Time'), xlabel('Time (s)'), ylabel('Torque (N.m)')
hold on
plot(t,T3)
hold off
legend('Mech 1','Mech 2')

% PLOTEO DE ENERGIA

figure(13)
%Ploteo de energía externa y cinética
plot(t,PKin,'o','Color',[153/255 153/255 153/255]), grid, xlim([0 T_movimiento])
hold on
plot(t,PExt,'Color',[0 110/255 199/255],'LineWidth',2)
title('External vs. Kinetic Power (Mecanismo 2)')
xlabel('Time (s)'), ylabel('Power (W)'), legend('Kinetic','External')
 
% PLOTEOS CRANK

figure(14)
plot(t,theta2*180/pi), grid, xlim([0 T_movimiento])
title('\theta_{2} vs Time'), xlabel('Time (s)'), ylabel('Position (°)')

figure(15)
plot(t,w2), grid, xlim([0 T_movimiento])
title('\omega_{2} vs Time'), xlabel('Time (s)'), ylabel('Velocity (rad/s)')

figure(16)
plot(t,alfa_2), grid, xlim([0 T_movimiento])
title('\alpha_{2} vs Time'), xlabel('Time (s)'), ylabel('Acceleration (rad/s^2)')

 
 
 