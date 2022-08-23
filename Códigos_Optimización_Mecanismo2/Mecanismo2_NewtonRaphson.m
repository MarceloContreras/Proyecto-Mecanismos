%% MECANISMO 2 PROYECTO
%% Newton Raphson Method

%% Inicialización de variables
clear all; close all;clc;

AB = 130;
BE = 420;     % crank length (cm)
DE = 330;     % 
AD = 350;    % 
EG = 430;    % 
FG = 430;    % 
BF = 430;     %
CF = 390;
CD = 330;
GH = 230;
 
theta_10= pi-60*pi/180;
theta_1 = 0;

Hd=[ -0.3943   -0.2894  -0.1195;
     -0.8108   -0.7625  -0.819];

theta_min=270*pi/180;
theta_max=125*pi/180;
%%

N = 46;
% N= 1;
[t3,t4,t5,t6,t7,t8] = deal(zeros(1,N));

% initial guesses for Newton-Raphson algorithm
theta3 = pi+25*pi/180; theta4 = pi+75*pi/180; theta5 = pi-40*pi/180;
theta6 = pi-70*pi/180; theta7= pi-65*pi/180; theta8= pi-150*pi/180; % tn means theta_n

x_init=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];
vH=[];
H_path=[];

theta2=linspace(theta_min,theta_max,N); % k valores equidistantes entre 0 y 2pi

for k=1:N
    theta2(k);
     w2(k)=-5*(1-cos(2*pi*(k-1)/(N-1)));
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
    plot_mecanismo2_proyecto(xA,xB,xC,xD,xE,xF,xG,xH,Hd,H_path);
    pause(0.01)

    % Analisis de Velocidad
%     x_v=[x_init,theta2(k),theta3,theta4,theta5,theta6,theta7,theta8,w2(k)];
%     [w3(k),w4(k),w5(k),w6(k),w7(k),w8(k),v__H] =  find_vel_mecanismo2(x_v);
%     vH=[vH,v__H];
end

% figure(2)
%  plot(270-theta2*180/pi,H_path(1,:)-H_path(1,1))
%  title('xH_X vs Theta2')
%  xlabel('Crank angle (°)')
%  ylabel('Possition (m)')
%  xlim([0 145])
%  grid
%  
%  figure(3)
%  plot(270-theta2*180/pi,H_path(2,:)-H_path(2,1))
%  title('xH_Y vs Theta2')
%  xlabel('Crank angle (°)')
%  ylabel('Possition (m)')
%  xlim([0 145])
%  grid
%  
% figure(4)
%  plot(270-theta2*180/pi,vH(2,:))
%  title('vH_Y vs Theta2')
%  xlabel('Crank angle (°)')
%  ylabel('Velocity (m/s)')
%  xlim([0 145])
%  grid
%  
%  figure(5)
%  plot(270-theta2*180/pi,vH(1,:))
%  title('vH_X vs Theta2')
%  xlabel('Crank angle (°)')
%  ylabel('Velocity (m/s)')
%  xlim([0 145])
%  grid
% 
%  figure(6)
%  plot(270-theta2*180/pi,w8)
%  title('w8 vs Theta2')
%  xlabel('Crank angle (°)')
%  ylabel('Angular velocity (rad/s)')
%  xlim([0 145])
%  grid
