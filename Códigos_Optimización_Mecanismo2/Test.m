clc;clear all;close all

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

theta_min=270*pi/180;
theta_max=-125*pi/180;

theta2=200*pi/180;

t3 = pi+25*pi/180; t4 = pi+75*pi/180; t5 = pi-40*pi/180;
t6 = pi-70*pi/180; t7= pi-65*pi/180; t8= pi-150*pi/180; % tn means theta_n

x=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10,theta2,t3,t4,t5,t6,t7,t8];

[xA, xB, xC, xD, xE, xF, xG, xH,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo2(x);

global Hd 

Hd=[ -0.3943   -0.2894  -0.1195;
     -0.8108   -0.7625  -0.819];

H_path=[[-0.610144337063081;-0.744676802850388]];

plot_mecanismo2_proyecto(xA,xB,xC,xD,xE,xF,xG,xH,Hd,H_path)