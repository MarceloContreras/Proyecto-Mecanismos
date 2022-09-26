%% Eight Bar Linkage : Kevin Optimizacion 
% CURSO MT-0001 Mechanisms and movement transmition
% Profesor: Elvis Jara
% UTEC

clc;clear all;close all
%% Valores iniciales

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

theta3 = pi+25*pi/180; theta4 = pi+75*pi/180; theta5 = pi-40*pi/180;
theta6 = pi-70*pi/180; theta7= pi-65*pi/180; theta8= pi-150*pi/180; % tn means theta_n

%% Tarjet - 3,5,10,13 puntos deseados para H
global Hd th2d
% 
% Hd=[ -0.3943   -0.2894  -0.1195;
%      -0.8108   -0.7625  -0.819];
% 
% th2d=[231.76*pi/180     196.93*pi/180   162.99*pi/180];

% Hd=[ -0.3943   -0.3568 -0.2894 -0.1874 -0.1195;
%      -0.8108   -0.7693 -0.7625 -0.7958 -0.819];
% 
% th2d=[231.76*pi/180   218.23*pi/180  196.93*pi/180 175.81*pi/180 162.99*pi/180];


% Hd=[ -0.3943  -0.3568 -0.3228 -0.2894 -0.2573 -0.2213 -0.1874 -0.1533 -0.1195 -0.0621;
%      -0.8108  -0.7693 -0.7626 -0.7625 -0.7756 -0.7841 -0.7958 -0.8057 -0.819  -0.8296];
% 
% th2d=[231.76*pi/180 218.23*pi/180 206.91*pi/180 196.93*pi/180 188.72*pi/180 182.24*pi/180 175.81*pi/180 170*pi/180 162.99*pi/180 150.53*pi/180];

Hd=[-0.4093 -0.3943 -0.3844 -0.3714 -0.3568 -0.3228 -0.2894 -0.2573 -0.2213 -0.1874 -0.1533 -0.1195 -0.0621;
     -0.8277 -0.8108 -0.7951 -0.7813 -0.7693 -0.7626 -0.7625 -0.7756 -0.7841 -0.7958 -0.8057 -0.819  -0.8296];

th2d=[ 237.08*pi/180 231.76*pi/180 227.14*pi/180 222.47*pi/180 218.23*pi/180 206.91*pi/180 196.93*pi/180 188.72*pi/180 182.24*pi/180 175.81*pi/180 170*pi/180 162.99*pi/180 150.53*pi/180];
%% Condición de Grashof para el Loop superior
S=min([AB AD BE DE]);
L=max([AB AD BE DE]);
T=sum([AB AD BE DE]);
PQ=T-S-L;

if(S+L<PQ)
   disp('Mecanismo es Grashof')
else
    disp('Mecanismo no es Grashof')
    return 
end

N=361;

%% Simulación inicial 
x_init=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];
H_path=[];

th2=linspace(theta_min,theta_max,N); % k valores equidistantes entre 0 y 2pi

figure(1);
for k = 1:100
%      th2=(k-1)*(2*pi)/(N-1);
     
     [xA, xB, xC, xD, xE, xF, xG, xH,~,~,~,~,~,~]=find_pos_mecanismo2([x_init,th2(k),theta3,theta4,theta5,theta6,theta7,theta8]);
     H_path=[H_path xH];
     
     plot_mecanismo2_proyecto(xA,xB,xC,xD,xE,xF,xG,xH,Hd,H_path)
     
     title('Initial linkage');
     pause(0.01);
end

%% Optimization of te Costo Function CF

% costFun = @CostFun_Mecanismo2_CE; %Cross entropy
costFun = @CostFun_Mecanismo2_N; %Norm


% Restricciones: limites inferiores y superiores para cada link

%x_init = x_init=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];;    % initial values
% Upper and lower bound set to +- 20% offset of initial dimensions
% lb      = [0.104  0.336  0.264  0.28  0.344  0.344 0.344 0.3120 0.2640 0.1840 0.8378]; 
% ub      = [0.156  0.504  0.396  0.42  0.516  0.516 0.516 0.468  0.396  0.2760 1.2566];  

% lb      = [0.104  0.336  0.264  0.344  0.344 0.344 0.3120 0.1840 ]; 
% ub      = [0.156  0.504  0.396  0.516  0.516 0.516 0.468  0.2760 ];  

lb      = 0.90*[AB  BE  DE  EG  FG BF CF GH ]; 
ub      = 1.1*[AB  BE  DE  EG  FG BF CF GH ];  


x_init=[AB,BE,DE,EG,FG,BF,CF,GH];

A = []; b = []; Aeq = []; beq = [];


vOpt = fmincon(costFun,x_init,A,b,Aeq,beq,lb,ub);
% vOpt = patternsearch(costFun,x_init,A,b,Aeq,beq,lb,ub);
%vOpt = ga(costFun,8,A,b,Aeq,beq,lb,ub);
% vOpt = particleswarm(costFun,8,lb,ub);
% vOpt = surrogateopt(costFun,lb,ub);



% Costo inicial y final:
CFi    = CostFun_Mecanismo2_N(x_init);
    disp(strcat('Initial Cost =',num2str(CFi)));
CF_Opt = CostFun_Mecanismo2_N(vOpt);
    disp(strcat('Final Cost =',num2str(CF_Opt)));

vOpt=[vOpt(1:3) AD vOpt(4:7) CD vOpt(8) pi-60*pi/180];

%% Simulation of the optimized mechanism
figure(2);
H_path=[];
for k = 1:100
%     th2 = (k-1)*(2*pi)/(N-1);
    
    [xA, xB, xC, xD, xE, xF, xG, xH,~,~,~,~,~,~]=find_pos_mecanismo2([vOpt,th2(k),theta3,theta4,theta5,theta6,theta7,theta8]);
    H_path = [H_path xH];
    
    plot_mecanismo2_proyecto(xA,xB,xC,xD,xE,xF,xG,xH,Hd,H_path)
    title('Optimized linkage');
    
    pause(0.01);
end

   


