%% Eight Bar Linkage : Kevin Optimizacion 
% CURSO MT-0001 Mechanisms and movement transmition
% Profesor: Elvis Jara
% UTEC

clc;clear all;close all
%% Valores iniciales

load("mech2_data_points.mat")
mech2_load_param();

N=361;
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

%% Tarjet - 3,5,10,13 puntos deseados para H
global Hd th2d

keySet   = {'3','5','10','13'};
valueSet = [1 2 3 4];
dicc     = containers.Map(keySet,valueSet);

num_points = '3'; % Esto se puede cambiar

Hd   = data{2,dicc(num_points)}{1};
th2d = data{2,dicc(num_points)}{2};

%% Simulación inicial 
%x_init=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];
H_path=[];

% Simulation without optimization
mech2_simulation(0,0);

%% Optimization of the Cost Function CF

% Restricciones: limites inferiores y superiores para cada link
lb      = 0.90*[AB  BE  DE  EG  FG BF CF GH ]; 
ub      = 1.1*[AB  BE  DE  EG  FG BF CF GH ];  

%Initial values
x_init=[AB,BE,DE,EG,FG,BF,CF,GH];

%Optimization 
[vOpt,~,~] = mech2_opt(lb,ub,x_init,'fmincon');
%% Simulation of the optimized mechanism

mech2_simulation(vOpt,1)

   


