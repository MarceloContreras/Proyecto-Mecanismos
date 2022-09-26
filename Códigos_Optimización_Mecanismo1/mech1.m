clc;clear all;close all
%% Valores iniciales

load("mech1_data_points.mat")
mech1_load_param();

%% Condición de Grashof para el Loop superior

S=min([Lcd Lce Ldf Lef]);
L=max([Lcd Lce Ldf Lef]);
T=sum([Lcd Lce Ldf Lef]);
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

N=361;

%% Simulación inicial 
% x_init=[AB,BC,AD,AE,CD,CE,DF,EF];

mech1_simulation(0,0)

 %% Optimization of te Costo Function CF

% Upper and lower bound set to +- 20% offset of initial dimensions
lb      = 0.90*[Lab Lbc Lad Lae Lcd Lce Ldf Lef]; 
ub      = 1.1*[Lab Lbc Lad Lae Lcd Lce Ldf Lef];  

% Init val
x_init=[Lab Lbc Lad Lae Lcd Lce Ldf Lef theta1];

[vOpt,~,~] = mech1_opt(lb,ub,x_init,'fmincon');

%% Simulation of the optimized mechanism

mech1_simulation(vOpt,1)

   





























