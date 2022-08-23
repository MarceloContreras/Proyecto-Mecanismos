
clc;clear all;close all
%% Valores iniciales

 Lab = 0.2;
    Lbc = 0.15;
    Lad = 0.6;
    Lae = 0.6;
    Lcd = 0.3;
    Lce = 0.3;
    Ldf = 0.22;
    Lef = 0.25;
    
% AB,BC ,AD,AE ,CE,DF,EF

k = 200;

omega2 = 2;
omega1 = 0;

theta1 = -pi/2;

alpha2 = 0;
alpha1 = 0;

T1 = linspace(-120,-56,k);

des = [0.5;0.5;0];
%  theta3 = -2.1123; theta4 = -1.926; theta5 = -2.2063;
%     theta6 = -1.832; theta7= -2.2063; theta8= -1.832; 
    
    
    theta3 = -121.7*pi/180; theta4 = -84.39*pi/180; theta5 = -142.81*pi/180;
    theta6 = -63.28*pi/180; theta7= -160.3*pi/180; theta8= -50.98*pi/180; 

    
%figure('units','normalized','outerposition',[0 0 1 1])

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

% prueba 3 puntos
% Hd =[[-0.1443   -0.0728    0.1305];
%       [-0.7608   -0.7126   -0.7690]+0.05*ones(1,3)];
% 
%    
%  th2d = [-2.0103   -1.6347   -1.0713];  


% prueba 5 puntos
% 
% Hd =[[-0.1443   -0.1214   -0.0728    0.0626    0.1305];
%       [-0.7608   -0.7313   -0.7126   -0.7458   -0.7690]+0.05*ones(1,5)];
% 
%    
%  th2d = [-2.0103   -1.8225   -1.6347   -1.2591   -1.0713];  


% prueba 10 puntos
% Hd =[[-0.1443   -0.1344   -0.1214   -0.1068   -0.0728   -0.0394   0.0287    0.0626    0.0967    0.1305];
%       [-0.7608   -0.7451   -0.7313   -0.7193   -0.7126   -0.7125   -0.7341   -0.7458   -0.7557   -0.7690]+0.05*ones(1,10)];
% 
%    
%  th2d = [-2.0103   -1.9164   -1.8225   -1.7286   -1.6347   -1.5408   -1.3530   -1.2591   -1.1652   -1.0713];  
% 
%  % prueba 13 puntos
% 
% 
Hd =[[-0.1593   -0.1443   -0.1344   -0.1214   -0.1068   -0.0728   -0.0394   -0.0073    0.0287    0.0626    0.0967    0.1305    0.1879];
      [-0.7777   -0.7608   -0.7451   -0.7313   -0.7193   -0.7126   -0.7125   -0.7256   -0.7341   -0.7458   -0.7557   -0.7690   -0.7796]+0.05*ones(1,13)];

   th2d = (linspace(-120.56,-56,13))*pi/180;
% th2d = [-2.1042   -2.0103   -1.9164   -1.8225   -1.7286   -1.6347   -1.5408   -1.4469   -1.3530   -1.2591   -1.1652   -1.0713   -0.9774];  

 N=361;

%% Simulación inicial 
% x_init=[AB,BC,AD,AE,CD,CE,DF,EF];
H_path=[];
k =200;
% th2=linspace(theta_min,theta_max,N); % k valores equidistantes entre 0 y 2pi
% T1 = linspace(-49,28,k);
% T1 = linspace(-35.51,31.94,k);
T1 = linspace(-120,-56,k);
figure(1);
for i = 1:k
    theta2 = T1(i)*pi/180;
     
     
    
    x=[Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef,theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta8];
    [rA, rB, rC, rD, rE, rF,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo1_v2(x);
     
  
     H_path=[H_path rA];
     plot_mecanismo1_proyecto(rA,rB,rC,rD,rE,rF,H_path,Hd)
     
     title('Initial linkage');
     hold off
%      pause(0.01);
end
hold off


 %% Optimization of te Costo Function CF



% % costFun = @CostFun_Mecanismo2_CE; %Cross entropy
 costFun = @CostFun_Mecanismo1_N; %Norm


% Restricciones: limites inferiores y superiores para cada link

%x_init = x_init=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];;    % initial values
% Upper and lower bound set to +- 20% offset of initial dimensions
% lb      = [0.104  0.336  0.264  0.28  0.344  0.344 0.344 0.3120 0.2640 0.1840 0.8378]; 
% ub      = [0.156  0.504  0.396  0.42  0.516  0.516 0.516 0.468  0.396  0.2760 1.2566];  

% lb      = [0.104  0.336  0.264  0.344  0.344 0.344 0.3120 0.1840 ]; 
% ub      = [0.156  0.504  0.396  0.516  0.516 0.516 0.468  0.2760 ];  

lb      = 0.90*[Lab Lbc Lad Lae Lcd Lce Ldf Lef]; 
ub      = 1.1*[Lab Lbc Lad Lae Lcd Lce Ldf Lef];  


 x_init=[Lab Lbc Lad Lae Lcd Lce Ldf Lef theta1];

A = []; b = []; Aeq = []; beq = [];

tic
%   vOpt = fmincon(costFun,x_init,A,b,Aeq,beq,lb,ub);
%    vOpt = patternsearch(costFun,x_init,A,b,Aeq,beq,lb,ub);
%    vOpt = ga(costFun,8,A,b,Aeq,beq,lb,ub); % 8 = valores iniciales
 vOpt = particleswarm(costFun,8,lb,ub);
% vOpt = surrogateopt(costFun,lb,ub);

T=toc

% Costo inicial y final:
CFi    = CostFun_Mecanismo1_N(x_init);
    disp(strcat('Initial Cost =',num2str(CFi)));
CF_Opt = CostFun_Mecanismo1_N(vOpt);
    disp(strcat('Final Cost =',num2str(CF_Opt)));


%% Simulation of the optimized mechanism

H_path2=[];
rA = [];
% 
theta1 = -pi/2;
 theta3 = -2.1123; theta4 = -1.926; theta5 = -2.2063;
    theta6 = -1.832; theta7= -2.2063; theta8= -1.832; 

if(S+L<PQ)
   disp('Mecanismo es Grashof')
else
    disp('Mecanismo no es Grashof')
    return 
end

 theta1 = -pi/2;
  theta3 = -121.7*pi/180; theta4 = -84.39*pi/180; theta5 = -142.81*pi/180;
    theta6 = -63.28*pi/180; theta7= -160.3*pi/180; theta8= -50.98*pi/180; 
    
    hold off
    figure(2)
for j = 1:200
 
    
     theta2 = T1(j)*pi/180;
    x=[[vOpt(1:8)],theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta8];
    [rA, rB, rC, rD, rE, rF,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo1_v2(x);
  
     H_path2=[H_path2 rA];
     plot_mecanismo1_proyecto(rA,rB,rC,rD,rE,rF,H_path2,Hd)
     
     title('Optimized Linkage');
     hold off
     pause(0.01);
   
    
end


%    % global optimization toolboix
   





























