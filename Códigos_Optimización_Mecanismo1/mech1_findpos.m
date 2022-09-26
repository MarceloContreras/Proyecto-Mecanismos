% Esta función calcula la posición de los puntos A,B,C,D,E y F, además de
% los valores de theta3, theta4, theta5,theta6,theta7 y theta8 del
% mecanismo 1 mediante el método de Newton Raphson.
% x=[Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef,theta1,theta2,t3,t4,t5,t6,t7,t8]

function [xA, xB, xC, xD, xE, xF,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo1_v2(x)
Lab = x(1);
Lbc = x(2);
Lad = x(3);
Lae = x(4);
Lcd = x(5);
Lce = x(6);
Ldf = x(7);
Lef = x(8);

theta1 = x(9);
theta2 = x(10);

t3=x(11);
t4=x(12);
t5=x(13);
t6=x(14);
t7=x(15);
t8=x(16);

[e1,~] = UnitVector(theta1);
[e2,~] = UnitVector(theta2);

%% Newton Raphson Method
for z = 1:100  % máximo 100 iteraciones
    [e3,n3] = UnitVector(t3);
    [e4,n4] = UnitVector(t4);
    [e5,n5] = UnitVector(t5);
    [e6,n6] = UnitVector(t6);
    [e7,n7] = UnitVector(t7);
    [e8,n8] = UnitVector(t8);
    
    phi(:,1) = [ Ldf*e8 + Lcd*e5 - Lef*e7 - Lce*e6 ;
                 Lce*e6 + Lbc*e2 + Lab*e1 - Lae*e4;
                 Lcd*e5 + Lbc*e2 + Lab*e1 - Lad*e3 ];
    % q = [theta3, theta4, theta5, theta6, theta7, theta8]
    J =  [zeros(2,1)   zeros(2,1)  Lcd*n5     -Lce*n6     -Lef*n7      Ldf*n8;
          zeros(2,1)  -Lae*n4      zeros(2,1)  Lce*n6      zeros(2,1)  zeros(2,1);
         -Lad*n3       zeros(2,1)  Lcd*n5      zeros(2,1)  zeros(2,1)  zeros(2,1)];
    
    dq = -J\phi;    % - inv(J)*phi
    t3 = t3 + dq(1);
    t4 = t4 + dq(2);
    t5 = t5 + dq(3);
    t6 = t6 + dq(4);
    t7 = t7 + dq(5);
    t8 = t8 + dq(6);
    
    if norm(phi) < 0.00001
       % disp(strcat('Convergió en la iteración:',num2str(z)));
        break
    end
end
%%
theta3 = t3;
theta4 = t4;
theta5 = t5;
theta6 = t6;
theta7 = t7;
theta8 = t8;

[e6,~] = UnitVector(t6);
[e7,~] = UnitVector(t7);
[e8,~] = UnitVector(t8);

% Cálculo de las posiciones de los puntos A,B,C,D,E,F
xF = [0;0]; % ground pin at F (origin)
xD = xF + Ldf*e8;
xE = xF + Lef*e7;
xC = FindPos( xE, Lce, e6);
xB = FindPos( xC, Lbc, e2);
xA = FindPos( xB, Lab, e1);
end