% Esta función calcula la posición de los puntos A,B,C,D,E,F,G y H, además de
% los valores de theta3, theta4, theta5,theta6,theta7 y theta8 
% mediante el método de Newton Raphson.
% x=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10,theta2,t3,t4,t5,t6,t7,t8]

function [xA, xB, xC, xD, xE, xF, xG, xH,theta3,theta4,theta5,theta6,theta7,theta8] = mech2_findpos(x)
AB = x(1);
BE = x(2);     
DE = x(3);      
AD = x(4);     
EG = x(5);     
FG = x(6);     
BF = x(7);     
CF = x(8);
CD = x(9);
GH = x(10);
 
theta_10= x(11);
theta_1 = pi;
theta2= x(12);

t3=x(13);
t4=x(14);
t5=x(15);
t6=x(16);
t7=x(17);
t8=x(18);

[e1,~] = UnitVector(theta_1);
[e10,~] = UnitVector(theta_10);

[e2,~] = UnitVector(theta2);

%% Newton Raphson Method
for z = 1:100  % máximo 100 iteraciones
    [e3,n3] = UnitVector(t3);
    [e4,n4] = UnitVector(t4);
    [e5,n5] = UnitVector(t5);
    [e6,n6] = UnitVector(t6);
    [e7,n7] = UnitVector(t7);
    [e8,n8] = UnitVector(t8);
    
    phi(:,1) = [ AB*e2 + BE*e3 - DE*e4 - AD*e1 ;
        BE*e3 - EG*e7 + FG*e8 + BF*e6;
        AB*e2 - BF*e6 + CF*e5 + CD*e10 - AD*e1];
    % q = [theta3, theta4, theta5, theta6, theta7, theta8]
    J = [BE*n3 -DE*n4 zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);
        BE*n3 zeros(2,1) zeros(2,1) BF*n6 -EG*n7 FG*n8;
        zeros(2,1) zeros(2,1) CF*n5 -BF*n6 zeros(2,1) zeros(2,1)];
    
    dq = -J\phi;    % - inv(J)*phi
    t3 = t3 + dq(1);
    t4 = t4 + dq(2);
    t5 = t5 + dq(3);
    t6 = t6 + dq(4);
    t7 = t7 + dq(5);
    t8 = t8 + dq(6);
    
    if norm(phi) < 0.00001
        disp(strcat('Convergió en la iteración:',num2str(z)));
        break
    end
end
%%
theta3=t3;
theta4=t4;
theta5= t5;
theta6= t6;
theta7= t7;
theta8 = t8;

[e4,~] = UnitVector(t4);
[e6,~] = UnitVector(t6);
[e7,~] = UnitVector(t7);
[e8,~] = UnitVector(t8);

% Cálculo de las posiciones de los puntos A,B,C,D,E,F,G,H
xA = [0;0]; % ground pin at A (origin)
xD = xA+AD*e1;
xC = xD-CD*e10;
xB = FindPos( xA, AB, e2);
xF = FindPos( xB, BF, -e6);
xE = FindPos( xD, DE, e4);
xG = FindPos( xE, EG, -e7);
xH = FindPos( xG, GH, -e8);

end



