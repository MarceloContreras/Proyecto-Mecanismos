% Esta función calcula las aceleraciones del mecanismo 1.
% x=[Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef,theta1,theta2,t3,t4,t5,t6,t7,t8,w2,w3,w4,w5,w6,w7,w8,alpha2]
                                                                                             %ELIMINAR LO DE ABAJO
function [alpha3,alpha4,alpha5,alpha6,alpha7,alpha8,a_g1,a_g2,a_g3,a_g4,a_g5,a_g6,a_g7,a_g8,aA]=find_acel_mecanismo1(x)
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

w2=x(17);
w3=x(18);
w4=x(19);
w5=x(20);
w6=x(21);
w7=x(22);
w8=x(23);

alpha2=x(24);

[e1,n1] = UnitVector(theta1);
[e2,n2] = UnitVector(theta2);
[e3,n3] = UnitVector(t3);
[e4,n4] = UnitVector(t4);
[e5,n5] = UnitVector(t5);
[e6,n6] = UnitVector(t6);
[e7,n7] = UnitVector(t7);
[e8,n8] = UnitVector(t8);

A2 = [Lae*n4,-Lce*n6      ,zeros(2,4)                        ;
      zeros(2,2)          ,Lad*n3,-Lcd*n5     ,zeros(2,2)    ;
      zeros(2,1),-Lce*n6  ,zeros(2,1),Lcd*n5  ,Ldf*n8,-Lef*n7];
  
B2 = [Lae*w4^2*e4-Lce*w6^2*e6+Lbc*alpha2*n2-w2^2*Lbc*e2;
      Lad*w3^2*e3-Lcd*w5^2*e5+Lbc*alpha2*n2-w2^2*Lbc*e2;
      Lcd*w5^2*e5-Lce*w6^2*e6-Lef*w7^2*e7+w8^2*Ldf*e8];

vec2 = linsolve(A2,B2);

alpha4 = vec2(1);
alpha6 = vec2(2);
alpha3 = vec2(3);
alpha5 = vec2(4);
alpha8 = vec2(5);
alpha7 = vec2(6);

aD = FindAcc([0;0], Ldf, w8, alpha8, e8, n8);
aE = FindAcc([0;0], Lef, w7, alpha7, e7, n7);
aC = FindAcc(aE, Lce, w6, alpha6, e6, n6);
aB = FindAcc(aC, Lbc, w2, alpha2, e2, n2);
aA = FindAcc(aB, Lab, 0, 0, e1, n1);

a_g8 = FindAcc([0;0], Ldf/2, w8, alpha8, e8, n8);
a_g7 = FindAcc([0;0], Lef/2, w7, alpha7, e7, n7);
a_g6 = FindAcc(aE, Lce/2, w6, alpha6, e6, n6);
a_g5 = FindAcc(aD, Lcd/2, w5, alpha5, e5, n5);
a_g4 = FindAcc(aE, Lae/2, w4, alpha4, e4, n4);
a_g3 = FindAcc(aD, Lad/2, w3, alpha3, e3, n3);
a_g2 = FindAcc(aC, Lbc/2, w2, alpha2, e2, n2);
a_g1 = FindAcc(aB, Lab/2, 0, 0, e1, n1);
end