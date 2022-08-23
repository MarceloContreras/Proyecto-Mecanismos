% Esta función calcula las velocidades w3,w4,w5,w6,w7 y w8.
% x=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10,theta2,t3,t4,t5,t6,t7,t8,w2,w3,w4,w5,w6,w7,w8,alfa_2,xbar8]


function [alfa_3,alfa_4,alfa_5,alfa_6,alfa_7,alfa_8,a_g2,a_g3,a_g4,a_g5,a_g6,a_g7,a_g8,a_H,a_H1,a_H2]=find_acel_aceleracion2(x)
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
theta2= x(12);

t3=x(13);
t4=x(14);
t5=x(15);
t6=x(16);
t7=x(17);
t8=x(18);

w2=x(19);
w3=x(20);
w4=x(21);
w5=x(22);
w6=x(23);
w7=x(24);
w8=x(25);

alfa_2=x(26);
xbar8=x(27);

[e2,n2] = UnitVector(theta2);
[e3,n3] = UnitVector(t3);
[e4,n4] = UnitVector(t4);
[e5,n5] = UnitVector(t5);
[e6,n6] = UnitVector(t6);
[e7,n7] = UnitVector(t7);
[e8,n8] = UnitVector(t8);

A_a=[BE*n3 -DE*n4 zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);
    BE*n3 zeros(2,1) zeros(2,1) BF*n6 -EG*n7 +FG*n8;
    zeros(2,1) zeros(2,1) CF*n5 -BF*n6 zeros(2,1) zeros(2,1)];
B_a=[-AB*alfa_2*n2 + AB*w2^2*e2 + BE*w3^2*e3 - DE*w4^2*e4; 
    BE*w3^2*e3 - EG*w7^2*e7 + FG*w8^2*e8 + BF*w6^2*e6;
    -AB*alfa_2*n2 + AB*w2^2*e2 - BF*w6^2*e6 + CF*w5^2*e5];
C_a= A_a\B_a;
alfa_3=C_a(1);
alfa_4=C_a(2);
alfa_5=C_a(3);
alfa_6=C_a(4);
alfa_7=C_a(5);
alfa_8=C_a(6);

a_g2= FindAcc([0;0], AB/2, w2, alfa_2, e2, n2);
a_B= FindAcc([0;0], AB, w2, alfa_2, e2, n2);
a_g3= FindAcc(a_B, BE/2, w3, alfa_3, e3, n3);
a_g4= FindAcc([0;0], DE/2, w4, alfa_4, e4, n4);
a_E= FindAcc([0;0], DE, w4, alfa_4, e4, n4);

a_g6= FindAcc(a_B, BF/2, w6, alfa_6, -e6, -n6);
a_F= FindAcc(a_B, BF, w6, alfa_6, -e6, -n6);
a_g5= FindAcc(a_F, CF/2, w5, alfa_5, e5, n5);

a_g7= FindAcc(a_E, EG/2, w7, alfa_7, -e7, -n7);
a_G= FindAcc(a_E, EG, w7, alfa_7, -e7, -n7);

a_g8=FindAcc(a_F, FG+GH-xbar8, w8, alfa_8, -e8, -n8);

a_H=AB*alfa_2*n2 - AB*w2^2*e2 - BF*alfa_6*n6 + BF*w6^2*e6 -(FG+GH)*alfa_8*n8 + (FG+GH)*w8^2*e8;
a_H1=FindAcc(a_G, GH, w8, alfa_8, -e8, -n8);
a_H2=FindAcc(a_F, FG+GH, w8, alfa_8, -e8, -n8);
end