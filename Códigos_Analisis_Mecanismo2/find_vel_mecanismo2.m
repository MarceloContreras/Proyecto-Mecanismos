% Esta función calcula las velocidades w3,w4,w5,w6,w7 y w8.
% x=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10,theta2,t3,t4,t5,t6,t7,t8,w2,xbar8]


function [w3,w4,w5,w6,w7,w8,v_g2,v_g3,v_g4,v_g5,v_g6,v_g7,v_g8,v_H,v_H1,v_H2]=find_vel_mecanismo2(x)
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
xbar8=x(20);

[~,n2] = UnitVector(theta2);
[~,n3] = UnitVector(t3);
[~,n4] = UnitVector(t4);
[~,n5] = UnitVector(t5);
[~,n6] = UnitVector(t6);
[~,n7] = UnitVector(t7);
[~,n8] = UnitVector(t8);

A_v=[BE*n3 -DE*n4 zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);
    BE*n3 zeros(2,1) zeros(2,1) BF*n6 -EG*n7 +FG*n8;
    zeros(2,1) zeros(2,1) CF*n5 -BF*n6 zeros(2,1) zeros(2,1)];
B_v=[-AB*w2*n2; zeros(2,1); -AB*w2*n2];
C= A_v\B_v;
w3=C(1);
w4=C(2);
w5=C(3);
w6=C(4);
w7=C(5);
w8=C(6);

v_g2= FindVel([0;0], AB/2, w2, n2);
v_B= FindVel([0;0], AB, w2, n2);
v_g3= FindVel(v_B, BE/2, w3, n3);
v_g4= FindVel([0;0], DE/2, w4, n4);
v_E= FindVel([0;0], DE, w4, n4);

v_g6= FindVel(v_B, BF/2, w6, -n6);
v_F= FindVel(v_B, BF, w6, -n6);
v_g5= FindVel(v_F, CF/2, w5, n5);

v_g7= FindVel(v_E, EG/2, w7, -n7);
v_G= FindVel(v_E, EG, w7, -n7);

v_g8=FindVel(v_F, FG+GH-xbar8, w8, -n8);

v_H=AB*w2*n2 -BF*w6*n6 -(FG+GH)*w8*n8;
v_H1=FindVel(v_G, GH, w8, -n8);
v_H2=FindVel(v_F, FG+GH, w8, -n8);
end
