% Esta función calcula las velocidades w3,w4,w5,w6,w7 y w8.
% x=[Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef,theta1,theta2,t3,t4,t5,t6,t7,t8,w2]

function [w3,w4,w5,w6,w7,w8,v_g1,v_g2,v_g3,v_g4,v_g5,v_g6,v_g7,v_g8,vD,vE,vC,vB,vA]=find_vel_mecanismo1(x)
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

[~,n1] = UnitVector(theta1);
[~,n2] = UnitVector(theta2);
[~,n3] = UnitVector(t3);
[~,n4] = UnitVector(t4);
[~,n5] = UnitVector(t5);
[~,n6] = UnitVector(t6);
[~,n7] = UnitVector(t7);
[~,n8] = UnitVector(t8);

A1 = [Lae*n4,-Lce*n6      ,zeros(2,4)                        ;
      zeros(2,2)          ,Lad*n3,-Lcd*n5     ,zeros(2,2)    ;
      zeros(2,1),-Lce*n6  ,zeros(2,1),Lcd*n5  ,Ldf*n8,-Lef*n7];

B1 = [w2*Lbc*n2;
      w2*Lbc*n2;
      0;
      0];

vec1 = linsolve(A1,B1);

w4 = vec1(1);
w6 = vec1(2);
w3 = vec1(3);
w5 = vec1(4);
w8 = vec1(5);
w7 = vec1(6);

vD = FindVel(zeros(2,1),Ldf,w8,n8);
vE = FindVel(zeros(2,1),Lef,w7,n7);
vC = FindVel(vE,Lce,w6,n6);
vB = FindVel(vC,Lbc,w2,n2);
vA = FindVel(vB,Lab,0,n1);

v_g8 = FindVel(zeros(2,1),Ldf/2,w8,n8);
v_g7 = FindVel(zeros(2,1),Lef/2,w7,n7);
v_g6 = FindVel(vE,Lce/2,w6,n6);
v_g5 = FindVel(vD,Lcd/2,w5,n5);
v_g4 = FindVel(vE,Lae/2,w4,n4);
v_g3 = FindVel(vD,Lad/2,w3,n3);
v_g2 = FindVel(vC,Lbc/2,w2,n2);
v_g1 = FindVel(vB,Lab/2,0,n1);

end