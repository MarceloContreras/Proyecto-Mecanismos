% Calcula los vectores asociados al centro de masa

% ** Inputs **
% a = longitud de A a B
% c = longitud de A a C
% gamma= angulo entre AB y AC
% xbar = coordenadas de CM
% theta = angulo del objeto en el sistema global

%** Outputs**
% eAg,nAg = vector unitario y normal de A al CM
% LAg = longitud del punto A a CM
% sgA = normal al vector CM a A
% sgB = normal al vector CM a B
% sgC = normal al vector CM a C

function [eAg,nAg,LAg,sgA,sgB,sgC]= LinkCG(a,c,gamma,xbar,theta)
    rAB=a*[1;0];
    rAC=c*[cos(gamma);sin(gamma)];
    rgA=[-xbar(1);-xbar(2)];
    rgB=rAB+rgA;
    rgC=rAC+rgA;

    LAg=norm(rgA);
    eAg=-(1/LAg)*rgA;
    nAg=[-eAg(2);eAg(1)];
    
    sgA=[-rgA(2);rgA(1)];
    sgB=[-rgB(2);rgB(1)];
    sgC=[-rgC(2);rgC(1)];

    R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
    
    sgA=R*sgA; sgB=R*sgB; sgC=R*sgC;
    eAg=R*eAg; nAg=R*nAg; 
end