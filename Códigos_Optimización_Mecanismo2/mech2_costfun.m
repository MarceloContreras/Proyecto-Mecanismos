%% Esta función calcula el costo como la suma de las normas de la 
% diferencia entre el valor deseado y el real para los puntos deseados 
% v = [AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];

function CF = mech2_costfun(v)

    global Hd th2d

    theta3 = pi+25*pi/180; theta4 = pi+75*pi/180; theta5 = pi-40*pi/180;
    theta6 = pi-70*pi/180; theta7= pi-65*pi/180; theta8= pi-150*pi/180; 
    AD=0.350;
    theta10=pi-60*pi/180 ;
    CD= 0.330;
    
    CF = 0;
    for k=1:length(th2d)
%         x = [v th2d(k) theta3 theta4 theta5 theta6 theta7 theta8];
        x = [v(1:3) AD v(4:7) CD v(8) theta10 th2d(k) theta3 theta4 theta5 theta6 theta7 theta8];
        [~, ~,~, ~,~, ~,~, H,theta3,theta4,theta5,theta6,theta7,theta8] = mech2_findpos(x);
         CF = CF + norm(H - Hd(:,k));
    end
end