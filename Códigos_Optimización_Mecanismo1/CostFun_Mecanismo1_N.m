%% Esta función calcula el costo como la suma de las normas de la 
% diferencia entre el valor deseado y el real para los puntos deseados 
% v = [Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef,theta1];

function CF = CostFun_Mecanismo1_N(v)

    global Hd th2d
    
    % Valores iniciales
    theta1 = -pi/2;
  theta3 = -121.7*pi/180; theta4 = -84.39*pi/180; theta5 = -142.81*pi/180;
    theta6 = -63.28*pi/180; theta7= -160.3*pi/180; theta8= -50.98*pi/180; 
    CF = 0;
    for k=1:length(th2d)
         x = [v(1:8) theta1 th2d(k) theta3 theta4 theta5 theta6 theta7 theta8];
         [P, ~, ~, ~,~, ~,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo1_v2(x);
         CF = CF + norm(P - Hd(:,k));
    end
end