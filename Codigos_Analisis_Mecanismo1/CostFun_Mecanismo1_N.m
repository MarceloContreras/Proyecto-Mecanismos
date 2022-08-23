%% Esta función calcula el costo como la suma de las normas de la 
% diferencia entre el valor deseado y el real para los puntos deseados 
% v = [Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef,theta1];

function CF = CostFun_Mecanismo1_N(v)

    global Pd th2d
    
    % Valores iniciales
    theta3 = -2.1123; theta4 = -1.926; theta5 = -2.2063;
    theta6 = -1.832; theta7= -2.2063; theta8= -1.832; 
    
    CF = 0;
    for k=1:length(th2d)
        x = [v th2d(k) theta3 theta4 theta5 theta6 theta7 theta8];
        [P, ~,~, ~,~, ~,~, ~,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo1(x);
         CF = CF + norm(P - Pd(:,k));
    end
end