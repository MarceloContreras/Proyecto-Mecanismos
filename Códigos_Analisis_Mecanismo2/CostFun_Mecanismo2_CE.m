%% Esta función calcula el costo como la entropia cruzada (cross-entropy) de la diferencia entre el
% valor deseado y el real para cada punto deseado en Pd.
% v = [AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];

function CF = CostFun_Mecanismo2_CE(v)

    global Pd th2d
    theta3 = pi+25*pi/180; theta4 = pi+75*pi/180; theta5 = pi-40*pi/180;
    theta6 = pi-70*pi/180; theta7= pi-65*pi/180; theta8= pi-150*pi/180; % tn means theta_n
    
    CF = 0;
    for k=1:length(th2d)
        x = [v th2d(k) theta3 theta4 theta5 theta6 theta7 theta8];
        [~, ~,~, ~,~, ~,~, P,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo2(x);
        CF= CF + (Pd(:,k)*log(P)+(1-Pd(:,k)*log(1-P)));
    end
    CF=-1/n*CF;
end