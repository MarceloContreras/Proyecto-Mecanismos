%% Esta función calcula el costo como la entropia cruzada (cross-entropy) de la diferencia entre el
% valor deseado y el real para cada punto deseado en Pd.
% v = [AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];

function CF = CostFun_Mecanismo2_CE(v)

    global Hd th2d
    theta3 = -25*pi/180; theta4 = -75*pi/180; theta5 = 40*pi/180;
    theta6 = 70*pi/180; theta7= 65*pi/180; theta8= 150*pi/180; 
    
    CF = 0;
    for k=1:length(th2d)
        x = [v th2d(k) theta3 theta4 theta5 theta6 theta7 theta8];
        [~, ~,~, ~,~, ~,~, P,theta3,theta4,theta5,theta6,theta7,theta8] = find_pos_mecanismo2(x);
        CF= CF + (Hd(:,k)*log(P)+(1-Hd(:,k)*log(1-P)));
    end
    CF=-1/n*CF;
end