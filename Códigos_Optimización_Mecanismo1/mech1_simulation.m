function mech1_simulation(vOpt,optimized)
    
    global Hd

    H_path=[];
    k =200;
    T1 = linspace(-120,-56,k);

    if optimized == 1
        L = vOpt(1:8);
    else
        L = [0.2, 0.15, 0.6, 0.6, 0.3, 0.3, 0.22, 0.25];
        % L = Lab,Lbc,Lad,Lae,Lcd,Lce,Ldf,Lef
    end

    theta1 = -pi/2;
    % Valor inicial en deg
    theta3 = -121.7*pi/180; theta4 = -84.39*pi/180; theta5 = -142.81*pi/180;
    theta6 = -63.28*pi/180; theta7= -160.3*pi/180; theta8= -50.98*pi/180; 

    figure(1);
    for i = 1:k
         theta2 = T1(i)*pi/180;
         
         x=[L,theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta8];
         [rA, rB, rC, rD, rE, rF,theta3,theta4,theta5,theta6,theta7,theta8] = mech1_findpos(x);
         
         H_path=[H_path rA];
         mech1_plot(rA,rB,rC,rD,rE,rF,H_path,Hd)
         
         title('Initial linkage');
         hold off
    %    pause(0.01);
    end
    hold off
end

