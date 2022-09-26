% Esta función plotea el mecanismo a partir de las coordenadas de los
% puntos rA,rB,rC,rD,rE y rF del mecanismo; asimismo plotea el camino
% del punto A

function mech1_plot(rA,rB,rC,rD,rE,rF,A_path,Hd)
    set(groot,'defaultLineLineWidth',3.0)
    color = [135 206 250]./255;
    
    
    plot([rA(1) rB(1)],[rA(2) rB(2)])
    hold on
    plot([rB(1) rC(1)],[rB(2) rC(2)])
    plot([rA(1) rB(1)],[rA(2) rB(2)])
    plot([rA(1) rD(1)],[rA(2) rD(2)])
    plot([rA(1) rE(1)],[rA(2) rE(2)])
    plot([rC(1) rD(1)],[rC(2) rD(2)])
    plot([rC(1) rE(1)],[rC(2) rE(2)])
    plot([rD(1) rF(1)],[rD(2) rF(2)])
    plot([rE(1) rF(1)],[rE(2) rF(2)])
    
    %=============================================
%     
%     Plot de velocidad
%     plot([rB(1) rB(1)+e1(1)],[rB(2) rB(2)+e1(2)],'-r')
%     plot([rC(1) rC(1)+e2(1)],[rC(2) rC(2)+e2(2)],'-r')
%     plot([rD(1) rD(1)+e3(1)],[rD(2) rD(2)+e3(2)],'-r')
%     plot([rE(1) rE(1)+e4(1)],[rE(2) rE(2)+e4(2)],'-r')
%     plot([rD(1) rD(1)+e5(1)],[rD(2) rD(2)+e5(2)],'-r')
%     plot([rE(1) rE(1)+e6(1)],[rE(2) rE(2)+e6(2)],'-r')
%     plot([rF(1) rF(1)+e7(1)],[rF(2) rF(2)+e7(2)],'-r')
%     plot([rF(1) rF(1)+e8(1)],[rF(2) rF(2)+e8(2)],'-r')
% 
% Plot de aceleracion
%     plot([rC(1) rC(1)+vC(1)*0.1],[rC(2) rC(2)+vC(2)*0.1],'-b')
%     plot([rD(1) rD(1)+vD(1)*0.1],[rD(2) rD(2)+vD(2)*0.1],'-b')
%     plot([rE(1) rE(1)+vE(1)*0.1],[rE(2) rE(2)+vE(2)*0.1],'-b')
%     plot([rB(1) rB(1)+vB(1)*0.1],[rB(2) rB(2)+vB(2)*0.1],'-b')
%     plot([rA(1) rA(1)+vA(1)*0.1],[rA(2) rA(2)+vA(2)*0.1],'-b')
    
    %=============================================
    
    text(rA(1)+0.02,rA(2)+0.02,'A')
    text(rB(1)+0.02,rB(2)+0.02,'B')
    text(rC(1)+0.02,rC(2)+0.02,'C')
    text(rD(1)+0.02,rD(2)+0.02,'D')
    text(rE(1)+0.02,rE(2)+0.02,'E')
    text(rF(1)+0.02,rF(2)+0.02,'F')
    
    %ang = num2str(180*Theta/pi+90);
%     text(-6.8,-8,ang)
%     text(-9,-8,'Ángulo: ')
    
    plot(A_path(1,:),A_path(2,:),'g.');
    plot(Hd(1,:),Hd(2,:),'.r')
    axis equal,xlim([-1 1]),ylim([-0.9 0.1]),grid on,hold off

    % figure
    % plot_mecanismo1_proyecto(rA,rB,rC,rD,rE,rF,[0;-1])
     pause(0.01)
    
end