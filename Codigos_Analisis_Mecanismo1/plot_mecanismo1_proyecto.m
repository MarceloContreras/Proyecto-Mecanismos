% Esta función plotea el mecanismo a partir de las coordenadas de los
% puntos rA,rB,rC,rD,rE y rF del mecanismo 1; asimismo plotea el camino
% del punto A

function plot_mecanismo1_proyecto(rA,rB,rC,rD,rE,rF,A_path)
    set(groot,'defaultLineLineWidth',3.0)
        
    % Draw the lines
    plot([rA(1) rB(1)],[rA(2) rB(2)])
    hold on
    plot([rB(1) rC(1)],[rB(2) rC(2)])
    plot([rA(1) rD(1)],[rA(2) rD(2)])
    plot([rA(1) rE(1)],[rA(2) rE(2)])
    plot([rC(1) rD(1)],[rC(2) rD(2)])
    plot([rC(1) rE(1)],[rC(2) rE(2)])
    plot([rD(1) rF(1)],[rD(2) rF(2)])
    plot([rE(1) rF(1)],[rE(2) rF(2)])
    
    % Ploteo de los puntos deseados para P y Q
    punto_P=plot(A_path(1,:),A_path(2,:),'r.');
    hold off
    
    xlim([-1 1]);
    ylim([-1 0.1]);
    xlabel('x');
    ylabel('y');
    title('Mecanismo 1 Proyecto')
    grid on;
    axis equal;
    
end