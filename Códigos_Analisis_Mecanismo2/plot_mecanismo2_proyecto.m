% Esta función plotea el mecanismo a partir de las coordenadas de los
% puntos xA,xB,xC,xD,xE,xF,xG y xH del mecanismo; asimismo plotea el camino
% del punto H

function plot_mecanismo2_proyecto(xA,xB,xC,xD,xE,xF,xG,xH,H_path)
    set(groot,'defaultLineLineWidth',3.0)
    color = [135 206 250]./255;
        
    % Draw the lines
    fill([xA(1) xC(1) xD(1) xA(1)],[xA(2) xC(2) xD(2) xA(2)],color); hold on;
    plot([xA(1) xB(1)],[xA(2) xB(2)]); % Link AB       
    plot([xD(1) xE(1)],[xD(2) xE(2)]); % Link DE   
    plot([xB(1) xE(1)],[xB(2) xE(2)]); % Link BE
    plot([xB(1) xF(1)],[xB(2) xF(2)]); % Link BF
    plot([xC(1) xF(1)],[xC(2) xF(2)]); % Link CF
    plot([xE(1) xG(1)],[xE(2) xG(2)]); % Link EG
    plot([xF(1) xH(1)],[xF(2) xH(2)]); % Link FH
    
    % Ploteo de los puntos deseados para P y Q
    punto_P=plot(H_path(1,:),H_path(2,:),'r.');
    
    hold off;
    xlim([-30/100 30/100]);
    ylim([-105/100 25/100]);
    xlabel('x (m)');
    ylabel('y (m)');
    title('Mecanismo 2 Proyecto')
    grid on;
    axis equal;
end