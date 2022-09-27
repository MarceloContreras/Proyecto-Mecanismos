function mech2_simulation(vOpt,optimized)
    
    global Hd

    AB = 0.130;
    BE = 0.420;     % crank length (cm)
    DE = 0.330;      
    AD = 0.350;    
    EG = 0.430;    
    FG = 0.430;    
    BF = 0.430;     
    CF = 0.390;
    CD = 0.330;
    GH = 0.230;
    theta_10= pi-60*pi/180;

    H_path=[];
    th2 = linspace(270*pi/180,-125*pi/180,361);

    if optimized == 1
        x_init = [vOpt(1:3) AD vOpt(4:7) CD vOpt(8) pi-60*pi/180];
    else
        x_init=[AB,BE,DE,AD,EG,FG,BF,CF,CD,GH,theta_10];
    end

    theta_1 = 0;
    theta3 = pi+25*pi/180; theta4 = pi+75*pi/180; theta5 = pi-40*pi/180;
    theta6 = pi-70*pi/180; theta7= pi-65*pi/180; theta8= pi-150*pi/180;

    figure(1);
    for k = 1:100
         
         [xA, xB, xC, xD, xE, xF, xG, xH,~,~,~,~,~,~]=mech2_findpos([x_init,th2(k),theta3,theta4,theta5,theta6,theta7,theta8]);
         H_path=[H_path xH];
         
         mech2_plot(xA,xB,xC,xD,xE,xF,xG,xH,Hd,H_path)
         
         title('Initial linkage');
         pause(0.01);
    end
end

