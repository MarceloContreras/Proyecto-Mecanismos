function [vOpt,CFi,CF_Opt] = mech2_opt(lb,ub,x_init,solver)
    
    costFun = @mech2_costfun; %Norm    
    
    A = []; b = []; Aeq = []; beq = [];
    
    tic
    switch solver
        case 'fmincon'
            vOpt = fmincon(costFun,x_init,A,b,Aeq,beq,lb,ub);
        case 'pattern'
            vOpt = patternsearch(costFun,x_init,A,b,Aeq,beq,lb,ub);
        case 'ga'
            vOpt = ga(costFun,8,A,b,Aeq,beq,lb,ub);
        case 'particle'
            vOpt = particleswarm(costFun,8,lb,ub);
        case 'surrogate'
            vOpt = surrogateopt(costFun,lb,ub);
    end
    T=toc
    
    % Costo inicial y final:
    CFi    = mech2_costfun(x_init); disp(strcat('Initial Cost =',num2str(CFi)));
    CF_Opt = mech2_costfun(vOpt);   disp(strcat('Final Cost =',num2str(CF_Opt)));
    
    AD = 0.350; CD = 0.330;
    vOpt=[vOpt(1:3) AD vOpt(4:7) CD vOpt(8) pi-60*pi/180];
end

