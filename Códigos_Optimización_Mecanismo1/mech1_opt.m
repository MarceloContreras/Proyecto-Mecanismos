function [vOpt,CFi,CF_Opt] = mech1_opt(lb,ub,x_init,solver)

    % Cost function of norm_2
    costFun = @mech1_costfun; 
   
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
    CFi    = mech1_costfun(x_init); disp(strcat('Initial Cost =',num2str(CFi)));
    CF_Opt = mech1_costfun(vOpt) ;  disp(strcat('Final Cost =',num2str(CF_Opt)));

end

