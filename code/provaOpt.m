function [x,f,eflag,outpt] = provaOpt(x0, profileFunction, lb, ub)

options = optimoptions('fmincon','Display','iter-detailed','Algorithm',...
                       'interior-point','UseParallel',true,'Diagnostics',...
                       'on', 'OutputFcn', @outfun);

                   
% Set up shared variables with OUTFUN
history.x = [];
history.fval = [];
searchdir = [];


xLast = []; % Last place computeall was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq = []; % Use for nonlinear equality constraint

fun = @objfun; % the objective function, nested below
cfun = @constr; % the constraint function, nested below

A = [];
b = [];
Aeq = [];
beq = [];

% Call fmincon
[x,f,eflag,outpt] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,cfun,options);

    function y = objfun(x)
        sprintf('X = %2.16e   ', x)
        if ~isequal(x,xLast) % Check if computation is necessary
            disp("Evaluating objective function")
            [myf,myc,myceq] = MainRoutine(x, profileFunction);
            xLast = x;
        else
            disp("Obtaining evaluated objective function")
        end
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = constr(x)
        sprintf('X = %2.16e   ', x)
        if ~isequal(x,xLast) % Check if computation is necessary
            disp("Evaluating nonlinear constraints")
            [myf,myc,myceq] = MainRoutine(x, profileFunction);
            xLast = x;
        else
            disp("Obtaining evaluated nonlinear constraints")
        end
        % Now compute constraint functions
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

    function stop = outfun(x,optimValues,state)
    stop = false;

       switch state
           case 'init'
               hold on
           case 'iter'
               % Concatenate current point and objective function
               % value with history. x must be a row vector.
               history.fval = [history.fval; optimValues.fval];
               history.x = [history.x; x];
               % Concatenate current search direction with 
               % searchdir.
               searchdir = [searchdir;...
                            optimValues.searchdirection'];
               
               plot(x(1),x(2),'o');
               % Label points with iteration number.
               % Add .15 to x(1) to separate label from plotted 'o'
               text(x(1)+.15,x(2),num2str(optimValues.iteration));
               
               plot(profileFunction(x, 2000),)
               
           case 'done'
               hold off
           otherwise
       end
    end

end