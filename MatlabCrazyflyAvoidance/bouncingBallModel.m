function [t,j,x] = bouncingBallModel(x0, simTime)
    global HEQOptions
    TSPAN = [0, simTime];
    JSPAN = [0, max(10,2*simTime)]; % allow two bounces per second
    [t,j,x] = HyEQsolver(@fobj,@gobj,@Cobj,@Dobj,x0,TSPAN,JSPAN,1,HEQOptions);
end

function xdot = fobj(x)
    global gamma
    if(~isempty(gamma))
        gamma = 9.8;
    end
    xdot = [x(2); 0; x(4); 0; x(6); -gamma; 0; 0];
end

function xplus = gobj(x)
    global lambda
    xplus = [x(1); x(2)+spinEffect(x(7),x(8)); x(3); x(4)+spinEffect(x(7),x(8)); 0; -lambda*x(6); 0; 0];
end

function flow = Cobj(x)
    flow = x(5) >= 0;
end

function jump = Dobj(x)
    jump = x(5) <= 0 && x(6) <= 0;
end

function out = spinEffect(sigmaMin, sigmaMax)
    if(sigmaMin == sigmaMax)
        out = sigmaMax;
    else
        out = mod(rand(), sigmaMin+sigmaMax) - sigmaMin;
    end
end