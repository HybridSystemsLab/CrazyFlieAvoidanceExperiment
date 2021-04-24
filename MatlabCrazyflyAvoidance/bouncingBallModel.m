function [tout, jout, xout] = bouncingBallModel(x0, simTime)
%     global HEQOptions
%     TSPAN = [0, simTime];
%     JSPAN = [0, max(10,2*simTime)]; % allow two bounces per second
%     [tout,jout,xout] = HyEQsolver(@fobj,@gobj,@Cobj,@Dobj,x0,TSPAN,JSPAN,1,HEQOptions);
    timeStep = 0.05;
    x = x0;
    t = 0;
    j = 0;
    tjx = zeros(floor(simTime/timeStep) +1, 10);
    tjx(1,:) = [t,j,x0];
    for i = 2:floor(2*simTime/timeStep)+1
        if(Dobj(x'))
            x = gobj(x')';
            j = j+1;
            tjx(i,:) = [t,j,x];
        elseif(Cobj(x))
            x = fobj(x)'*timeStep + x;
            t = t+timeStep;
            tjx(i,:) = [t,j,x];
        else
            error('bbal out of CD');
        end
    end
    tout = tjx(:,1);
    jout = tjx(:,2);
    xout = tjx(:,3:end);
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
    xplus = [x(1); x(2)+spinEffect(x(7),x(8)); x(3); x(4)+spinEffect(x(7),x(8)); 0; -lambda*x(6); x(7:8)];
end

function flow = Cobj(x)
    flow = x(5) >= 0;
end

function jump = Dobj(x)
    jump = x(5) < 0 || (x(5) == 0 && x(6) <= 0);
end

function out = spinEffect(sigmaMin, sigmaMax)
    if(sigmaMin == sigmaMax)
        out = sigmaMax;
    else
        out = mod(rand(), sigmaMin+sigmaMax) - sigmaMin;
    end
end