% x_a = p, v, R, omega
function [t,j,out] = simHybridVehicleOnly(x, T, M, simTime)
    TSPAN = [0,simTime];
    JSPAN = [0,10];
    HEQOptions = odeset('AbsTol', 1e-1,'RelTol',1e-1);
    [t,j,out] =  HyEQsolver(@f,@g,@C,@D,[x; T; M],TSPAN,JSPAN,1,HEQOptions);
    if(t(end) < simTime)
        error('Jump Out')
    end
end

function out = f(x)
    T = x(end-3);
    M = x(end-2:end);
    global Mass InertialTensor;
    J = InertialTensor;
    R = reshape(x(7:15), [3 3]);
    pdot = x(4:6);
    vdot = -R*[0;0;1]*(T/Mass) + [0;0;-9.8];
    Rdot = R*S(x(16:18));
    omegadot = -J\S(x(16:18))*J*x(16:18) + (J\M);
    out = [pdot; vdot; reshape(Rdot, [9 1]); omegadot; 0; zeros(3,1)];
end

function out = g(x)
    Rfixing = eye(3);
    R = reshape(x(7:15), [3 3]);
    [Rx,Ry,~] = rot2eul(Rfixing*R);
    Mx = x(end-2);
    My = x(end-1);
    wx = x(end-6);
    wy = x(end-5);
    if(abs(abs(Rx)-180) > 30)
        Mx = 0;
        wx = 0;
    end
    if(abs(Ry) > 30)
        My = 0;
        wy = 0;
    end
    out = [x(1:end-7);wx;wy;x(end-4:end-3);Mx;My;x(end)];
end
 
function out = C(x)
    out = ~D(x);
end

function out = D(x)
    Rfixing = eye(3);
    R = reshape(x(7:15), [3 3]);
    [Rx,Ry,~] = rot2eul(Rfixing*R);
    if((abs(abs(Rx)-180) > 30 && x(end-2) ~= 0) || (abs(Ry) > 30 && x(end-1) ~= 0))
        out = 1;
    else
        out = 0;
    end
end

function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end

function [thetaX, thetaY, thetaZ] = rot2eul(R)
    if(R(3,1)<1)
        if(R(3,1)>-1)
            thetaY = asin(-R(3,1));
            thetaZ = atan2(R(2,1), R(1,1));
            thetaX = atan2(R(3,2), R(3,3));
        else
            thetaY = pi/2;
            thetaZ =-atan2(-R(2,3) , R(2,2));
            thetaX = 0;
        end
    else
        thetaY = -pi /2;
        thetaZ = atan2(-R(2,3) , R(2,2));
        thetaX = 0;
    end
    thetaX = rad2deg(thetaX);
    thetaY = rad2deg(thetaY);
    thetaZ = rad2deg(thetaZ);
end