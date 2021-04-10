% x_a = p, v, R, omega
function [t,j,out] = simHybridVehicleOnly(x, T, M, simTime)
    TSPAN = [0,simTime];
    JSPAN = [0,100];
    HEQOptions = odeset('AbsTol', 1e-1,'RelTol',1e-1);
    [t,j,out] =  HyEQsolver(@f,@g,@C,@D,[x; T; M],TSPAN,JSPAN,1,HEQOptions);
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
    omegadot = -inv(J)*S(x(16:18))*J*x(16:18) + (J^-1 *M);
    out = [pdot; vdot; reshape(Rdot, [9 1]); omegadot; 0; zeros(1,3)];
end

function out = g(x)
    out = x;
end
 
function out = C(x)
    out = 1;
end

function out = D(x)
    out = 0;
end

function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end