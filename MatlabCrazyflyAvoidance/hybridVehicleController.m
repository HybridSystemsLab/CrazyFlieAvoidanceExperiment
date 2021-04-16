% x_a = [p_x_a; p_y_a; p_z_a; v_x_a; v_y_a; v_z_a; R_11-R_31; R_12-R_32; R_13-R_33; omega; z; h; qhat_1; t]
% r_p = 1:3
% r_v = 4:6
% r_a = 7:9
% r_j = 10:12
% r_s = 13:15
% r_R = 16:24
% r_omega = 25:27
% r_omeagadot = 28:30
function [T,M] = hybridVehicleController(r, x, t)
    global QUAD_REFERENCE lastState
    
    persistent zdot
    
    if(isempty(lastState))
        lastState = [x;0];
    end
    if(isempty(zdot))
        zdot = zeros(3,1);
    end
    dt = t-lastState(end);
    x = updateIntegralState(x, dt, zdot);
    
    QUAD_REFERENCE = r;
    OutOfCD = 1;
    if(D([x; t]))
        x = g([x; t]);
        OutOfCD = 0;
    end
    if(C([x; t]))
        [T,M,zdot] = f([x; t], dt);
        OutOfCD = 0;
    else
        disp("NO FLOW")
        T = -1;
        M = [0;0;0];
    end
    if(OutOfCD)
        error("Not in C \cup D");
    end
    lastState = [x;t];
end

function xOut = updateIntegralState(x, dt, zdot)
    xOut = [x(1:18); x(19:21) + dt*zdot; x(22:end)];
end

function [T,M,zdot] = f(x,dt)
    global InertialTensor Mass k k_omega k_z MaxMotorThrust d_xy
    persistent lastomega_0 lastp_0 lastv_0
    
    omega_0 = omega_0Func(x);
    
    if(isempty(lastomega_0))
        lastomega_0 = omega_0;
    end
    
    R = reshape(x(7:15),3,3);
    r = getReference(x(end));
    R_r = reshape(r(16:24),3,3);
    mu = muFunc(x);
    R_0 = makeR0(R_r, mu);
    
%     dt = x(end) - lastt;
    domega_0 = omega_0 - lastomega_0;
    domega_0dt = domega_0/dt;
    if(any(isnan(domega_0dt)|isinf(domega_0dt)|(abs(dt)<1e-18&abs(domega_0dt)>1e6)))
        domega_0dt = [0; 0; 0];
    end
    lastomega_0 = omega_0;

    torque = S(x(16:18))*InertialTensor*x(16:18)+((InertialTensor)*(-k_omega*(x(16:18)-omega_0)-k*x(20)*R_0'*[1; 0; 0] + domega_0dt));
    motorDiff = min(MaxMotorThrust - min(norm(mu)*Mass/4, MaxMotorThrust), norm(mu)*Mass/4);
    maxSingleTorque = (2^0.5 * d_xy) * motorDiff;
    torqueInQuadFrame = R*torque;
    maxTorqueNorm = maxSingleTorque*(2^0.5)*sin(pi/4)/sin(pi/4 - atan2(torqueInQuadFrame(1),torqueInQuadFrame(2)));
    if(norm(torque(1:2)) > maxTorqueNorm)
        torque = maxTorqueNorm*torque/norm(torque);
    end
    
    if(any(isnan(torque)))
        global QUAD_REFERENCE
        disp(QUAD_REFERENCE)
        error("Torque is NAN");
    end
    
    p_0 = r(1:3) - x(1:3);
    v_0 = r(4:6) - x(4:6);
    if(isempty(lastp_0))
        lastp_0 = p_0;
    end
    if(isempty(lastv_0))
        lastv_0 = v_0;
    end
    
%     pdot = x(4:6);
%     vdot = mu + [0;0;-9.8];
%     Rdot = reshape(R*S(x(16:18)),9,1);
%     omegadot = (-inv(InertialTensor)*S(x(16:18))*InertialTensor*x(16:18) + inv(InertialTensor)*torque);
    dV_0_bar = (V_0_bar(p_0, v_0)-V_0_bar(lastp_0, lastv_0));
    dv_0 = (v_0-lastv_0);
    derivativePart =[dV_0_bar/dv_0(1);dV_0_bar/dv_0(2);dV_0_bar/dv_0(3)];
    derivativePart(isnan(derivativePart)|isinf(derivativePart)|(abs(dv_0)<1e-18&abs(derivativePart)>1e6)) = 0;
    
    zdot = k_z*(derivativePart);
    T = Mass*norm(mu);
    M = torque;
end

function out = g(x)
    if(D_1(x) == 1 && D_2(x) == 1)   
        R = reshape(x(7:15),3,3);
        G_c = [x(19:21);-x(22);argmaxP(R, x(23:26))];
    elseif(D_1(x) == 1)
        G_c = [x(19:21);-x(22);x(23:26)];
    else
        R = reshape(x(7:15),3,3);
        G_c = [x(19:21);x(22);argmaxP(R, x(23:26))];
    end
    out = [x(1:18); G_c];
end

function out = C(x)
    global Qdelta alpha
    R = reshape(x(7:15),3,3);
    r = getReference(x(end));
    
    R_r = reshape(r(16:24),3,3);
    mu = muFunc(x);
    R_0 = makeR0(R_r, mu);
    a = argmaxP(R*R_0', x(23:26));
    if(a(1)*x(22) >= -Qdelta) % a \in Q_delta^+
        out1 = 1;
    else
        out1 = 0;
    end
    Q = Qfunc(R*R_0');
    out2 = norm(x(23:26)'*Q(:,1), Inf) <= alpha;
    out = out1 & out2;
    if(~out)
        a(1)*x(22)
        norm(x(23:26)'*Q(:,1), Inf)
    end
end

function out = D(x)
    d1val = D_1(x);
    d2val = D_2(x);
    out = d1val | d2val;
end

function out = D_1(x)
    global Qdelta
    R = reshape(x(7:15),3,3);
    r = getReference(x(end));
    R_r = reshape(r(16:24),3,3);
    
    mu = muFunc(x);
    R_0 = makeR0(R_r, mu);
    a = argmaxP(R*R_0', x(23:26));
    D_1_eval = a(1)*x(22);
    if(D_1_eval <= -Qdelta) % a \in Q_delta^-
        out = 1;
    else
        out = 0;
    end
end

function out = D_2(x)
    global alpha;
    
    r = getReference(x(end));
    R = reshape(x(7:15),3,3);
    R_r = reshape(r(16:24),3,3);
    
    mu = muFunc(x);
    R_0 = makeR0(R_r, mu);
    Q = Qfunc(R*R_0');
    out = norm(x(23:26)'*Q(:,1), Inf) >= alpha;
end

%R determines set Q(R)
function out = argmaxP(R, qhat)
    Q = Qfunc(R);
    out = Q(:,1);
    if(qhat'*Q(:,1) < qhat'*Q(:,2))
        out = Q(:,2);
    end
end

function out = Qfunc(R)
    % From https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    tr = trace(R);
    qw = 1;
    qx = 0;
    qy = 0;
    qz = 0;
    if(tr>0)
        S = (tr+1)^0.5 *2;
        qw = S/4;
        qx = (R(3,2) - R(2,3))/S;
        qy = (R(1,3) - R(3,1))/S;
        qz = (R(2,1) - R(1,2))/S;
    elseif ((R(1,1) > R(2,2))&&(R(1,1) > R(3,3)))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
        qw = (R(3,2) - R(2,3)) / S;
        qx = 0.25 * S;
        qy = (R(1,2) + R(2,1)) / S; 
        qz = (R(1,3) + R(3,1)) / S; 
    elseif (R(2,2) > R(3,3))
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
        qw = (R(1,3) - R(3,1)) / S;
        qx = (R(1,2) + R(2,1)) / S; 
        qy = 0.25 * S;
        qz = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
        qw = (R(2,1) - R(1,2)) / S;
        qx = (R(1,3) + R(3,1)) / S;
        qy = (R(2,3) + R(3,2)) / S;
        qz = 0.25 * S;
    end
    quat = [qw, qx, qy, qz];
    
    out = [quat/norm(quat); -quat/norm(quat)]';
end

function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end

function mu = muFunc(x_a)
    global Mass MaxMotorThrust
    r = getReference(x_a(end));
    a_r = r(7:9);
    r_tildae = r_tildae_Func(x_a);
    mu = sumK(r_tildae) - sumK(x_a(19)) - [0;0;-9.8] + a_r;
    if(norm(mu)*Mass > MaxMotorThrust*4)
%         disp('Over max thrust')
%         disp(norm(mu)*Mass/(MaxMotorThrust*4))
        mu = MaxMotorThrust*4*mu/norm(mu);
    end
end

function R_0 = makeR0(R_r, mu)
    muOverMuNorm = mu/norm(mu);
    if(abs(muOverMuNorm)<1e-12)
        disp('In makeR0, very small muOverMuNorm')
        muOverMuNorm
        pause
    end
    gamma = -S(R_r*[0;0;1])*muOverMuNorm;
    squareTerm = ((S(gamma)^2)/(1-[0;0;1]'*R_r'*muOverMuNorm));
    if(gamma == zeros(3,1))
        squareTerm = zeros(3,3);
    end
    R_0 = (eye(3) + S(gamma)+ squareTerm)*R_r;
end

function r_tildae = r_tildae_Func(x_a)
    global k_p k_v
    r = getReference(x_a(end));
    p_0 = r(1:3) - x_a(1:3);
    v_0 = r(4:6) - x_a(4:6);
    r_tildae = k_p*p_0 + k_v*v_0;
end

function out = sigma_K(x)
    global K
    out = (2*K/pi)*atan(x);
end

function out = sigma_K_Integral(x)
    global K
    out = K*(2*x*atan(x)-log(x^2+1))/pi;
end

function out = sumK(x)
    [r,~] = size(x);
    out = zeros([r,1]);
    for i = 1:r
        out(i) = sigma_K(x(i));
    end
end

function out = omega_0Func(x)
    global k_z k_V_0 k k_q xi_a
    global omegaMapReset
    
    persistent omega_0FuncVals
    if(isempty(omega_0FuncVals) || omegaMapReset == 1)
        omega_0FuncVals = [];
    end
    
    persistent lastMapOmegaFunc 
    if (isempty(lastMapOmegaFunc) || omegaMapReset == 1)
        omegaMapReset = 0;
        tempr = getReference(0);
        tempmu = muFunc(x);
        lastMapOmegaFunc = [0, (tempr(1:6) - xi_a(1:6))', xi_a(19:21)', reshape(makeR0(reshape(tempr(16:24),3,3), tempmu), [1,9])]';
    end
    if(lastMapOmegaFunc(1,end) < x(end))
        lastVals = lastMapOmegaFunc(:,end);
        lastt = lastVals(1);
        lastp_0 = lastVals(2:4);
        lastv_0 = lastVals(5:7);
        lastz = lastVals(8:10);
        lastR_0 = reshape(lastVals(11:19),[3 3]);
    elseif(any(size(lastMapOmegaFunc(1,:)) >= 2))
        lastVals = interp1(lastMapOmegaFunc(1,:)', lastMapOmegaFunc', x(end), 'previous')';
        if(lastVals(1) == x(end))
            lastVals = interp1(lastMapOmegaFunc(1,:)', lastMapOmegaFunc', x(end)-min(1e-9, 1e-9*x(end)), 'previous')';
        end
        lastt = lastVals(1);
        lastp_0 = lastVals(2:4);
        lastv_0 = lastVals(5:7);
        lastz = lastVals(8:10);
        lastR_0 = reshape(lastVals(11:19),[3 3]);
    else
        tempr = getReference(0);
        tempmu = muFunc(x);
        lastt = 0;
        lastp_0 = (tempr(1:3) - xi_a(1:3));
        lastv_0 = (tempr(4:6) - xi_a(4:6));
        lastz = xi_a(19:21);
        lastR_0 = makeR0(reshape(tempr(16:24),3,3), tempmu);
    end
    
    r = getReference(x(end));
    R_r = reshape(r(16:24),3,3);
    mu = muFunc(x);
    R = reshape(x(7:15),3,3);
    R_0 = makeR0(R_r, mu);
    q = argmaxP(R*R_0', x(23:26));
    p_0 = r(1:3) - x(1:3);
    v_0 = r(4:6) - x(4:6);
    dV_0 = (V_0(p_0, v_0,x(19:21))-V_0(lastp_0, lastv_0, lastz));
    dv_0 = (v_0-lastv_0);
    D_v0V0 =[dV_0/dv_0(1);dV_0/dv_0(2);dV_0/dv_0(3)];
    D_v0V0(isnan(D_v0V0)|isinf(D_v0V0)|(abs(dv_0)<1e-18&abs(D_v0V0)>1e6)) = 0;
    omega_0_star = (2*k_z*k_V_0/(k*x(22))) * (q(1)*S(mu)-S(mu)*S(q(2:4)))* D_v0V0;
    timeDelta = x(end)-lastt;
    dR_dt = ((reshape(R_0-lastR_0, 9,1))/timeDelta);
    if(any(isnan(dR_dt)|isinf(dR_dt)|(abs(timeDelta)<1e-18&abs(dR_dt)>1e6)))
        dR_dt = zeros(9,1);
    end
    out = -0.5*R_0'*(gamma(R_0)'*dR_dt) + R_0'*(-omega_0_star - k_q*x(22)*q(2:4));
    if(~ismember(lastMapOmegaFunc(1,:),x(end)))
        lastMapOmegaFunc(:,end+1) = [x(end), p_0', v_0', x(19:21)', reshape(R_0, [1, 9])]';
    else
        [locMap, ~] = ismember(lastMapOmegaFunc(1,:),x(end));
        lastMapOmegaFunc(:,locMap) = [x(end), p_0', v_0', x(19:21)', reshape(R_0, [1, 9])]';
    end
    omega_0FuncVals(end+1,:) = [x(end); out];
end

function out = V_0_bar(p0, v0)
    global beta k_p k_v
    P = [k_v/k_p * beta, -beta; -beta, k_p];
    r_tildae = k_p*p0 + k_v*v0;
    out = 0;
    evec = eye(3);
    v0 = reshape(v0, [3 1]);
    for i = 1:3
        out = out + 0.5 * [sigma_K(r_tildae(i)) evec(i,:)*v0] * P * [sigma_K(r_tildae(i)); evec(i,:)*v0] + (-sigma_K_Integral(0) + sigma_K_Integral(r_tildae(i)));
    end
end

function out = gamma(R)
    out = [S(R*[1;0;0]) S(R*[0;1;0]) S(R*[0;0;1])]';
end

%Assumed b = [0,0,0]
function out = V_0(p0,v0,z)
    global k_z
    sumPart = 0;
    for i = 1:3
        sumPart = sumPart + (sigma_K_Integral(norm(z(i))) - sigma_K_Integral(0));
    end
    out  = k_z * V_0_bar(p0,v0) + sumPart;
end

function out = getReference(t)
    global QUAD_REFERENCE
    if(t < QUAD_REFERENCE(end,1))
        out = interp1(QUAD_REFERENCE(:,1),QUAD_REFERENCE(:,2:end),t)';
    else
        out = QUAD_REFERENCE(end,2:end)';
    end
end