function [t, pos, vel, outR, omega] = getQuadState(Client, obj_ID)
    debug = 0;
    if(debug == 1)
    %Set up optitrak
        dllPath = fullfile('c:','Users','yzeleke','Desktop','HSL_exp','NatNetSDK','lib','x64','NatNetML.dll');
        assemblyInfo = NET.addAssembly(dllPath);
        Client = NatNetML.NatNetClientML(0);
        HostIP = char('128.114.56.19');
        Client.Initialize(HostIP, HostIP);
        cleanupObj = onCleanup(@()Client.Uninitialize());
        obj_ID = 1;
    end
    
    R = [-1, 0, 0; 0 0 1; 0 1 0];

    persistent lastPos lastRot lastT
    frameOfData = Client.GetLastFrameOfData();
    obj_pose = frameOfData.RigidBodies(obj_ID);
    t = frameOfData.fTimestamp;
    pos = R*[obj_pose.x;obj_pose.y;obj_pose.z];
    R = R*doubleCover([obj_pose.qw; obj_pose.qx; obj_pose.qy; obj_pose.qz]);
    if isempty(lastPos)
        lastPos = pos;
    end
    if isempty(lastRot)
        lastRot = R;
    end
    if isempty(lastT)
        lastT = t;
    end
    if(t - lastT ~= 0)
        vel = (pos - lastPos)/(t - lastT);
    else
        vel = [0;0;0];
    end
    omega = calcOmega(R, lastRot, t-lastT);
    outR = R*[1 0 0; 0 1 0; 0 0 1]; % need to flip axis to match controller axis definitions
%     t, pos, vel, outR, omega
    % p_x_a; p_y_a; p_z_a; v_x_a; v_y_a; v_z_a; R_11-R_31; R_12-R_32; R_13-R_33; omega; z; h; qhat
    
    
%     if(debug == 1)
%         Client.Uninitialize();
%     end
end

function omega = calcOmega(R, lastR, dt)
    if(dt == 0)
        omega = [0;0;0];
    else
        Somega = lastR\((R-lastR)/dt);
        omega = [-Somega(1,2); Somega(1,3); -Somega(2,3)];
    end
end

function R = doubleCover(q)
    R = eye(3) + 2*q(1)*S(q(2:4)) + 2*S(q(2:4))^2;
end

function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end