function [t, pos, vel, outR, omega, objState] = getWorldState(Client, obj_ID)
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
    vehicle_pose = frameOfData.RigidBodies(obj_ID);
    t = frameOfData.fTimestamp;
    pos = R*[vehicle_pose.x;vehicle_pose.y;vehicle_pose.z];
    R = R*doubleCover([vehicle_pose.qw; vehicle_pose.qx; vehicle_pose.qy; vehicle_pose.qz]);
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
    
    
    
    % Identify and report on obstacles
    firstObjPos = 0;
    global sigma_max
    persistent lastObjPos lastObjNum
    if isempty(lastObjPos)
        firstObjPos = 1;
    end
    
    objPos = zeros(3,frameOfData.nOtherMarkers);
    
    objState = {};
    for i = 1:frameOfData.nOtherMarkers
        obj = frameOfData.OtherMarkers(i);
        objPos(:,i) = R*[obj.x;obj.y;obj.z];
        if(firstObjPos)
            lastObjPos(:,i) = objPos(:,i);
        else
            if(lastObjNum < double(frameOfData.nOtherMarkers))
                for j = lastObjNum:double(frameOfData.nOtherMarkers)
                    lastObjPos(:,j) = objPos(:,i);
                end
            end
        end
        if(t - lastT ~= 0)
            objVel(:,i) = (objPos(:,i) - lastObjPos(:,i))/(t - lastT);
        else
            objVel(:,i) = [0;0;0];
        end
        objState(:,i) = mat2cell([0,0,objPos(1,i),objVel(1,i),objPos(2,i),objVel(2,i),objPos(3,i),objVel(3,i), -sigma_max, sigma_max], 1, 10);
    end
    lastObjPos = objPos;
    lastObjNum = double(frameOfData.nOtherMarkers);
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