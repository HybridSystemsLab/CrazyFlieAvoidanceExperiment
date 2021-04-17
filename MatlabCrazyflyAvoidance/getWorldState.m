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
    
    Rpos = [-1, 0, 0; 0 0 1; 0 1 0];
    Rq = eye(3);
    Rq = [1, 0, 0; 0 -1 0; 0 0 -1];

    persistent lastPos lastRot lastT
    frameOfData = Client.GetLastFrameOfData();
    vehicle_pose = frameOfData.RigidBodies(obj_ID);
    t = frameOfData.fTimestamp;
    pos = Rpos*[vehicle_pose.x;vehicle_pose.y;vehicle_pose.z];
%     [vehicle_pose.qw; vehicle_pose.qx; vehicle_pose.qy; vehicle_pose.qz]
%     R = Rq*Rpos*doubleCover([vehicle_pose.qw; vehicle_pose.qx; vehicle_pose.qy; vehicle_pose.qz]);
    q = [-vehicle_pose.qw; vehicle_pose.qx; vehicle_pose.qz; vehicle_pose.qy];
    R = Rq*doubleCover(q);
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
    outR = R;
    
    
    
    % Identify and report on obstacles
    firstObjPos = 0;
    global sigma_max
    persistent lastObjPos lastObjNum
    if isempty(lastObjPos)
        firstObjPos = 1;
    end
    
%     objPos = zeros(3,frameOfData.nOtherMarkers);
    
    objState = {};
    nskip = 0;
    objPos = [];
    for i = 1:double(frameOfData.nOtherMarkers)
        obj = frameOfData.OtherMarkers(i);
        tempPos = Rpos*[obj.x;obj.y;obj.z];
        if(norm(tempPos - pos) < 0.2)
            nskip = nskip+1;
            continue
        end
        obji = i-nskip;
        objPos(:,obji) = tempPos;
        if(firstObjPos)
            lastObjPos(:,obji) = objPos(:,obji);
        else
            if(lastObjNum < double(frameOfData.nOtherMarkers))
                for j = lastObjNum:double(frameOfData.nOtherMarkers)
                    lastObjPos(:,j) = objPos(:,obji);
                end
            end
        end
        if(t - lastT ~= 0)
            objVel(:,obji) = (objPos(:,obji) - lastObjPos(:,obji))/(t - lastT);
        else
            objVel(:,obji) = [0;0;0];
        end
        objState(:,obji) = mat2cell([0,0,objPos(1,obji),objVel(1,obji),objPos(2,obji),objVel(2,obji),objPos(3,obji),objVel(3,obji), -sigma_max, sigma_max], 1, 10);
    end
    lastObjPos = objPos;
    [~,lastObjNum] = size(objPos);
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