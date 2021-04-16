function [phis,rs] = genSafeTraj(x_a, U)
    global usingPointMassVehicle objColStep ObjPlot55 spx spy spz spherePts 
    
    if(ObjPlot55)
        f56 = figure(56);
        clf(f56);
    end
    
    Mset = [];
%     spPoints = sphere(spherePts);
    global useMotionPrim executeTime
    if(useMotionPrim)
        [Mset,all_rs] = motionPrimMobility(x_a);
    else
        if(~isempty(usingPointMassVehicle) && usingPointMassVehicle == 1)
            %Generate trajectories
            [Mset,all_rs] = pointMassMobility(x_a);
        else
            [Mset,all_rs] = hybridModelMobility(x_a);
        end
    end
    numSafe = 0;
    numObj = size(U);
    %Remove unsafe sets
    numTraj = size(Mset);
%     phis = {};
%     rs = {};
    safeVals = [];
    tess = cell(numTraj(1),numObj(1),2);
    xosCell = cell(numObj(1),1);
    for i = 1:numTraj(1)
        unsafeFlag = 0;
        if(useMotionPrim)
            traj = cell2mat(Mset(i)) + [0; 0; x_a(1:3) + x_a(4:6)*executeTime; x_a(4:6); zeros(12,1)]';
        else
            traj = cell2mat(Mset(i));
        end
        for j = 1:numObj(1)
            xos = [];
            if(i == 1)
                x_o = cell2mat(U(j,1));
                [~,u_t,~] = unique(x_o(:,1));
                xos(:,:,1) = x_o(u_t,:);
                % Resample obstacle trajectories to match time stamps of first obstacle trajectory
                for k = 2:4
                    x_o = cell2mat(U(j,k));
                    [~,u_t,~] = unique(x_o(:,1));
                    u_x_o = x_o(u_t,:);
                    u_x_o_size = size(u_x_o);
                    if(u_x_o_size(1) == 1)
                        xos(:,:,k) = u_x_o;
                    else
                        xos(:,:,k) = interp1(u_x_o(:,1), u_x_o, xos(:,1,1));
                    end
                end
                xosCell{j} = xos;
            else
                xos = xosCell{j};
            end
            
            if(i == 1)
                numXos = size(xos(:,1,1));
                objColStepMaxed = min(objColStep, floor(numXos(1)/2));
                hullPts = zeros(spherePts*(spherePts-1)*(4*(objColStepMaxed+1)),3);
                for t = 1:max(objColStepMaxed,1):max(numXos(1)-objColStepMaxed,1)
                    %Unroll points in sphere arond each point
                    pt_count = 0;
                    % Build convext hull over subset of obstacle samapled points
                    for k = 1:4
                        [sp_num_pts,~] = size(spx);
                        for l = 0:objColStepMaxed
                            hullPts(pt_count+1:pt_count+sp_num_pts,:) = [spx,spy,spz]+[xos(t+l,3,k),xos(t+l,5,k),xos(t+l,7,k)];
                            pt_count = pt_count + sp_num_pts;
                        end
                        if(i == 1 && ObjPlot55 == 1)
                            figure(55);
                            hold on;
                            color = ['r','g','b','k'];
                            hullPtsNotOrig = (hullPts(:,1:3) ~= [0 0 0]);
                            scatter3(hullPts(hullPtsNotOrig(:,1),1),hullPts(hullPtsNotOrig(:,1),2),hullPts(hullPtsNotOrig(:,1),3),color(k));
                            figure(56);
                            hold on
                            plot3(hullPtsNotOrig(:,1), hullPtsNotOrig(:,2), hullPtsNotOrig(:,3));
                        end
                    end
                
                % Check if any trajectory point is in hull
%                     a = tic;
                    tempTess = convhulln(hullPts);
                    [r,c] = size(tempTess);
                    [rH,cH] = size(hullPts);
                    tess(j,t,:) = [mat2cell(hullPts,rH,cH), mat2cell(tempTess,r,c)];
%                     cell2mat(tess(j,t));
%                     toc(a)
                end
%                 ptstic = tic;
                hullTess = tess(j,t,:);
                ptsInHull = inhull(traj(:,3:5),hullTess{1}, hullTess{2});
%                 ptstocs(i,j,t) = toc(ptstic);
                if(sum(ptsInHull) > 0)
                    unsafeFlag = 1;
                    break
                end
            end
        end
        
        if unsafeFlag == 0
            % Does not collide with any
            numSafe = numSafe+1;
%             phis(numSafe) = Mset(i);
%             rs(numSafe) = all_rs(i);
            safeVals(numSafe) = i;
        end
    end
    phis = Mset(safeVals);
    rs = all_rs(safeVals);
end

% returns path with structure [t,p_x, p_y, p_z, v_x, v_y, v_z, R(3x3), omeag(3x1)]
function [trajs,rs] = pointMassMobility(x_a)
    global planTime planStep maxAcc
%Discritize reachable space
    sampleAngs = 20; %number of angles to test
    sampleAccSteps = 5; %number of distances to test at each angle

    angstep = 2*pi/sampleAngs;
    numTraj = 0;
    trajs = cell(ceil(pi/angstep)*sampleAngs*sampleAccSteps,1);
    rs = cell(ceil(pi/angstep)*sampleAngs*sampleAccSteps,1);
%     figure;
    for i = (-pi/2):angstep:(pi/2)
        for j = angstep:angstep:2*pi
            for k = 1:sampleAccSteps
                r_x = [maxAcc/k * cos(i) * cos(j), maxAcc/k * cos(i) * sin(j), maxAcc/k * sin(i), 0,0,0];
                r = [(planStep:planStep:planTime)', repmat(r_x, [size((planStep:planStep:planTime)'),1])];
                if(any(isnan(r(:))))
                    r
                    r_x
                    pause
                end
                [time,jump,phi] = pointMassVehicle(r, x_a, planTime);
%                 hold on;
%                 plot3(phi(:,1), phi(:,2), phi(:,3),'b');
                if(any(isnan(phi(:))))
                    r
                    r_x
                    phi
                    pause
                end
                [row,col] = size([time,jump,phi]);
                numTraj = numTraj+1;
                trajs(numTraj) = mat2cell([time,jump,phi],row,col);
                [row,col] = size(r);
                rs(numTraj) = mat2cell(r,row,col);
            end
        end
    end
%     safeTrajectories = cell(1,numSample);
%     for i = 1:numSample
%         path(:,1) = time + linspace(0,planTime,pathSamples);
%         path(:,2) = linspace(xa(1), samples(1,i), pathSamples);
%         path(:,3) = linspace(xa(2), samples(1,i), pathSamples);
%         path(:,4) = linspace(xa(3), samples(1,i), pathSamples);
%         path(:,5:7) = repmat(transpose((samples(1:3,i) - xa(1:3))/planTime), pathSamples,1);
%         path(:,8:16) = repmat([1,0,0,0,1,0,0,0,1],pathSamples,1);
%         path(:,17:19) = repmat([0,0,0],pathSamples,1);
%         
%         safeTrajectories(i) = mat2cell(path(2:end,:), pathSamples-1,19);
%     end
end

function [trajs, rs] = hybridModelMobility(x_a)
    global d_xy MaxMotorThrust planTime FirstRunEject ConstThrust PlanningThrustMult
    numMVals = 12; % +1
    
    %Generate reference trajs for vehicle
    persistent TMs i
    if(FirstRunEject == 1)
        disp("first run")
        FirstRunEject = 2;
        TMs = zeros(4,1);
        m1 = 1;
        m2 = 1;
        m3 = 1;
        m4 = 1;
        roll = (-m1-m2+m3+m4)*MaxMotorThrust*d_xy;
        pitch = (+m1-m2-m3+m4)*MaxMotorThrust*d_xy;
        yaw = (m1-m2+m3-m4)*MaxMotorThrust*d_xy;
        thrust = (m1+m2+m3+m4)*MaxMotorThrust;
        i = 1;
        TMs(:,i) = [thrust, roll, pitch, yaw];
        
        pause
    elseif(isempty(TMs)||FirstRunEject == 2)
        disp("genning TM ball")
        FirstRunEject = 0;
        TMs = zeros(4, 2+(numMVals-1)*(numMVals+1)^2);
        i = 2;
%         for m1=0:1/numMVals:1
%             for m2=0:1/numMVals:1
%                 for m3=0:1/numMVals:1
%                     for m4=0:1/numMVals:1
%                         roll = (-m1-m2+m3+m4)*MaxMotorThrust*0.9*d_xy;
%                         pitch = (+m1-m2-m3+m4)*MaxMotorThrust*0.9*d_xy;
%                         yaw = (m1-m2+m3-m4)*MaxMotorThrust*0.9*d_xy;
%                         thrust = (m1+m2+m3+m4)*MaxMotorThrust*0.9;
%                         if(yaw ~= 0)
%                             continue;
%                         end
%                         i = i+1;
%                         TMs(:,i) = [thrust, roll, pitch, yaw];
%                     end
%                 end
%             end
%         end
        TMs(:,1) = [0; 0; 0; 0];
%         TMs(:,2) = [4*MaxMotorThrust; 0; 0; 0];
        for trst=1/(numMVals+2):1/(numMVals+2):(1-1/(numMVals+2))
            diffVals = min(1-trst, trst)/2; % base is over 2
            for m1m3diff=-diffVals:2*diffVals/numMVals:diffVals
                for m2m4diff=-diffVals:2*diffVals/numMVals:diffVals
                    m1 = (trst+m1m3diff)*PlanningThrustMult;
                    m2 = (trst+m2m4diff)*PlanningThrustMult;
                    m3 = (trst-m1m3diff)*PlanningThrustMult;
                    m4 = (trst-m2m4diff)*PlanningThrustMult;
                    if(any([m1 m2 m3 m4] > 1)|| any([m1 m2 m3 m4] < 0))
                        disp('out of motor range')
                        pause
                        continue
                    end
                    roll = (-m1-m2+m3+m4)*MaxMotorThrust*d_xy;
                    pitch = (+m1-m2-m3+m4)*MaxMotorThrust*d_xy;
                    yaw = (m1-m2+m3-m4)*MaxMotorThrust*d_xy;
                    thrust = (m1+m2+m3+m4)*MaxMotorThrust;
%                     pause
                    if(abs(yaw) <= 1e-6)
                        yaw = 0;
                    else
                        disp('bad yaw');
                        pause
                        continue
                    end
                    i = i+1;
                    TMs(:,i) = [thrust; roll; pitch; yaw];
                end
            end
        end
    else
        disp("No gen, using last TM ball")
    end
%     pause
%     disp(TMs)
%     disp(i)
    disp("finished ref gen");
%     pause
    numTraj = 0;
    trajs = cell(i,1);
    rs = cell(i,1);
    figure(76)
    hold on;
    for j = 1:i
        % Find reference trajectories
        if(ConstThrust == 1)
            PitchRollTime = planTime/3;
            [t,simJump,r] = simHybridVehicleOnly(x_a(1:18)', TMs(1,j), TMs(2:4,j), [0 PitchRollTime]);
            [t2,simJump2,r2] = simHybridVehicleOnly(r(end,1:end-4)', TMs(1,j), [0; 0; 0], [PitchRollTime planTime]);
            t = [t;t(end)+t2(3:end)];
            simJump = [simJump;simJump(end)+simJump2(3:end)];
            r = [r;r2(3:end,:)];
%             UsimVals = unique([t,simJump,r],'rows');
%             t = UsimVals(:,1);
%             simJump = UsimVals(:,2);
%             r = UsimVals(:,3:end);
        else
            [t,simJump,r] = simHybridVehicleOnly(x_a(1:18)', TMs(1,j), TMs(2:4,j), planTime);
        end
        r = r(:,1:18);
        if(t(1) == t(2))
            t = t(2:end);
            simJump = simJump(2:end);
            r = r(2:end,:);
        end
        %Use hybrid sym as traj out put
        phi = r;
        jump = simJump;
        time = t;
        
        dt = gradient(t);
        plot3(r(:,1),r(:,2),r(:,3));
        pose = reshape(r(end,7:15),[3 3])*[0; 0; 0.01];
        plot3([r(end,1),pose(1)+r(end,1)], [r(end,2),pose(2)+r(end,2)], [r(end,3),pose(3)+r(end,3)], 'r')
        [~,dv] = gradient(r(:,4:6));
        vdot = dv./dt;
        numrpts = size(t);
        adot = zeros(numrpts(1),3);
        jdot = zeros(numrpts(1),3);
        [~,dw] = gradient(r(:,16:18));
        wdot = dw./dt;
        r = [t, r(:,1:6), vdot, adot, jdot, r(:,7:end), wdot];
        %Run reference trajectories through controller
%         [time,jump,phi] = hybridVehicle(r, x_a, planTime,1);
        %Check if jumped out
%         if(jump(end) == 50)
%             continue
%         end
        %Save reference and resulting trajectories
        [row,col] = size([time,jump,phi]);
        numTraj = numTraj+1;
        trajs(numTraj) = mat2cell([time,jump,phi],row,col);
        [row,col] = size(r);
        rs(numTraj) = mat2cell(r,row,col);
    end
end

function [Mset,all_rs] = motionPrimMobility(x_a)
    global motionPrims executeTime

    [~,~,yawAng] = rot2eul(reshape(x_a(7:15),[3,3]));
    yawRot = [cos(yawAng) -sin(yawAng) 0; sin(yawAng) cos(yawAng) 0; 0 0 1];
    searchRot = reshape(yawRot*reshape(x_a(7:15),[3,3]),[1,9]);
    entryNum = dsearchn(motionPrims.motionPoints(1:9,:)', searchRot);
    tempCell = motionPrims.motionPrimatives(entryNum);
    Mset = tempCell{1}(:,1);
    all_rs = tempCell{1}(:,2);
%     [row col] = size(tempCell{1});
%     all_rs = cell(row, 1);
%     Mset = cell(row, 1);
    % Need to add acceleration compensation
%     for i = 1:row
%         tempMset = cell2mat(tempCell{1}(i,1));
%         [r,c] = size(tempMset);
%         Mset(i) = mat2cell(tempMset + [0; 0; x_a(1:3) + x_a(4:6)*executeTime; x_a(4:6); zeros(12,1)]', r,c);
%         tempRs = cell2mat(tempCell{1}(i,2));
%         [r,c] = size(tempRs);
%         all_rs(i) = mat2cell(tempRs + [0; x_a(1:3) + executeTime*x_a(4:6); x_a(4:6); zeros(24,1)]', r,c);
%     end
%     all_rs
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