function out = setBasedPlanner(x_a, x_o, costFun)
    global planTime sigma_max executeTime useMotionPrim
    %Generate unsafe sets for obstacles
    xSpin = [sigma_max, sigma_max, -sigma_max, -sigma_max];
    ySpin = [sigma_max, -sigma_max, sigma_max, -sigma_max];
    numObj = size(x_o);
    U = cell(numObj(1),numObj(2)*4);
    for i=1:numObj(1)
        for j = 1:4
            x_o_mat = cell2mat(x_o(i));
            x_o_mat_size = size(x_o_mat);
            x_o_mat(:,9:10) = repmat([xSpin(j), ySpin(j)], [x_o_mat_size(1), 1]);
            [bb_t, bb_j, bb_x] = bouncingBallModel(x_o_mat(end,end-7:end), planTime);
            if(bb_j(end) == 0 && j == 1)
                bballOut = [bb_t, bb_j, bb_x];
                [row, col] = size(bballOut);
                U(i,1) = mat2cell(bballOut,row,col);
                for k = 2:4
                    U(i,k) = U(i,1);
                end
                break
            end
            bballOut = [bb_t, bb_j, bb_x];
            [row, col] = size(bballOut);
            U(i,j) = mat2cell(bballOut,row,col);
        end
    end
    %Generate safe reachable map
    [phi,r] = genSafeTraj(x_a,U);
    global plot77
    if(plot77)
        f77 = figure(77);
        clf(f77)
        numRef = size(r);
        hold on;
        for i = 1:numRef(1)
            ref = cell2mat(r(i));
            if(useMotionPrim)
                [~,~,yawAng] = rot2eul(x_a(7:15));
                yawRot = [cos(yawAng) -sin(yawAng) 0; sin(yawAng) cos(yawAng) 0; 0 0 1];
                [rr, ~] = size(ref);
                spunr = [ref(:,1), ref(:,2:4)*yawRot', ref(:,5:7)*yawRot', ref(:,8:10)*yawRot', ref(:,11:13)*yawRot', ref(:,14:16)*yawRot', zeros(rr,9), ref(:,26:31)];
                for ri = 1:rr
                    spunr(ri,17:25) = reshape(yawRot*reshape(ref(ri,17:25),[3,3]),[1,9]);
                end
                ref = spunr + [zeros(rr,1), x_a(1:3)' + x_a(4:6)'.*ref(:,1), repmat(x_a(4:6)', [rr,1]), zeros(rr,24)];
            end
%             plot3(ref(:,2),ref(:,3),ref(:,4));
            scatter3(ref(end,2),ref(end,3),ref(end,4));
        end
        xlabel('x');
        ylabel('y');
        zlabel('z');
        daspect([1 1 1])
    end
    
    %Find "optimal" trajectory
    [numTraj,~] = size(phi);
    if(numTraj == 0)
        error("No safe trajectories, phi:",phi)
    end
    if(useMotionPrim)
        temppath = cell2mat(phi(1));
        [~,~,yawAng] = rot2eul(x_a(7:15));
        yawRot = [cos(yawAng) -sin(yawAng) 0; sin(yawAng) cos(yawAng) 0; 0 0 1];
        [rr, ~] = size(temppath);
        spunpath = [temppath(:,1:2), temppath(:,3:5)*yawRot', temppath(:,6:8)*yawRot', zeros(rr,9), temppath(:,18:20)];
%         for ri = 1:rr
%             spunpath(ri,9:17) = reshape(yawRot*reshape(temppath(ri,9:17),[3,3]),[1,9]);
%         end
        path = spunpath + [zeros(rr,2), x_a(1:3)' + x_a(4:6)'.*temppath(:,1), repmat(x_a(4:6)', [rr,1]), zeros(rr,12)];
%         [pr, pc] = size(temppath);
%         path = temppath + [zeros(pr,2), x_a(1:3)' + x_a(4:6)'.*temppath(:,1), repmat(x_a(4:6)', [pr,1]), zeros(pr,12)];
        if(any(isnan(path)))
            disp('bad path');
        end
    else
        path = cell2mat(phi(1));
    end
    closestPathVals = path(end,3:5);
%     closestTraj = [1, costFun(path(end,3:5), path(end,6:8), path(end,9:17))];
    closestTraj = [1, costFun(path)];
    for i=2:numTraj
        if(useMotionPrim)
            temppath = cell2mat(phi(i));
            [~,~,yawAng] = rot2eul(x_a(7:15));
            yawRot = [cos(yawAng) -sin(yawAng) 0; sin(yawAng) cos(yawAng) 0; 0 0 1];
            [rr, ~] = size(temppath);
            spunpath = [temppath(:,1:2), temppath(:,3:5)*yawRot', temppath(:,6:8)*yawRot', zeros(rr,9), temppath(:,18:20)];
%             for ri = 1:rr
%                 spunpath(ri,9:17) = reshape(yawRot*reshape(temppath(ri,9:17),[3,3]),[1,9]);
%             end
            path = spunpath + [zeros(rr,2), x_a(1:3)' + x_a(4:6)'.*temppath(:,1), repmat(x_a(4:6)', [rr,1]), zeros(rr,12)];
%             [pr, pc] = size(temppath);
%             path = temppath + [zeros(pr,2), x_a(1:3)' + x_a(4:6)'.*temppath(:,1), repmat(x_a(4:6)', [pr,1]), zeros(pr,12)];
            if(any(isnan(path)))
                disp('bad path');
            end
        else
            path = cell2mat(phi(i));
        end
%         cost = costFun(path(end,3:5), path(end,6:8), path(end,9:17));
        cost = costFun(path);
        if(closestTraj(2) > cost)
%             global Target
%             fprintf('Found better path at (%f,%f,%f) cost (%f) than (%f,%f,%f) cost (%f)\n', path(end,3)-Target(1),path(end,4)-Target(2),path(end,5)-Target(3),cost, closestPathVals(1)-Target(1),closestPathVals(2)-Target(2),closestPathVals(3)-Target(3), closestTraj(2))
%             closestPathVals = path(end,3:5);
            closestTraj = [i,cost];
        end
    end
%     out = cell2mat(phi(closestTraj(1)));
    
    if(useMotionPrim)
%         out = cell2mat(r(closestTraj(1))) + [0; x_a(1:3) + executeTime*x_a(4:6); x_a(4:6); zeros(24,1)]';
        tempr = cell2mat(r(closestTraj(1)));
        [~,~,yawAng] = rot2eul(x_a(7:15));
        yawRot = [cos(yawAng) -sin(yawAng) 0; sin(yawAng) cos(yawAng) 0; 0 0 1];
        [rr, ~] = size(tempr);
        spunr = [tempr(:,1), tempr(:,2:4)*yawRot', tempr(:,5:7)*yawRot', tempr(:,8:10)*yawRot', tempr(:,11:13)*yawRot', tempr(:,14:16)*yawRot', zeros(rr,9), tempr(:,26:31)];
        for i = 1:rr
            spunr(i,17:25) = reshape(yawRot*reshape(tempr(i,17:25),[3,3]),[1,9]);
        end
        out = spunr + [zeros(rr,1), x_a(1:3)' + x_a(4:6)'.*tempr(:,1), repmat(x_a(4:6)', [rr,1]), zeros(rr,24)];
    else
        out = cell2mat(r(closestTraj(1)));
    end
    global plot88
    if(plot88)
        figure(88);
        plot3(out(:,2),out(:,3),out(:,4));
        xlabel('x');
        ylabel('y');
        zlabel('z');
        hold on;
        numPts = size(out);
        for i = 1:numPts(1)
            pose = reshape(out(i,17:25),[3 3])*[0; 0; 0.1];
            plot3([out(i,2),pose(1)+out(i,2)], [out(i,3),pose(2)+out(i,3)], [out(i,4),pose(3)+out(i,4)], 'r')
            pose = reshape(out(i,17:25),[3 3])*[0.1; 0; 0];
            plot3([out(i,2),pose(1)+out(i,2)], [out(i,3),pose(2)+out(i,3)], [out(i,4),pose(3)+out(i,4)], 'g')
            pose = reshape(out(i,17:25),[3 3])*[0; 0.1; 0];
            plot3([out(i,2),pose(1)+out(i,2)], [out(i,3),pose(2)+out(i,3)], [out(i,4),pose(3)+out(i,4)], 'b')
            scatter3(out(1,2),out(1,3),out(1,4),'k');
    %         plot3([out(i,2),pose(1)], [out(i,3),pose(2)], [out(i,4),pose(3)], 'k')
        end
        title("Selected Trajectory");
        daspect([1 1 1]);
    end
%     pause
end

function [thetaX, thetaY, thetaZ] = rot2eul(R)
    if(R(7)<1)
        if(R(7)>-1)
            thetaY = asin(-R(7));
            thetaZ = atan2(R(4), R(1));
            thetaX = atan2(R(8), R(9));
        else
            thetaY = pi/2;
            thetaZ =-atan2(-R(6) , R(5));
            thetaX = 0;
        end
    else
        thetaY = -pi /2;
        thetaZ = atan2(-R(6) , R(5));
        thetaX = 0;
    end
    thetaX = rad2deg(thetaX);
    thetaY = rad2deg(thetaY);
    thetaZ = rad2deg(thetaZ);
end