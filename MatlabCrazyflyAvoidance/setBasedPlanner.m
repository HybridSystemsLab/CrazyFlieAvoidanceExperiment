function out = setBasedPlanner(x_a, x_o, costFun)
    global planTime sigma_max
    %Generate unsafe sets for obstacles
    xSpin = [sigma_max, sigma_max, -sigma_max, -sigma_max];
    ySpin = [sigma_max, -sigma_max, sigma_max, -sigma_max];
    numObj = size(x_o)
    U = cell(numObj(1),numObj(2)*4)
    for i=1:numObj(1)
        for j = 1:4
            x_o_mat = cell2mat(x_o(i));
            x_o_mat_size = size(x_o_mat);
            x_o_mat(:,9:10) = repmat([xSpin(j), ySpin(j)], [x_o_mat_size(1), 1]);
            [bb_t, bb_j, bb_x] = bouncingBallModel(x_o_mat(end,end-7:end), planTime);
            bballOut = [bb_t, bb_j, bb_x];
            [row, col] = size(bballOut);
            U(i,j) = mat2cell(bballOut,row,col);
        end
    end
    
    %Generate safe reachable map
    genTime = tic;
    [phi,r] = genSafeTraj(x_a,U);
    toc(genTime)
    
    global plot77
    if(plot77)
        f77 = figure(77);
        clf(f77)
        numRef = size(r);
        hold on;
        for i = 1:numRef(2)
            ref = cell2mat(r(i));
            plot3(ref(:,2),ref(:,3),ref(:,4));
        end
        xlabel('x');
        ylabel('y');
        zlabel('z');
        daspect([1 1 1])
    end
    
%     disp("Gened Trajs")
    
    %Find "optimal" trajectory
    [~,numTraj] = size(phi);
    if(numTraj == 0)
        error("No safe trajectories, phi:",phi)
    end
    path = cell2mat(phi(1));
%     closestTraj = [1, costFun(path(end,3:5), path(end,6:8), path(end,9:17))];
    closestTraj = [1, costFun(path)];
    for i=2:numTraj
        path = cell2mat(phi(i));
%         cost = costFun(path(end,3:5), path(end,6:8), path(end,9:17));
        cost = costFun(path);
        if(closestTraj(2) > cost)
            closestTraj = [i,cost];
        end
    end
%     out = cell2mat(phi(closestTraj(1)));
    
    out = cell2mat(r(closestTraj(1)));
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
    %         plot3([out(i,2),pose(1)], [out(i,3),pose(2)], [out(i,4),pose(3)], 'k')
        end
        title("Selected Trajectory");
        daspect([1 1 1]);
    end
%     pause
end