function plotResults(t_a,x_a, x_o,r)
    disp("Plotting Results")
    global Target TargetRad
    figure(1);
    plot3(x_a(:,1), x_a(:,2), x_a(:,3), 'b');
    hold on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    for i = 1:size(x_o)
        x_o_1 = cell2mat(x_o(i));
        plot3(x_o_1(:,3), x_o_1(:,5), x_o_1(:,7), 'r');
    end
    
    [SX,SY,SZ] = sphere();
    SXT = SX*TargetRad + Target(1);
    SYT = SY*TargetRad + Target(2);
    SZT = SZ*TargetRad + Target(3);
    tSurface = surface(SXT, SYT, SZT);
    set(tSurface,'FaceColor',[0 1 0], 'FaceAlpha',0.5,'FaceLighting','gouraud');
    hold off;
    
    figure(2)
    subplot(3,1,1); plot(t_a, x_a(:,1)); xlabel('t'); ylabel('x'); title('Quad pos')
    subplot(3,1,2); plot(t_a, x_a(:,2)); xlabel('t'); ylabel('y');
    subplot(3,1,3); plot(t_a, x_a(:,3)); xlabel('t'); ylabel('z');
    
    figure(3)
    subplot(3,1,1); plot(t_a, x_a(:,4)); xlabel('t'); ylabel('x');title('Quad vel')
    subplot(3,1,2); plot(t_a, x_a(:,5)); xlabel('t'); ylabel('y');
    subplot(3,1,3); plot(t_a, x_a(:,6)); xlabel('t'); ylabel('z');
    
    figure(4)
    cost_t = zeros(size(t_a));
    dist_t = zeros(size(t_a));
    vel_t = zeros(size(t_a));
    for i=1:size(t_a)
        [cost_t(i), dist_t(i), vel_t(i)] = cost(x_a(i,:),1);
    end
    subplot(3,1,1);plot(t_a, cost_t);
    title('Cost over time');
    subplot(3,1,2);plot(t_a, dist_t);
    subplot(3,1,3);plot(t_a, vel_t);
    
    figure(5)
    subplot(3,1,1); plot(r(:,1), r(:,2))
    title('Reference Trajectory, includes discarded section')
    subplot(3,1,2); plot(r(:,1), r(:,3))
    subplot(3,1,3); plot(r(:,1), r(:,4))
    
    figure(6)
    subplot(3,1,1); plot(x_o_1(:,1), x_o_1(:,3)); xlabel('t'); ylabel('x'); title('obj pos')
    subplot(3,1,2); plot(x_o_1(:,1), x_o_1(:,5)); xlabel('t'); ylabel('y');
    subplot(3,1,3); plot(x_o_1(:,1), x_o_1(:,7)); xlabel('t'); ylabel('z');
    
    figure(7)
    plot3(r(:,2), r(:,3), r(:,4),'r');
    title('Reference and Actual Trajectories');
    xlabel('x'); ylabel('y'); zlabel('z');
    hold on;
    plot3(x_a(:,1), x_a(:,2), x_a(:,3), 'b');
    
    figure(8)
    plot3(x_a(:,1),x_a(:,2),x_a(:,3));
    title('Vehicle Pose')
    xlabel('x'); ylabel('y'); zlabel('z');
    hold on;
    numPts = size(x_a);
    for i = 1:min(floor(numPts(1)/50),1):numPts(1)
        pose = reshape(x_a(i,7:15),[3 3])*[0; 0; 0.1];
        plot3([x_a(i,1),pose(1)+x_a(i,1)], [x_a(i,2),pose(2)+x_a(i,2)], [x_a(i,3),pose(3)+x_a(i,3)], 'r')
        pose = reshape(x_a(i,7:15),[3 3])*[0; 0.1; 0];
        plot3([x_a(i,1),pose(1)+x_a(i,1)], [x_a(i,2),pose(2)+x_a(i,2)], [x_a(i,3),pose(3)+x_a(i,3)], 'b')
        pose = reshape(x_a(i,7:15),[3 3])*[0.1; 0; 0];
        plot3([x_a(i,1),pose(1)+x_a(i,1)], [x_a(i,2),pose(2)+x_a(i,2)], [x_a(i,3),pose(3)+x_a(i,3)], 'g')
%         pose = reshape(out(i,17:25),[3 3])*[0.1; 0; 0];
%         plot3([out(i,2),pose(1)+out(i,2)], [out(i,3),pose(2)+out(i,3)], [out(i,4),pose(3)+out(i,4)], 'g')
%         pose = reshape(out(i,17:25),[3 3])*[0; 0.1; 0];
%         plot3([out(i,2),pose(1)+out(i,2)], [out(i,3),pose(2)+out(i,3)], [out(i,4),pose(3)+out(i,4)], 'b')
%         plot3([out(i,2),pose(1)], [out(i,3),pose(2)], [out(i,4),pose(3)], 'k')
    end
    
%     figure(9)
%     plot3(r(:,2),r(:,3),r(:,4));
%     title('Reference Pose')
%     xlabel('x'); ylabel('y'); zlabel('z');
%     hold on;
%     numPts = size(r);
%     for i = 1:min(floor(numPts(1)/50),1):numPts(1)
%         pose = reshape(r(i,17:25),[3 3])*[0; 0; 0.1];
%         plot3([r(i,2),pose(1)+r(i,2)], [r(i,3),pose(2)+r(i,3)], [r(i,3),pose(4)+r(i,4)], 'r')
%         pose = reshape(r(i,17:25),[3 3])*[0; 0.1; 0];
%         plot3([r(i,2),pose(1)+r(i,2)], [r(i,3),pose(2)+r(i,3)], [r(i,4),pose(3)+r(i,4)], 'b')
%         pose = reshape(r(i,17:25),[3 3])*[0.1; 0; 0];
%         plot3([r(i,2),pose(1)+r(i,2)], [r(i,3),pose(2)+r(i,3)], [r(i,4),pose(3)+r(i,4)], 'g')
%     end
end