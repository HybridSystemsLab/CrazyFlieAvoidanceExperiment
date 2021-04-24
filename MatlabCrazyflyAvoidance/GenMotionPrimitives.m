close all
clear
clc
global Mass InertialTensor MaxMotorThrust d_xy PlanningThrustMult ConstThrust planTime makePlot
planTime = .3;
Mass = 0.028; %kg
InertialTensor = [16.571710, 0.830806, 0.718277; 0.830806, 16.655602, 1.800197; 0.718277, 1.800197, 29.261652] * 10^-6; % 10^-6kg meters^2
MaxMotorThrust = 0.1597;%newtons % 2.130295*10^-11 *65535^2 + 1.032633*10^-6 *65535 + 5.484560*10^-4
d_xy = (( 2^0.5)/4)*0.092; %meters
PlanningThrustMult = 0.8;
ConstThrust = 0;
makePlot = 0;

global numStep vMax omegaMax motionPoints motionPrimatives startRot
motionPrimatives = {};
motionPoints = [];
numStep = 5; % + 1
vMax = 1;
omegaMax = 0;
startRot = [1 0 0; 0 -1 0; 0 0 -1];

omegaLoop()
% hybridModelMobility([zeros(6,1); reshape(startRot, [9,1]); zeros(3,1)])


function vLoop(R,omega)
    global motionPoints motionPrimatives
%     global numStep vMax
%     for vx = -vMax:2*vMax/numStep:vMax
%         for vy = -vMax:2*vMax/numStep:vMax
%             vz = 0;
%             for vz = -vMax:2*vMax/numStep:vMax
    vx = 0;
    vy = 0;
    vz = 0;
                [trajs, rs] = hybridModelMobility([0;0;0;vx;vy;vz;R;omega]);
                motionPoints(:,end+1) = [R;omega];
                [row,col] = size([trajs, rs]);
                motionPrimatives(end+1) = mat2cell([trajs, rs], row,col);
%             end
%         end
%     end
end

function RLoop(omega)
    global numStep startRot
    persistent allR
    if(isempty(allR))
        tempQ = zeros(4,(numStep+1)^4);
        for qx = -1:1/numStep:1
            for qy = -1:1/numStep:1
                qz = 0;
%                 for qz = -1:1/numStep:1
                    for qw = -1:1/numStep:1
                        rawquat = [qw;qx;qy;qz];
                        if(norm(rawquat) == 0)
                            continue
                        else
                            tempQ(:,end+1) = rawquat/norm(rawquat);
                        end
                    end
%                 end
            end
        end
        allQ = unique(tempQ', 'row')';
        [~, sz] = size(allQ(1,:));
        tempR = [];
        for i = 1:sz
            dc = doubleCover(allQ(:,i));
            newz = dc*[0;0;1];
            if( acos(dot(newz,[0;0;1])) > pi/3)
                continue
            end
            [Rx,Ry,Rz] = rot2eul(dc);
            if(abs(Rx) > 30 || abs(Ry) > 30)
                continue
            end
            tempR(:,end+1) = reshape(dc*startRot, [9,1]);
        end
        allR = unique(tempR', 'row')';
    end
    [~, numR] = size(allR(1,:));
    for i = 1:numR
        vLoop(allR(:,i),omega)
    end
end

function omegaLoop()
    global numStep omegaMax motionPoints motionPrimatives
    i = 1;
%     for omegax = -omegaMax:2*omegaMax/numStep:omegaMax
    omegax = 0;
    omegay = 0;
%         for omegay = -omegaMax:2*omegaMax/numStep:omegaMax
            %Skip omegaz since assume no yaw
%             for omegaz = -omegaMax:2*omegaMax/numStep:omegaMax
                RLoop([omegax;omegay;0])
%             end
%         end
        fileName = ['D:/adrames/motionData', num2str(i),'.mat']
        i = i+1;
        save(fileName, 'motionPoints', 'motionPrimatives', '-v7.3')
        motionPrimatives = {};
        motionPoints = [];
%     end
end

function R = doubleCover(q)
    R = eye(3) + 2*q(1)*S(q(2:4)) + 2*S(q(2:4))^2;
end
function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end

function [trajs, rs] = hybridModelMobility(x_a)
    global d_xy MaxMotorThrust planTime FirstRunEject ConstThrust PlanningThrustMult
    numMVals = 4; % Use even number
    
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
    elseif(isempty(TMs)||FirstRunEject == 2)
        disp("genning TM ball")
        FirstRunEject = 0;
        TMs = zeros(4, 2+(numMVals-1)*(numMVals+1)^2);
        i = 2;
        TMs(:,1) = [0; 0; 0; 0];
%         for trst=0.3:0.7/(numMVals+2):1
        for tempTrst=1/(numMVals+2):1/(numMVals+2):(1-1/(numMVals+2))
            trst = 1 - tempTrst*tempTrst;
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
    disp("finished ref gen");
    numTraj = 0;
    trajs = cell(i,1);
    rs = cell(i,1);
    global makePlot
    if makePlot
        figure(76)
        hold on;
    end
    global j28
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
            [t,simJump,r] = simHybridVehicleOnly(x_a(1:18), TMs(1,j), TMs(2:4,j), planTime);
            if(any(isnan(r)))
                error('Nan in r');
            end
            if(t(end) < planTime)
                error('Jump Out')
            end
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
        
        %Remove jumps since time does not advance
%         rawtSize = size(t);
        [t, ut_id] = unique(t);
        r = r(ut_id,:);
%         if(size(t) ~= rawtSize | jump(end) > 0)
%             disp('nonunique t')
%         end

        dt = gradient(t);
        if makePlot 
            plot3(r(:,1),r(:,2),r(:,3));
%             pose = reshape(r(end,7:15),[3 3])*[0; 0; 0.01];
%             plot3([r(end,1),pose(1)+r(end,1)], [r(end,2),pose(2)+r(end,2)], [r(end,3),pose(3)+r(end,3)], 'r')
            xlabel('x');
            ylabel('y');
            zlabel('z');
            [numPts,~] = size(r);
            for i = 1:10:numPts(1)
                pose = reshape(r(i,7:15),[3 3])*[0.01; 0; 0];
                plot3([r(i,1),pose(1)+r(i,1)], [r(i,2),pose(2)+r(i,2)], [r(i,3),pose(3)+r(i,3)], 'g')
                pose = reshape(r(i,7:15),[3 3])*[0; 0.01; 0];
                plot3([r(i,1),pose(1)+r(i,1)], [r(i,2),pose(2)+r(i,2)], [r(i,3),pose(3)+r(i,3)], 'b')
                pose = reshape(r(i,7:15),[3 3])*[0; 0; 0.01];
                plot3([r(i,1),pose(1)+r(i,1)], [r(i,2),pose(2)+r(i,2)], [r(i,3),pose(3)+r(i,3)], 'r')
                scatter3(r(1,2),r(1,3),r(1,4),'k');
        %         plot3([out(i,2),pose(1)], [out(i,3),pose(2)], [out(i,4),pose(3)], 'k')
            end
        end
        [~,dv] = gradient(r(:,4:6));
        vdot = dv./dt;
        numrpts = size(t);
        adot = zeros(numrpts(1),3);
        jdot = zeros(numrpts(1),3);
        [~,dw] = gradient(r(:,16:18));
        wdot = dw./dt;
        r = [t, r(:,1:6), vdot, adot, jdot, r(:,7:end), wdot];
        if(any(isnan(r)))
            error('Nan in r');
        end
        %Run reference trajectories through controller
%         [time,jump,phi] = hybridVehicle(r, x_a, planTime,1);
        %Check if jumped out
%         if(jump(end) == 50)
%             continue
%         end
        %Save reference and resulting trajectories
        [row,col] = size([time,jump,phi]);
        numTraj = numTraj+1;
        if(any(isnan([time,jump,phi])))
            error("Found Nan in trajs")
        end
%         if(numTraj == 330)
%             disp('330');
%         end
        trajs(numTraj) = mat2cell([time,jump,phi],row,col);
        [row,col] = size(r);
        if(any(isnan(r)))
            error("Found Nan in rs")
        end
        rs(numTraj) = mat2cell(r,row,col);
    end
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