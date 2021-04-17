close all
% clear
clc

global motionPrims plot77
plot77 = 0;
% motionPrims = [];

global plot88
plot88 = 0;

global noSendCF
noSendCF = 1;

runCFavoidance();

function runCFavoidance()
    global HEQOptions;
    HEQOptions = odeset('AbsTol', 1e-1,'RelTol',1e-1);%, 'InitialStep',1e-3);%,'MaxStep',.1);

    global gamma lambda sigma_max planTime planStep executeTime worldTimeStep obstacleSize FirstRunEject ConstThrust objColStep prevRef ObjPlot55
    gamma = 9.8;
    lambda = .6;
    sigma_max = 0.2;
    planTime = 0.5;
    planStep = 0.1;
    executeTime = 0.3;
    worldTimeStep = 0.1;
    obstacleSize = 0.05;
    FirstRunEject = 0;
    ConstThrust = 1;
    objColStep = 200;
    prevRef = [];
    ObjPlot55 = 0;

    global Target TargetRad
    Target = [0.7,-0.1,0.76];
    TargetRad = 0.0;

    global mobilityRadius usingPointMassVehicle maxAcc Mass InertialTensor MaxMotorThrust d_xy PlanningThrustMult useMotionPrim
    usingPointMassVehicle = 0;
    mobilityRadius = planTime;
    maxAcc = 23;
    Mass = 0.028 + 0.013; %kg
    InertialTensor = [16.571710, 0.830806, 0.718277; 0.830806, 16.655602, 1.800197; 0.718277, 1.800197, 29.261652] * 10^-6; % 10^-6kg meters^2
    MaxMotorThrust = 0.1597;%newtons % 2.130295*10^-11 *65535^2 + 1.032633*10^-6 *65535 + 5.484560*10^-4
    d_xy = (( 2^0.5)/4)*0.092; %meters
    PlanningThrustMult = 0.9;
    useMotionPrim = 1;

    %Hybrid controller constants
    global Qdelta alpha K k_V_0 k_z k_p k_v k k_omega k_b1 k_b2 beta k_q
    Qdelta = 0.75; % Qdelta \in (0,1)
    alpha = 0.75; % \in (0,1)
    K = 1;
    k_V_0 = 0.01;
    k_z = 0.3;
    k_p = 3;
    k_v = 6;
    k = 3;
    k_omega = 40;
    k_b1 = 1;
    k_b2 = 1;
    beta = k_v/4; % beta \in (0,k_v)
    k_q = 1;
    
    global spherePts spx spy spz
    spherePts = 3;
    [spx, spy, spz] = sphere(spherePts);
    spx = obstacleSize*reshape(spx(:,1:spherePts),[],1);
    spy = obstacleSize*reshape(spy(:,1:spherePts),[],1);
    spz = obstacleSize*reshape(spz(:,1:spherePts),[],1);

    % p_x_a; p_y_a; p_z_a; v_x_a; v_y_a; v_z_a; R_11-R_31; R_12-R_32; R_13-R_33; omega; z; h; qhat
    startq = [0; 1; 0; 0];
    startR = doubleCover([0 1 0 0]/norm([0 1 0 0]));
    x_a = [zeros(6,1); reshape(startR,[9,1]); zeros(3,1); zeros(3,1); 1; startq/norm(startq)];
    % p_x_o; v_x_o; p_y_o; v_y_o; p_z_o; v_z_o; sigmaMin; sigmaMax
    % xi_o_0 = [-.03, 0, 0, 1, 4, 0, -sigma_max, sigma_max];
    % xi_o_1 = [.03, 0, 1, -0.5, 2, 0, -sigma_max, sigma_max];
    % x_o = [mat2cell([0,0,xi_o_0], 1, 10);mat2cell([0,0,xi_o_1], 1, 10)];
    xi_p = 0;

    t_a = 0;
    j_a = 0;
    r_log = zeros(1,31);

    %Set up optitrak
    dllPath = fullfile('c:','Users','adrames','Desktop','Aerial Vehicle','CrazyFlieAvoidanceExperiment','NatNetML.dll');
    assemblyInfo = NET.addAssembly(dllPath);
    Client = NatNetML.NatNetClientML(0);
    HostIP = char('128.114.56.19');
    Client.Initialize(HostIP, HostIP);
    
    %Connect to cfclient
    addpath(genpath('C:\Users\adrames\Desktop\Aerial Vehicle\MatlabZMQTest\matlab-zmq\lib'))
    addpath('C:\Users\adrames\Desktop\Aerial Vehicle\MatlabZMQTest\libzmq')
    context = zmq.core.ctx_new();
    socket  = zmq.core.socket(context, 'ZMQ_PUSH');
    address = 'tcp://127.0.0.1:1212';
    zmq.core.connect(socket, address);
    
    cleanupObj = onCleanup(@()cleanupFunc(Client, socket));

    %Load motion primatives
%     if(~exist('motionPrims', 'var'))
        global motionPrims
%     end
    if(isempty(motionPrims))
        disp('Loading motionPrims from file')
    %     motionPrims = load('D:\adrames\motionData_omegaMax5_numStep11_numVals7\motionData.mat');
%         motionPrims = load('D:\adrames\motionData_omegaMax0_numStep9_numVals7_AngLim\motionData_v2.mat');
        motionPrims = load('D:\adrames\motionData1.mat');
        disp('Done loading motionPrims from file')
    end

    %Run planner and update world
    global xi_a 
    xi_a = [];
    T = 2*MaxMotorThrust;
    M = [0;0;0];
    r = [0,0,0,0,zeros(1,12),reshape(eye(3),[1,9]),zeros(1,3); 1000000,0, 0, 0,zeros(1,12),reshape([-1 0 0; 0 0 1; 0 1 0],[1,9]),zeros(1,3)];
    lastT = 0;
    startT = 0;
    numPlans = 1;
    numIt = 100;
    Tlog = zeros(numIt,1);
%     yprLog = zeros(numIt,3);
    x_aLog = zeros(numIt, 18);
%     Rmod = eye(3);
    Rmod = [1 0 0; 0 -1 0; 0 0 -1];
    TM_log = zeros(numIt, 4);
    cmdlog = zeros(numIt, 4);
    rawLog = zeros(numIt, 4);
    % 30deg about x (pos pitch)
%     Rmod = [  1.0000000,  0.0000000,  0.0000000; 0.0000000,  0.9659258, -0.2588190; 0.0000000,  0.2588190,  0.9659258 ];
    % 30deg about y (pos roll)
%     Rmod = [  0.8660254,  0.0000000,  0.5000000;   0.0000000,  1.0000000,  0.0000000;  -0.5000000,  0.0000000,  0.8660254 ];
    
    % -30deg about y (neg roll)
%     Rmod = [  0.8660254,  0.0000000, -0.5000000; 0.0000000,  1.0000000,  0.0000000; 0.5000000,  0.0000000,  0.8660254 ]

    fullTime = tic;
    lptime = 0;
    for i=1:numIt
        loopTime = tic;
        [~, p_a,v_a,R_a,omega_a, x_o] = getWorldState(Client, 1);
        x_a = [p_a;v_a;reshape(R_a, [9,1]); omega_a; x_a(19:end)];
        x_aLog(i,:) = x_a(1:18);
        
        %Run planner
        if(isempty(xi_a))
            xi_a = x_a;
    %         r = [t,p_a',zeros(1,9),reshape(eye(3),[1,9]),zeros(1,3);t+2,p_a'+[.0, .0, .3],zeros(1,9),reshape(eye(3),[1,9]),zeros(1,3)];
        end
        
        if(i == 1)
            Target = p_a'
%             f = parfeval(@setBasedPlanner,1,x_a, x_o, @cost);
%             tarState = [p_a' + [0 0 0], zeros(1,12), reshape([1 0 0; 0 -1 0; 0 0 -1]*Rmod, [1,9]), zeros(1,3)];
%             tarState = [p_a' + [0 0 0.1], zeros(1,12), reshape(Rmod*[1 0 0; 0 1 0; 0 0 1], [1,9]), zeros(1,3)];
%             r = [0,tarState; 100, tarState];
            r = setBasedPlanner(x_a, x_o, @cost);
            lpTime = tic;
            r_log = [r_log(end,1)+r(:,1), r(:,2:end)];
%             startT = t;
%             lastT = t;
        else
            [rr, ~] = size(r);
            if(r(floor(rr*(executeTime/planTime)),1) < toc(fullTime))
%             if(t > lastT + executeTime)
%             if(toc(lpTime) >= executeTime)
                curTime = toc(fullTime);
                rAsStateTemp = interp1(r(:,1),r(:,2:end), curTime);
                rAsState = [rAsStateTemp(1:6), rAsStateTemp(16:27)]';
                if(any(isnan(rAsState)))
                    disp('bad r state');
                    return
                end
                lpTime = tic;
                r = setBasedPlanner(rAsState, x_o, @cost);
%                 toc(lpTime)
                timeOffset = toc(fullTime);
                r(:,1) = r(:,1) + timeOffset;
%                 scatter3(r(1,2), r(1,3), r(1,4), 'g');
                r_log = [r_log;r(:,1), r(:,2:end)];
                numPlans = numPlans +1
%                 lastT = t;
            end
        end

        [Tnew, Mnew] = hybridVehicleController(r,x_a,toc(fullTime));
        if(Tnew >= 0)
            T = Tnew;
            M = Mnew;
        end
        
        TM_log(i,:) = [T,M'];
        Tlog(i) = 100*T/(MaxMotorThrust*4);
%         yprLog(i) = 
        [cmdlog(i,:), rawLog(i,:)] = sendCFdata(socket,T,M,x_a(7:15),x_a(16:18),x_a);
        
        loopDt = toc(loopTime);
        if(loopDt < 0.020)
            java.lang.Thread.sleep((0.020 - loopDt)*1000)
        end
    end
    toc(fullTime)
%     str = ['{"version":1, "client_name": "N/A", "ctrl": {"thrust": ',num2str(0.0),', "yaw": ',num2str(0.0),', "roll": ',num2str(0.0),', "pitch": ',num2str(0.0),'}}'];
    str = ['{"m1": ',num2str(0),', "m2": ',num2str(0),', "m3": ',num2str(0),', "m4": ',num2str(0),'}'];
    zmq.core.send(socket, uint8(str));
%     x_a
%     r(:, 2:end)
%     pause
%     x_aLog
    logSize = size(x_aLog)
    c = autumn(logSize(1));
    figure
    hold on;
    up = reshape(r(1,17:25), [3,3])*[0;0;.2];
    xaxis = reshape(r(1,17:25), [3,3])*[.2;0;0];
    yaxis = reshape(r(1,17:25), [3,3])*[0;.2;0];
    plot3([r(1,2), r(1,2)+up(1)], [r(1,3), r(1,3)+up(2)], [r(1,4), r(1,4)+up(3)],'r');
    plot3([r(1,2), r(1,2)+xaxis(1)], [r(1,3), r(1,3)+xaxis(2)], [r(1,4), r(1,4)+xaxis(3)],'g');
    plot3([r(1,2), r(1,2)+yaxis(1)], [r(1,3), r(1,3)+yaxis(2)], [r(1,4), r(1,4)+yaxis(3)],'b');
    scatter3(r(1,2), r(1,3), r(1,4), 'k');
    xlabel('x'); ylabel('y'); zlabel('z');
    for i = 1:logSize(1)
        up = reshape(x_aLog(i,7:15), [3,3])*[0;0;.1];
        xaxis = reshape(x_aLog(i,7:15), [3,3])*[.1;0;0];
        yaxis = reshape(x_aLog(i,7:15), [3,3])*[0;.1;0];
        plot3([x_aLog(i,1), x_aLog(i,1)+xaxis(1)], [x_aLog(i,2), x_aLog(i,2)+xaxis(2)], [x_aLog(i,3), x_aLog(i,3)+xaxis(3)],'g');
        plot3([x_aLog(i,1), x_aLog(i,1)+yaxis(1)], [x_aLog(i,2), x_aLog(i,2)+yaxis(2)], [x_aLog(i,3), x_aLog(i,3)+yaxis(3)],'b');
        plot3([x_aLog(i,1), x_aLog(i,1)+up(1)], [x_aLog(i,2), x_aLog(i,2)+up(2)], [x_aLog(i,3), x_aLog(i,3)+up(3)], 'color', c(i,:));
        scatter3(x_aLog(i,1), x_aLog(i,2), x_aLog(i,3), 'k');
    end
    scatter3(r_log(:,2), r_log(:,3), r_log(:,4), 'r');
    r(end,1)
    
    figure
    subplot(4,1,1);plot(1:numIt, cmdlog(:,1)/655.35);title("CMDs");ylabel('Percent');
    subplot(4,1,2);plot(1:numIt, cmdlog(:,2)/655.35);ylabel('Percent');
    subplot(4,1,3);plot(1:numIt, cmdlog(:,3)/655.35);ylabel('Percent');
    subplot(4,1,4);plot(1:numIt, cmdlog(:,4)/655.35);ylabel('Percent');
    
    figure
    subplot(4,1,1);plot(1:numIt, TM_log(:,1));title("TMs");
    subplot(4,1,2);plot(1:numIt, TM_log(:,2))
    subplot(4,1,3);plot(1:numIt, TM_log(:,3))
    subplot(4,1,4);plot(1:numIt, TM_log(:,4))
    
    figure
    subplot(4,1,1);plot(1:numIt, rawLog(:,1));title("Raw Motor Vals"); ylabel('Newtons');
    yline(MaxMotorThrust);
    subplot(4,1,2);plot(1:numIt, rawLog(:,2)); ylabel('Newtons');
    yline(MaxMotorThrust);
    subplot(4,1,3);plot(1:numIt, rawLog(:,3)); ylabel('Newtons');
    yline(MaxMotorThrust);
    subplot(4,1,4);plot(1:numIt, rawLog(:,4)); ylabel('Newtons');
    yline(MaxMotorThrust);
    
%     simState = x_aLog(1,1:18)
%     for i = floor(1:numIt/20:numIt)
%         simState = x_aLog(i,1:18)
%         [~,~,simOut] = simHybridVehicleOnly(simState', TM_log(i,1), TM_log(i,2:4)', [0, 0.01]);
%         simState = simOut(end,1:18);
%         scatter3(simOut(:,1),simOut(:,2),simOut(:,3),'rx');
%         up = reshape(simOut(end,7:15), [3,3])*[0;0;.01];
%         xaxis = reshape(simOut(end,7:15), [3,3])*[.01;0;0];
%         yaxis = reshape(simOut(end,7:15), [3,3])*[0;.01;0];
%         plot3([simOut(end,1), simOut(end,1)+xaxis(1)], [simOut(end,2), simOut(end,2)+xaxis(2)], [simOut(end,3), simOut(end,3)+xaxis(3)],'g');
%         plot3([simOut(end,1), simOut(end,1)+yaxis(1)], [simOut(end,2), simOut(end,2)+yaxis(2)], [simOut(end,3), simOut(end,3)+yaxis(3)],'b');
%         plot3([simOut(end,1), simOut(end,1)+up(1)], [simOut(end,2), simOut(end,2)+up(2)], [simOut(end,3), simOut(end,3)+up(3)], 'color', c(i,:));
%     end
%     TM_log
%     pause
%     cmdLog
%     pause
%     numPlans
%     Tlog
%     mean(Tlog)
%     pause
%     yprLog
%     mean(yprLog)
end

function rplot(r)
    figure(27);
    hold on
    plot3(r(:,2), r(:,3), r(:,4), 'r');
end

function cleanupFunc(Client, socket)
    Client.Uninitialize();
%     str = ['{"version":1, "client_name": "N/A", "ctrl": {"thrust": ',num2str(0.0),', "yaw": ',num2str(0.0),', "roll": ',num2str(0.0),', "pitch": ',num2str(0.0),'}}'];
    str = ['{"m1": ',num2str(0),', "m2": ',num2str(0),', "m3": ',num2str(0),', "m4": ',num2str(0),'}'];
    zmq.core.send(socket, uint8(str));
end

function [out, decelTerm, AngTerm] = cost(traj,plotCost)
    global Target TargetRad executeTime planTime
%     [Tr Tc] = size(Target);
%     if(Tr > Tc)
%         Target = Target';
%         disp('Target must be a row vector');
%     end
%     dist = max(norm(traj(end,3:5)-Target) - TargetRad, 0);
%     if(~exist('plotCost','var'))
%         plotCost = 0;
%     end
%     if plotCost == 1
%         exState = traj(:,1:end);
%     else
%         exState = interp1(traj(:,1),traj(:,3:end),traj(end,1) - planTime + executeTime)
        exState = traj(end,3:end);
%     end
%     if(any(isnan(exState)))
%         traj(end,1) - planTime + executeTime
%         traj
%         AngTerm = inf;
%         decelTerm = inf;
%         out = inf;
%         pause
%         return
%     end
    dist = max(norm(exState(end,1:3)-Target) - TargetRad, 0);
%     if(isempty(prevRef))
%         hysteresisCost = 0;
%     else
%         lastEndT = prevRef(end,1) - (planTime - executeTime)
%         trajAtEndT = interp1(traj(:,1),traj(:,3:5),lastEndT);
%         hysteresisCost = norm(trajAtEndT-prevRef(end,2:4))
%         if(hysteresisCost<1)
%         pause
%     end
    hysteresisCost = 0;
    
%     R = reshape(traj(end,9:17),[3,3]);
%     phi = acos(dot(R*[0;0;1], [0;0;1]));
%     phi = min(abs(phi), abs(phi-pi));
%     poseTerm = 1;%2-cos(phi)^4;
%     if(phi>=pi/4)
%         poseTerm = 10000;
%     end
    
%     R = reshape(exState(end,7:15),[3,3]);
%     phi = acos(dot(R*[0;0;1], [0;0;1]));
%     phi = min(abs(phi), abs(phi-pi));
%     poseTerm = 1;%2-cos(phi)^4;
%     if(phi>=pi/3)
%         poseTerm = 10000;
%         if(phi>=pi/2)
%             poseTerm = Inf;
%         end
%     end

%     stopingPoint = (exState(4:6).*abs(exState(4:6)))/(maxAcc*2) + exState(1:3);
%     dist = max(norm(stopingPoint-Target) - TargetRad, 0);
    
%     AB = (stopingPoint-exState(1:3));
%     AT = (Target-exState(1:3));
%     proj = (dot(AB,AT)/(norm(AT)*norm(AT)))*AT;
%     rej = AB-proj;
    
%     stopingPoint = (vel.*abs(vel))/(maxAcc*2) + point;
%     AB = (stopingPoint-point);
%     AT = (Target-point);
%     proj = (dot(AB,AT)/(norm(AT)*norm(AT)))*AT;
%     rej = AB-proj;
    
    decelTerm = 0;%norm(exState(4:6))/10;
    AngTerm = 0;%phi;
    
%     out = (max(norm(AT-proj)-TargetRad, 0) + 4*norm(rej))*poseTerm;
    out = dist;%*poseTerm + decelTerm;
%     out = dist*poseTerm + norm(exState(6:8))/5 + hysteresisCost/10;
end

function R = doubleCover(q)
    R = eye(3) + 2*q(1)*S(q(2:4)) + 2*S(q(2:4))^2;
end

function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end

%[pitch, roll, yaw]
function  [cmds,rawMs] = sendCFdata(socket,T,M,Rin,omega,x)
    global d_xy MaxMotorThrust
    kappa = 0.005964552;%-10.311 * 10^-7;

%     R = reshape(Rin,[3,3]);
%     global InertialTensor MaxMotorThrust
%     omegadot = -InertialTensor\S(omega)*InertialTensor*omega + InertialTensor\M;
%     omegaAvg = omega + (omegadot*t)/2;
% %     R
%     Rdot = R*S(omegaAvg);
%     targetR = (R + Rdot*t)*[1 0 0; 0 -1 0; 0 0 -1]';
%     [pitch,roll,yaw] = rot2eul(targetR);
% %     pry = [pitch,roll,yaw]
%     thrust = 100*T/(MaxMotorThrust*4);

%     simHybridVehicleOnly(x(1:18)', 
    
%     roll = min(30, max(-30, roll));
%     pitch = min(30, max(-30, pitch));
%     yawRate = min(10, max(-10, yaw/t));
    
%     yaw = 0;
    
%     [M;T]

    m1 = [ -1/(4*d_xy), -1/(4*d_xy),  1/(4*kappa), 1/4] * [M;T];
    m2 = [  1/(4*d_xy), -1/(4*d_xy), -1/(4*kappa), 1/4] * [M;T];
    m3 = [  1/(4*d_xy),  1/(4*d_xy),  1/(4*kappa), 1/4] * [M;T];
    m4 = [ -1/(4*d_xy),  1/(4*d_xy), -1/(4*kappa), 1/4] * [M;T];
    
    rawMs = [m1, m2, m3, m4];
%     persistent bad_ct
%     if(isempty(bad_ct))
%         bad_ct = 0
%     end
    if(any(abs(rawMs) > MaxMotorThrust) & T < MaxMotorThrust*0.95)
%         disp('Bad Ms')
%         bad_ct = bad_ct + 1;
        cmds = [0 0 0 0];
        return
    end
    
    m1 = max(0,m1);
    m2 = max(0,m2);
    m3 = max(0,m3);
    m4 = max(0,m4);
    
    cmd1 = (154742504910672534362390528*((6592943689973623*m1)/77371252455336267181195264 + 727608011901638924708917863587871/713623846352979940529142984724747568191373312)^(1/2))/6592943689973623 - 159792217073422499840/6592943689973623;
    cmd2 = (154742504910672534362390528*((6592943689973623*m2)/77371252455336267181195264 + 727608011901638924708917863587871/713623846352979940529142984724747568191373312)^(1/2))/6592943689973623 - 159792217073422499840/6592943689973623;
    cmd3 = (154742504910672534362390528*((6592943689973623*m3)/77371252455336267181195264 + 727608011901638924708917863587871/713623846352979940529142984724747568191373312)^(1/2))/6592943689973623 - 159792217073422499840/6592943689973623;
    cmd4 = (154742504910672534362390528*((6592943689973623*m4)/77371252455336267181195264 + 727608011901638924708917863587871/713623846352979940529142984724747568191373312)^(1/2))/6592943689973623 - 159792217073422499840/6592943689973623;

    cmds = min(65535, max(0,floor([cmd1, cmd2, cmd3, cmd4])));
    
%     str = ['{"version":1, "client_name": "N/A", "ctrl": {"thrust": ',num2str(thrust),', "yaw": ',num2str(yawRate),', "roll": ',num2str(roll),', "pitch": ',num2str(pitch),'}}'];
    str = ['{"m1": ',num2str(cmds(1)),', "m2": ',num2str(cmds(2)),', "m3": ',num2str(cmds(3)),', "m4": ',num2str(cmds(4)),'}'];
%     global noSendCF
%     if(noSendCF ~= 1)
%         disp('Sending')
        if(zmq.core.send(socket, uint8(str)) == -1)
    %         disp("Bad send")
            error("Bad send")
        end
%     end
end

%https://www.geometrictools.com/Documentation/EulerAngles.pdf
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