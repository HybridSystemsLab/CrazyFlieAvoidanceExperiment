close all
clear
% clear global
% clc

global motionPrims plot77 errLog q_1Log resetPIDs
plot77 = 0;
% motionPrims = [];
errLog = [];
q_1Log = [];
resetPIDs = 1;

global plot88
plot88 = 0;

global noSendCF
noSendCF = 0;

runCFavoidance();

function runCFavoidance()
    global HEQOptions;
    HEQOptions = odeset('AbsTol', 1e-1,'RelTol',1e-1);%, 'InitialStep',1e-3);%,'MaxStep',.1);

    global omegaMapReset gamma lambda sigma_max planTime planStep executeTime worldTimeStep obstacleSize FirstRunEject ConstThrust objColStep prevRef ObjPlot55
    omegaMapReset = 1;
    gamma = 9.8;
    lambda = .6;
    sigma_max = 0.2;
    planTime = 0.3;
%     planStep = 0.5;
    executeTime = 0.28;
    worldTimeStep = 0.1;
    obstacleSize = 0.20;
    FirstRunEject = 0;
    ConstThrust = 1;
    objColStep = 200;
    prevRef = [];
    ObjPlot55 = 0;

    global Target TargetRad
    Target = [0.7,-0.1,0.76];
    TargetRad = 0.1;

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
    K = 0.5;
    k_V_0 = 0.001;
    k_z = 0.03;
    k_p = 1;
    k_v = .3;
    k = 3;
    k_omega = 4;
    k_b1 = 1;
    k_b2 = 1;
    beta = k_v/400; % beta \in (0,k_v)
    k_q = 2;

    global spherePts spx spy spz
    spherePts = 6;
    [spx, spy, spz] = sphere(spherePts);
    spx = obstacleSize*reshape(spx(:,1:spherePts),[],1);
    spy = obstacleSize*reshape(spy(:,1:spherePts),[],1);
    spz = obstacleSize*reshape(spz(:,1:spherePts),[],1);

    % p_x_a; p_y_a; p_z_a; v_x_a; v_y_a; v_z_a; R_11-R_31; R_12-R_32; R_13-R_33; omega; z; h; qhat
    startq = [1; 0; 0; 0];
    startR = doubleCover([0 1 0 0]/norm([0 1 0 0]));
    x_a = [zeros(6,1); reshape(startR,[9,1]); zeros(3,1); zeros(3,1); 1; startq/norm(startq)];
    % p_x_o; v_x_o; p_y_o; v_y_o; p_z_o; v_z_o; sigmaMin; sigmaMax
    xi_o_0 = [-.03, 0, 0, 1, 0, 0, -sigma_max, sigma_max];
    xi_o_1 = [.03, 0, 1, -0.5, 0, 0, -sigma_max, sigma_max];
    x_o = [mat2cell([0,0,xi_o_0], 1, 10);mat2cell([0,0,xi_o_1], 1, 10)];
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
    global motionPrims
    if(isempty(motionPrims))
        disp('Loading motionPrims from file')
%         motionPrims = load('D:\adrames\motionData_numStep7_numVals4_0.2sec\motionData.mat');
%         motionPrims = load('D:\adrames\motionData_omegaMax0_numStep9_numVals7_AngLim\motionData_v2.mat');
        motionPrims = load('D:\adrames\motionData1.mat');
        disp('Done loading motionPrims from file')
        pause
    end

    %Run planner and update world
    global xi_a 
    xi_a = [];
    T = 2*MaxMotorThrust;
    M = [0;0;0];
    r = [0,0,0,0,zeros(1,12),reshape(eye(3),[1,9]),zeros(1,3); 1000000,0, 0, 0,zeros(1,12),reshape([-1 0 0; 0 0 1; 0 1 0],[1,9]),zeros(1,3)];
    numPlans = 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    numIt = 300;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    Tlog = zeros(numIt,1);
    x_aLog = zeros(numIt, 27);
    simLog = cell(numIt,1);
    Rmod = [1 0 0; 0 -1 0; 0 0 -1];
    TM_log = zeros(numIt, 4);
    cmdlog = zeros(numIt, 4);
    rawLog = zeros(numIt, 4);

    fullTime = tic;
    lpTime = 0;
    LoopTime = zeros(10,1);
    
    x_oLog = [];
    
    
    simState = [];
    pTime = [];
    
%     pause(5)
    thrust = 60;
    pitch = 0;
    roll = 0;
    global errLog
    for i=1:numIt
        loopTime = tic;
        
        %Run planner
        if(i == 1)
            [~, p_a,v_a,R_a,omega_a, x_o, ~] = getWorldState(Client, 1, 1,x_oLog);
            x_a = [p_a;v_a;reshape(R_a, [9,1]); omega_a; x_a(19:end)];
            x_aLog(i,:) = [x_a', 0];
                
            xi_a = x_a;
            simState = x_a(1:18);
            
            Target = p_a';
%             rR = [1 0 0; 0 1 0; 0 0 1]
%             startState = [p_a' + [0 0 0], 0, 0, 0.0, 0,0,1,zeros(1,6), reshape(Rmod, [1,9]), zeros(1,3)];
%             tarState = [Target, 0, 0, 0.1, 0,0,0,zeros(1,6), reshape(Rmod, [1,9]), zeros(1,3)];
%             endState = [p_a' + [0 0 .1], zeros(1,12), reshape(Rmod, [1,9]), zeros(1,3)];
%             r = [0,startState; 1, tarState;2, endState; 100, endState];
%             r = [0, startState; 100, startState];
            
%             r = [0, tarState; 100, tarState];

%             rSine_t = (0:0.01:10)';
%             [rtr, rtc] = size(rSine_t);
%             rSine_p = [sin(2*pi*rSine_t/20)/5 + p_a(1), cos(2*pi*rSine_t/10)/5 + p_a(2), repmat(p_a(3)+0.1, rtr,rtc)];
%             [~,rSine_v] = gradient(rSine_p);
%             [~,rSine_a] = gradient(rSine_v);
%             rSine_R = repmat([1 0 0 0 -1 0 0 0 -1], rtr, rtc);
%             r = [rSine_t, rSine_p, rSine_v, rSine_a, zeros(rtr,6),rSine_R, zeros(rtr,3)];

            r = setBasedPlanner(x_a, x_o, @cost);
            fullTime = tic;
            r_log = [r_log(end,1)+r(:,1), r(:,2:end)];
        else
            if(r(1,1)+(executeTime) < toc(fullTime))      
                curTime = toc(fullTime);
%                 rAsStateTemp = interp1(r(:,1),r(:,2:end), curTime);
%                 rAsState = [rAsStateTemp(1:6), rAsStateTemp(16:27)]';
%                 if(any(isnan(rAsState)))
% %                     error('bad r state');
% %                     return
%                     rAsState = x_a;
%                 end
                [~, p_a,v_a,R_a,omega_a, x_o,~] = getWorldState(Client, 1, 1,x_oLog);
                x_a = [p_a;v_a;reshape(R_a, [9,1]); omega_a; x_a(19:end)];
                x_aLog(i,:) = [x_a; curTime];
%                 planTic = tic;
%                 sendCFPIDdata(socket,thrust, 0, pitch, roll);
                r = setBasedPlanner(x_a, x_o, @cost);
%                 toc(planTic)

%                 r = setBasedPlanner(rAsState, x_o, @cost);
%                 timeOffset = toc(fullTime);
                timeOffset = curTime;
                r(:,1) = r(:,1) + timeOffset;
                r_log = [r_log;r(:,1), r(:,2:end)];
                
                numPlans = numPlans +1;
            end
        end
        
        %Update only vehicle information
        [~, p_a,v_a,R_a,omega_a, ~, x_oLog] = getWorldState(Client, 1, 1,x_oLog);
        x_a = [p_a;v_a;reshape(R_a, [9,1]); omega_a; x_a(19:end)];

        curTime = toc(fullTime);
        x_aLog(i,:) = [x_a', curTime];
        
        [xErr,yErr,zErr,phiErr,dt] = calcErr(r,x_a,curTime);
        [thrust, yaw, pitch, roll] = PIDcontroller(xErr,yErr,zErr,phiErr,dt);
        TM_log(i,:) = [thrust, yaw, pitch, roll];
        sendCFPIDdata(socket,thrust, yaw, pitch, roll);
        errLog(end+1,:) = [xErr,yErr,zErr];
        if(any(abs([xErr,yErr,zErr])>1))
            disp('Err break')
            break
        end
        
        loopDt = toc(loopTime);
        if(loopDt < 0.020)
            java.lang.Thread.sleep((0.020 - loopDt)*1000)
        end
    end
    str = ['{"version":1, "client_name": "N/A", "ctrl": {"thrust": ',num2str(0.0),', "yaw": ',num2str(0.0),', "roll": ',num2str(0.0),', "pitch": ',num2str(0.0),'}}'];
    zmq.core.send(socket, uint8(str));
    disp('BREAK')
%     LoopTime
    ft = toc(fullTime)
    numPlans
    disp(['Average update rate = ',num2str(i/ft)])
    pause
%     errLog

    logSize = size(x_aLog)
    size(x_oLog)
    c = autumn(logSize(1));
    figure
    hold on;
    up = reshape(r_log(1,17:25), [3,3])*[0;0;.2];
    xaxis = reshape(r_log(1,17:25), [3,3])*[.2;0;0];
    yaxis = reshape(r_log(1,17:25), [3,3])*[0;.2;0];
    plot3([r_log(1,2), r_log(1,2)+up(1)], [r_log(1,3), r_log(1,3)+up(2)], [r_log(1,4), r_log(1,4)+up(3)],'k');
    plot3([r_log(1,2), r_log(1,2)+xaxis(1)], [r_log(1,3), r_log(1,3)+xaxis(2)], [r_log(1,4), r_log(1,4)+xaxis(3)],'k');
    plot3([r_log(1,2), r_log(1,2)+yaxis(1)], [r_log(1,3), r_log(1,3)+yaxis(2)], [r_log(1,4), r_log(1,4)+yaxis(3)],'k');
    scatter3(r(1,2), r(1,3), r(1,4), 'k');
    xlabel('x'); ylabel('y'); zlabel('z');
%     for i = 1:logSize(1)
%         up = reshape(x_aLog(i,7:15), [3,3])*[0;0;.1];
%         xaxis = reshape(x_aLog(i,7:15), [3,3])*[.1;0;0];
%         yaxis = reshape(x_aLog(i,7:15), [3,3])*[0;.1;0];
%         plot3([x_aLog(i,1), x_aLog(i,1)+xaxis(1)], [x_aLog(i,2), x_aLog(i,2)+xaxis(2)], [x_aLog(i,3), x_aLog(i,3)+xaxis(3)],'g');
%         plot3([x_aLog(i,1), x_aLog(i,1)+yaxis(1)], [x_aLog(i,2), x_aLog(i,2)+yaxis(2)], [x_aLog(i,3), x_aLog(i,3)+yaxis(3)],'b');
%         plot3([x_aLog(i,1), x_aLog(i,1)+up(1)], [x_aLog(i,2), x_aLog(i,2)+up(2)], [x_aLog(i,3), x_aLog(i,3)+up(3)], 'color', c(i,:));
%         scatter3(x_aLog(i,1), x_aLog(i,2), x_aLog(i,3), 'k');
%     end
    scatter3(x_aLog(:,1), x_aLog(:,2), x_aLog(:,3), 'k');
    scatter3(x_aLog(1,1), x_aLog(1,2), x_aLog(1,3), 'r');
    up = reshape(r_log(end,17:25), [3,3])*[0;0;.2];
    xaxis = reshape(r_log(end,17:25), [3,3])*[.2;0;0];
    yaxis = reshape(r_log(end,17:25), [3,3])*[0;.2;0];
    plot3([r_log(end,2), r_log(end,2)+up(1)], [r_log(end,3), r_log(end,3)+up(2)], [r_log(end,4), r_log(end,4)+up(3)],'r');
    plot3([r_log(end,2), r_log(end,2)+xaxis(1)], [r_log(end,3), r_log(end,3)+xaxis(2)], [r_log(end,4), r_log(end,4)+xaxis(3)],'g');
    plot3([r_log(end,2), r_log(end,2)+yaxis(1)], [r_log(end,3), r_log(end,3)+yaxis(2)], [r_log(end,4), r_log(end,4)+yaxis(3)],'b');
    scatter3(r_log(:,2), r_log(:,3), r_log(:,4), 'r');
    scatter3(x_oLog(:,1), x_oLog(:,2), x_oLog(:,3), 'b');
    scatter3(x_oLog(:,4), x_oLog(:,5), x_oLog(:,6), 'b');
    
    [SX,SY,SZ] = sphere();
    SXT = SX*TargetRad + Target(1);
    SYT = SY*TargetRad + Target(2);
    SZT = SZ*TargetRad + Target(3);
    tSurface = surface(SXT, SYT, SZT);
    set(tSurface,'FaceColor',[0 1 0], 'FaceAlpha',0.5,'FaceLighting','gouraud');
    hold off;
    
    figure
    subplot(3,1,1); hold on; scatter(x_aLog(:,end), x_aLog(:,1)); plot(x_aLog(:,end), x_oLog(:,1)); plot(x_aLog(:,end), x_oLog(:,4)); plot(r_log(:,1), r_log(:,2));
    subplot(3,1,2); hold on; scatter(x_aLog(:,end), x_aLog(:,2)); plot(x_aLog(:,end), x_oLog(:,2)); plot(x_aLog(:,end), x_oLog(:,5)); plot(r_log(:,1), r_log(:,3));
    subplot(3,1,3); hold on; scatter(x_aLog(:,end), x_aLog(:,3)); plot(x_aLog(:,end), x_oLog(:,3)); plot(x_aLog(:,end), x_oLog(:,6)); plot(r_log(:,1), r_log(:,4));
    
%     figure
%     xlabel('x'); ylabel('y'); zlabel('z');
%     [SX,SY,SZ] = sphere();
%     SXT = SX*TargetRad + Target(1);
%     SYT = SY*TargetRad + Target(2);
%     SZT = SZ*TargetRad + Target(3);
%     tSurface = surface(SXT, SYT, SZT);
%     set(tSurface,'FaceColor',[0 1 0], 'FaceAlpha',0.5,'FaceLighting','gouraud');
%     hold on
%     for i = 1:logSize(1)
%         up = reshape(x_aLog(i,7:15), [3,3])*[0;0;.1];
%         xaxis = reshape(x_aLog(i,7:15), [3,3])*[.1;0;0];
%         yaxis = reshape(x_aLog(i,7:15), [3,3])*[0;.1;0];
%         plot3([x_aLog(i,1), x_aLog(i,1)+xaxis(1)], [x_aLog(i,2), x_aLog(i,2)+xaxis(2)], [x_aLog(i,3), x_aLog(i,3)+xaxis(3)],'g');
%         plot3([x_aLog(i,1), x_aLog(i,1)+yaxis(1)], [x_aLog(i,2), x_aLog(i,2)+yaxis(2)], [x_aLog(i,3), x_aLog(i,3)+yaxis(3)],'b');
%         plot3([x_aLog(i,1), x_aLog(i,1)+up(1)], [x_aLog(i,2), x_aLog(i,2)+up(2)], [x_aLog(i,3), x_aLog(i,3)+up(3)], 'color', c(i,:));
%         scatter3(x_aLog(i,1), x_aLog(i,2), x_aLog(i,3), 'k');
%     end
    
    TM_log
    save('TestData8.mat', 'x_aLog', 'x_oLog', 'r_log', 'Target');
end

function [xErr, yErr, zErr, phiErr, dt] = calcErr(r, x_a, t)
    global resetPIDs
    persistent lastT yawset
    if isempty(lastT) || resetPIDs
        lastT = 0;
        yawset = 1;
    end
    dt = t - lastT;

%     if(t < r(end,1))
%         refpt = interp1(r(:,1),r(:,2:end),t)';
%     else
        refpt = r(end,2:end)';
%     end
    
    xErr = x_a(1) - refpt(1);
    yErr = x_a(2) - refpt(2);
    zErr = -x_a(3) + refpt(3);
    [xdeg,ydeg,phiErr] = rot2eul(reshape(x_a(7:15),[3,3]));
    if(abs(phiErr)>5 & ~ yawset)
        yawset = 1;
        xErr = 0;
        yErr = 0;
    end
        
end


function [T,Y,P,R] = PIDcontroller(xErr, yErr, zErr, phiErr, dt)
    global resetPIDs
    persistent x_cumm_error y_cumm_error z_cumm_error phi_cumm_error
    persistent x_prev_error y_prev_error z_prev_error phi_prev_error
    
    if resetPIDs || isempty(x_cumm_error)
        x_cumm_error = 0;
        y_cumm_error = 0;
        z_cumm_error = 0;
        phi_cumm_error = 0;
        x_prev_error = xErr;
        y_prev_error = yErr;
        z_prev_error = zErr;
        phi_prev_error = phiErr;
        resetPIDs = 0;
    end
    
    K_p_x = 75;
    K_i_x = 0;
    K_d_x = 0;
    
    K_p_y = K_p_x;
    K_i_y = K_i_x;
    K_d_y = K_d_x;
    
    K_p_z = 250;
    K_i_z = 1;
    K_d_z = 0.01;
    GravThrust = 62;
    
    K_p_phi = 2;
    K_i_phi = 0;
    K_d_phi = 0.01;
    
    x_cumm_error = x_cumm_error + xErr*dt;
    R = (K_p_x*xErr) + (K_i_x*x_cumm_error) + (K_d_x*(xErr-x_prev_error)/dt);
    x_prev_error = xErr;
    
    y_cumm_error = y_cumm_error + yErr*dt;
    P = (K_p_y*yErr) + (K_i_y*y_cumm_error) + (K_d_y*(yErr-y_prev_error)/dt);
    y_prev_error = yErr;
    
    phi_cumm_error = phi_cumm_error + phiErr*dt;
    Y = (K_p_phi*phiErr) + (K_i_phi*phi_cumm_error) + (K_d_phi*(phiErr-phi_prev_error)/dt);
    phi_prev_error = phiErr;
    
    z_cumm_error = z_cumm_error + zErr*dt;
    T = GravThrust + (K_p_z*zErr) + (K_i_z*z_cumm_error) + (K_d_z*(zErr-z_prev_error)/dt);
    z_prev_error = zErr;
end

function cleanupFunc(Client, socket)
    Client.Uninitialize();
    str = ['{"version":1, "client_name": "N/A", "ctrl": {"thrust": ',num2str(0.0),', "yaw": ',num2str(0.0),', "roll": ',num2str(0.0),', "pitch": ',num2str(0.0),'}}'];
%     str = ['{"m1": ',num2str(0),', "m2": ',num2str(0),', "m3": ',num2str(0),', "m4": ',num2str(0),'}'];
    zmq.core.send(socket, uint8(str));
end

function [out, decelTerm, AngTerm] = cost(traj,plotCost)
    persistent Tar TarRad
    if(isempty(Tar))
        global Target TargetRad
        Tar = Target;
        TarRad = TargetRad;
    end
%     global Target TargetRad executeTime planTime
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
    dist = max(norm(exState(end,1:3)-Tar) - TarRad, 0);
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
    
% %     R = reshape(exState(end,7:15),[3,3]);
% %     phi = acos(dot(R*[0;0;1], [0;0;1]));
%     phi = acos(dot([exState(end,9);exState(end,12);exState(end,15)], [0;0;1]));
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
    
    decelTerm = norm(exState(4:6))/10;
    AngTerm = 0;%phi;
    
%     out = (max(norm(AT-proj)-TargetRad, 0) + 4*norm(rej))*poseTerm;
    out = dist + decelTerm;%*poseTerm + decelTerm;
%     out = dist*poseTerm + norm(exState(6:8))/5 + hysteresisCost/10;
end

function R = doubleCover(q)
    R = eye(3) + 2*q(1)*S(q(2:4)) + 2*S(q(2:4))^2;
end

function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end

%[pitch, roll, yaw]
function sendCFPIDdata(socket,thrust, yaw, pitch, roll)
    thrust = min(max(thrust, 45), 68);
    yaw = min(max(yaw, -200), 200);
    pitch = min(max(pitch, -30), 30);
    roll = min(max(roll, -30), 30);
    str = ['{"version":1, "client_name": "N/A", "ctrl": {"thrust": ',num2str(thrust),', "yaw": ',num2str(yaw),', "roll": ',num2str(roll),', "pitch": ',num2str(pitch),'}}'];
    if(zmq.core.send(socket, uint8(str)) == -1)
        error("Bad send")
    end
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