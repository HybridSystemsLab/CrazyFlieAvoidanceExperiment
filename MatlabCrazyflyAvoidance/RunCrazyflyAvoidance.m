close all
% clear
clc

global HEQOptions;
HEQOptions = odeset('AbsTol', 1e-1,'RelTol',1e-1);%, 'InitialStep',1e-3);%,'MaxStep',.1);

global gamma lambda sigma_max planTime planStep executeTime worldTimeStep obstacleSize FirstRunEject ConstThrust objColStep prevRef ObjPlot55
gamma = 9.8;
lambda = .6;
sigma_max = 0.2;
planTime = 0.3;
planStep = 0.1;
executeTime = 0.05;
worldTimeStep = 0.1;
obstacleSize = 0.05;
FirstRunEject = 0;
ConstThrust = 1;
objColStep = 200;
prevRef = [];
ObjPlot55 = 0;

global Target TargetRad
Target = [0,0,2];
TargetRad = 0.3;

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

% p_x_a; p_y_a; p_z_a; v_x_a; v_y_a; v_z_a; R_11-R_31; R_12-R_32; R_13-R_33; omega; z; h; qhat
startq = [0.01; 1; 0; 0]/norm([0.01; 1; 0; 0]);
startR = doubleCover([0 1 0 0]/norm([0 1 0 0]));
x_a = [zeros(6,1); reshape(startR,[9,1]); zeros(3,1); zeros(3,1); 1; startq];
% p_x_o; v_x_o; p_y_o; v_y_o; p_z_o; v_z_o; sigmaMin; sigmaMax
xi_o_0 = [-.03, 0, 0, 1, 4, 0, -sigma_max, sigma_max];
xi_o_1 = [.03, 0, 1, -0.5, 2, 0, -sigma_max, sigma_max];
x_o = [mat2cell([0,0,xi_o_0], 1, 10);mat2cell([0,0,xi_o_1], 1, 10)];
xi_p = 0;

t_a = 0;
j_a = 0;
r_log = zeros(1,31);

% %Set up threads
% p = gcp('nocreate');
% if isempty(p)
%     p = parpool(1);
% end

%Set up optitrak
dllPath = fullfile('c:','Users','yzeleke','Desktop','HSL_exp','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
Client = NatNetML.NatNetClientML(0);
HostIP = char('128.114.56.19');
Client.Initialize(HostIP, HostIP);
cleanupObj = onCleanup(@()Client.Uninitialize());

%Connect to cfclient
addpath(genpath('C:\Users\adrames\Desktop\Aerial Vehicle\MatlabZMQTest\matlab-zmq\lib'))
addpath('C:\Users\adrames\Desktop\Aerial Vehicle\MatlabZMQTest\libzmq')
context = zmq.core.ctx_new();
socket  = zmq.core.socket(context, 'ZMQ_PUSH');
address = 'tcp://127.0.0.1:1212';
zmq.core.connect(socket, address);

%Load motion primatives
if(~exist('motionPrims', 'var'))
    global motionPrims
    disp('Loading motionPrims from file')
%     motionPrims = load('D:\adrames\motionData_omegaMax5_numStep11_numVals7\motionData.mat');
    motionPrims = load('D:\adrames\motionData_omegaMax5_numStep11_numVals7\motionData.mat');
    disp('Done loading motionPrims from file')
end

%Run planner and update world
global xi_a 
xi_a = [];
hvc_tocs = zeros(30,1);
T = 50;
M = [0;0;0];
r = [0,0, 0, .8,zeros(1,9),reshape(eye(3),[1,9]),zeros(1,3); 1000000,0, 0, .8,zeros(1,9),reshape(eye(3),[1,9]),zeros(1,3)];
for i=1:1
%     hvc = tic;
    [t, p_a,v_a,R_a,omega_a] = getQuadState(Client, 1);
    x_a = [p_a;v_a;reshape(R_a, [9,1]); omega_a; x_a(19:end)]
    %Run planner
    if(isempty(xi_a))
        xi_a = x_a;
%         r = [t,p_a',zeros(1,9),reshape(eye(3),[1,9]),zeros(1,3);t+2,p_a'+[.0, .0, .3],zeros(1,9),reshape(eye(3),[1,9]),zeros(1,3)];
    end
%     if(i == 1)
        plantic = tic;
        r = setBasedPlanner(x_a, x_o, @cost);
        toc(plantic)
        disp('Done ref gen')
% %         parfeval(@setBasedPlanner,0,x_a(end,:), x_o, @cost);
%     else
% %         x_a(end,:)
%         rAsStateTemp = interp1(prevRef(:,1),prevRef(:,2:end), executeTime);
%         rAsState = [rAsStateTemp(1:6), rAsStateTemp(16:27)];
%         r = setBasedPlanner(rAsState,x_o,@cost);
%     end
%     r_log = [r_log;r_log(end,1)+r(:,1), r(:,2:end)];
    
    [Tnew, Mnew] = hybridVehicleController(r,x_a,t);
    if(any(0 ~= [Tnew;Mnew]))
        T = Tnew;
        M = Mnew;
    end
    
%     [t_a,j_a,x_a] = updateVehicle(r,t_a,j_a,x_a);
    
    %Update obstacles
%     x_o = updateObjs(x_o);
    
%     plotResults(t_a, x_a, x_o, r_log(2:end,:));
%     if(norm(r(end,5:7))>1)
%         ConstThrust = 0;
%     else
%         ConstThrust = 1;
%     end
%     dist = max(norm(x_a(end,3:5)-Target) - TargetRad, 0);
%     if(dist<0 && norm(x_a(end,6:8))<0.2)
%         break
%     end

    sendCFdata(socket,T,M,x_a(7:15),x_a(16:18))
%     hvc_tocs(i) = toc(hvc);
end
str = ['{"version":1, "client_name": "N/A", "ctrl": {"thrust": ',num2str(0.0),', "yaw": ',num2str(0.0),', "roll": ',num2str(0.0),', "pitch": ',num2str(0.0),'}}'];
zmq.core.send(socket, uint8(str));

xi_a
x_a
x_a(1:3) - xi_a(1:3)
r(2,2:4)' - x_a(1:3)

% meanT = mean(hvc_tocs)
% [maxT, maxID] = max(hvc_tocs)
% histogram(hvc_tocs)
% [maxT, b] = max(hvc_tocs(2:end))

function newU = updateObjs(U)
    global executeTime
    [nobj,~] = size(U);
    newU = cell(nobj,1);
    for i=1:nobj
        x_o = cell2mat(U(i));
        [new_t, new_j, new_x_o] = bouncingBallModel(x_o(end,3:end), executeTime);
        out = [x_o; x_o(end,1)+new_t, x_o(end,2)+new_j, new_x_o];
        [r,c] = size(out);
        newU(i) = mat2cell(out,r,c);
    end
end

% function [t_out,j_out,x_a_out] = updateVehicle(r,t,j,x_a)
%     global executeTime usingPointMassVehicle prevRef
%     t_new = 0;
%     j_new = 0;
%     x_a_new = zeros(1,28);
%     if(~isempty(usingPointMassVehicle) & usingPointMassVehicle == 1)
%         [t_new,j_new,x_a_new] = pointMassVehicle(r, x_a, executeTime);
%     else
%         [t_new,j_new,x_a_new] = simHybridVehicleOnly(r, x_a(end,:), executeTime);
% %         pause
%     end
%     
%     t_out = [t; t(end)+t_new(2:end)];
%     j_out = [j; j(end)+j_new(2:end)];
%     x_a_out = [x_a; x_a_new(2:end,:)];
%     
%     prevRef = r;
% end

function [out, decelTerm, AngTerm] = cost(traj,plotCost)
    global Target TargetRad executeTime planTime
%     dist = max(norm(traj(end,3:5)-Target) - TargetRad, 0);
    if(~exist('plotCost','var'))
        plotCost = 0;
    end
    if plotCost == 1
        exState = traj;
    else
%         exState = interp1(traj(:,1),traj(:,3:end),traj(end,1) - planTime + executeTime)
        exState = traj(end,3:end);
    end
    if(any(isnan(exState)))
        traj(end,1) - planTime + executeTime
        traj
        AngTerm = inf;
        decelTerm = inf;
        out = inf;
        pause
        return
    end
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
    
    R = reshape(exState(end,7:15),[3,3]);
    phi = acos(dot(R*[0;0;1], [0;0;1]));
    phi = min(abs(phi), abs(phi-pi));
    poseTerm = 1;%2-cos(phi)^4;
    if(phi>=pi/4)
        poseTerm = 10000;
        if(phi>=pi/2)
            poseTerm = Inf;
        end
    end

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
    AngTerm = phi;
    
%     out = (max(norm(AT-proj)-TargetRad, 0) + 4*norm(rej))*poseTerm;
    out = dist*poseTerm + decelTerm;
%     out = dist*poseTerm + norm(exState(6:8))/5 + hysteresisCost/10;
end

function R = doubleCover(q)
    R = eye(3) + 2*q(1)*S(q(2:4)) + 2*S(q(2:4))^2;
end

function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end

function sendCFdata(socket,T,M,Rin,omega)
    t = 0.02; % in sec

    R = reshape(Rin,[3,3]);
    global InertialTensor MaxMotorThrust
    omegadot = -InertialTensor\S(omega)*InertialTensor*omega + InertialTensor\M;
    omegaAvg = omega + (omegadot*t)/2;
    Rdot = R*S(omegaAvg);
    targetR = R + Rdot*t;
    targetR = targetR * [1 0 0; 0 -1 0; 0 0 -1];
    [pitch,roll,yaw] = rot2eul(targetR);
    thrust = T/(MaxMotorThrust*4)*100;

    str = ['{"version":1, "client_name": "N/A", "ctrl": {"thrust": ',num2str(thrust),', "yaw": ',num2str(yaw/t),', "roll": ',num2str(roll),', "pitch": ',num2str(pitch),'}}'];
    zmq.core.send(socket, uint8(str));
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
end