% pts = []
% dtable = {};
% for i1 = 1:10
%     for i2 = 1:10
%         for i3 = 1:10
% %             for i4 = 1:10
% %                 for i5 = 1:10
%                     pts(end+1,:) = [i1,i2,i3];
%                     dtable(i1,i2,i3) = mat2cell([i1,i2,i3],1,3);
% %                 end
% %             end
%         end
%     end
% end
% 
% [a,~] = dsearchn(pts,[1.4 2.6 1.6])
% disp('pts:')
% dintex = pts(a,:)
% disp(cell2mat(dtable(dintex(1),dintex(2),dintex(3))))


% -----------------------------------------------------

% p = gcp('nocreate');
% if isempty(p)
%     p = parpool(1);
% end
% slowvals = 0;
% fastvals = 0;
% f = parfeval(@slowFunc,0);
% while(slowvals < 2)
%     if(strcmp(f.State,'finished'))
%         f.State
%         retVal = fetchNext(f)
%         slowvals = slowvals + retVal
%         f = parfeval(@slowFunc,0);
%     end
%     fastvals = fastvals+ fastFunc();
% end
% fastvals
% cancel(f)
% 
% function out = fastFunc()
%     java.lang.Thread.sleep(100)
%     out = 1;
% end
% 
% function out = slowFunc()
%     java.lang.Thread.sleep(500)
%     out = 1;
% end

% ----------------------------------------------

T = 0.2744
M = [0.4019; 0.4038; 0.0582]*1e-3
R = [1 0 0; 0 -1 0; 0 0 -1];
t = 0.010; % in sec

omega = [0;0;0]; % x(16:18)
omegadot = -InertialTensor\S(omega)*InertialTensor*omega + InertialTensor\M;
omegaAvg = omega + (omegadot*t)/2;
Rdot = R*S(omegaAvg);
targetR = R + Rdot*t;

targetR = targetR * [1 0 0; 0 -1 0; 0 0 -1];

[a,b,c] = rot2eul(targetR);

thrust = T
p = a
r = b
yrate = 0


function R = doubleCover(q)
    R = eye(3) + 2*q(1)*S(q(2:4)) + 2*S(q(2:4))^2;
end

function out = S(x)
    out = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
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
        thetaZ = atan2(-r12 , r11);
        thetaX = 0;
    end
end