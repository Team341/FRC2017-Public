%% These routines illustrates the mapping of camera detections into the world coordinate frame
% @author AJN

function IllustrateCoordinateTransforms
    %% Define the robot information
    robot = struct('pos', [0, 00],...        % in [DR, CR]
                   'yaw', 0 * pi/180,...    % rad
                   'size', [27 33]);        % in [length, width]

    turret = struct('pos', [-5, -8],...      % in [DR, CR]   (translation from robot base pos
                    'yaw', 14 * pi/180,...   % rad (rotation from robot base yaw)
                    'size', 4.5);           % in radius of turret

    camera = struct('pos', [2, 0],...   % in [DR, CR] (translation from turret pos)
                    'yaw', 0 * pi/180,...   % rad (rotation of camera pointing direction from turret forward direction)
                    'size', [0.5, 1]);      % in [length, width]

	%% Define the target information
    target = struct('pos', [102 0],...             % in [DR, CR]
                    'yaw', 0* pi/180,...           % rad
                    'size', [0.5, 2 + 12 +2]);      % in [length, width]
                
	%% Create the figure to draw evertying on
    hf = clf(figure(1));
    ax(1) = subplot(2,2,1, 'Parent', hf);
    xlabel('Alliance Wall (in)')
    ylabel('Down Field (in)');
    
    ax(2) = subplot(2,2,2, 'Parent', hf);
    title(ax(2), 'Robot Frame');
    
    ax(3) = subplot(2,2,3, 'Parent', hf);
    title(ax(3), 'Turret Frame');
    
    ax(4) = subplot(2,2,4, 'Parent', hf);
    title(ax(4), 'Camera Frame');
    
    for n = 1:length(ax)
        hold(ax(n), 'all');
        grid(ax(n), 'on');
        axis(ax(n), 'equal');
    end
    
    %% Draw the robot
	[r,t,c] = DrawScene(ax(1), robot, turret, camera);
    DrawRobotFrame(ax(2), robot, turret, camera, target);
    DrawTurretFrame(ax(3), robot, turret, camera, target);
    DrawCameraFrame(ax(4), robot, turret, camera, target);
    
    %% Draw the target
    tgt = DrawTarget(ax, target);
    
    %% Draw a ray from the origin of the 3 coordinate frames to the target
    plot(ax(1), [c(2) tgt(2)], [c(1) tgt(1)], 'r')
    plot(ax(1), [t(2) tgt(2)], [t(1) tgt(1)], 'Color', [0 0.5 0])
    plot(ax(1), [r(2) tgt(2)], [r(1) tgt(1)], 'k')
        
    %% Compute the measurements from the camera to the target
    dP = tgt - c;
    range = norm(dP);
    angle = wrapPi(atan2(dP(2), dP(1)) - (camera.yaw + turret.yaw + robot.yaw));
    
%     str = {sprintf('Range to Target: %0.2f (in)', range),...
%            sprintf('Angle to Target: %0.2f (deg)', angle*180/pi),...
%            'World Frame'};
% 	title(ax(1), str)
    
    %% Get the world coord of the target based on the measurement
    tgtX_inCamFrame = range*cos(angle);
    tgtY_inCamFrame = range*sin(angle);
    [tgtX, tgtY] = rotateFrom(tgtX_inCamFrame, tgtY_inCamFrame, camera);
    [tgtX, tgtY] = rotateFrom(tgtX, tgtY, turret);
    [tgtX, tgtY] = rotateFrom(tgtX, tgtY, robot);
    
    error = norm( tgt - [tgtX tgtY]);
    fprintf('Mapping position error: %0.2f (in)\n', error);
    
    %% Convert the target position into the turret frame
    [tgtX_inTurtFrame, tgtY_inTurtFrame] = rotateTo(tgtX, tgtY, robot);
    [tgtX_inTurtFrame, tgtY_inTurtFrame] = rotateTo(tgtX_inTurtFrame, tgtY_inTurtFrame, turret);
    
    turretRangeToTarget = norm([tgtX_inTurtFrame, tgtY_inTurtFrame]);
    turretAngleToTarget = atan2(tgtY_inTurtFrame, tgtX_inTurtFrame);
    
    dP = tgt - t;
    gtTurretAngle = wrapPi(atan2(dP(2), dP(1)) - (turret.yaw + robot.yaw));
    fprintf('Desired Turret Angle Error: %0.2f (deg)\n', (turretAngleToTarget - gtTurretAngle) * 180/pi)
        
    
    str = {sprintf('Shooter Range to Target: %0.2f (in)', turretRangeToTarget),...
           sprintf('Desired Turret Angle: %0.2f (deg)', ((turretAngleToTarget + turret.yaw))*180/pi),...
           sprintf('Error to Desired Angle: %0.2f (deg)', ((turretAngleToTarget + turret.yaw) -turret.yaw)*180/pi),...
           'World Frame'};
	title(ax(1), str)
    
    
end

function [r, t, c] = DrawScene(ax, robot, turret, camera)
    
    % Draw a square for the robot base
    baseX = robot.size(1)*[1/2 1/2 -1/2 -1/2 1/2];
    baseY = robot.size(2)*[1/2 -1/2 -1/2 1/2 1/2];
    
    [robotX, robotY] = rotateFrom(baseX, baseY, robot);
    r = robot.pos;
    
    plot(ax(1), robotY, robotX, 'k');

    % Draw a circle for the turret
    ang = linspace(0, 2*pi, 20);
    baseX = turret.size*[0 cos(ang)];
    baseY = turret.size*[0 sin(ang)];
    
    [tXinRobotFrame, tYinRobotFrame] = rotateFrom(baseX, baseY, turret);
    
    [turretX, turretY] = rotateFrom(tXinRobotFrame, tYinRobotFrame, robot);
    [t(1), t(2)] = rotateFrom(turret.pos(1), turret.pos(2), robot);
    
    plot(ax(1), turretY, turretX, 'Color', [0 0.5 0]);
    
    % Draw a square for the camera
    baseX = camera.size(1)*[1/2 1/2 -1/2 -1/2 1/2 nan [1/2 1/2 0 0 1/2]+1/2];
    baseY = camera.size(2)*[1/2 -1/2 -1/2 1/2 1/2 nan 1/5 -1/5 -1/5 1/5 1/5];
    
    [cXinTurretFrame, cYinTurretFrame] = rotateFrom(baseX, baseY, camera);
    
    [cXinRobotFrame, cYinRobotFrame] = rotateFrom(cXinTurretFrame, cYinTurretFrame, turret);
    
    [cameraX, cameraY] = rotateFrom(cXinRobotFrame, cYinRobotFrame, robot);
    [c(1), c(2)] = rotateFrom(camera.pos(1), camera.pos(2), turret);
    [c(1), c(2)] = rotateFrom(c(1), c(2), robot);
    
    plot(ax(1), cameraY, cameraX, 'Color', [0.75 0 0]);
    
end

function DrawRobotFrame(ax, robot, turret, camera, target)
    
    % Draw a square for the robot base
    baseX = robot.size(1)*[1/2 1/2 -1/2 -1/2 1/2];
    baseY = robot.size(2)*[1/2 -1/2 -1/2 1/2 1/2];
    
    plot(ax(1), baseY, baseX, 'k');

    % Draw a circle for the turret
    ang = linspace(0, 2*pi, 20);
    baseX = turret.size*[0 cos(ang)];
    baseY = turret.size*[0 sin(ang)];
    
    [turretX, turretY] = rotateFrom(baseX, baseY, turret);
    
    plot(ax(1), turretY, turretX, 'Color', [0 0.5 0]);
    
    % Draw a square for the camera
    baseX = camera.size(1)*[1/2 1/2 -1/2 -1/2 1/2 nan [1/2 1/2 0 0 1/2]+1/2];
    baseY = camera.size(2)*[1/2 -1/2 -1/2 1/2 1/2 nan 1/5 -1/5 -1/5 1/5 1/5];
    
    [cXinTurretFrame, cYinTurretFrame] = rotateFrom(baseX, baseY, camera);
    
    [cameraX, cameraY] = rotateFrom(cXinTurretFrame, cYinTurretFrame, turret);
    
    plot(ax(1), cameraY, cameraX, 'Color', [0.75 0 0]);
    
    % Draw the target
    baseX = target.size(1)*[1/2 1/2 -1/2 -1/2 1/2];
    baseY = target.size(2)*[1/2 -1/2 -1/2 1/2 1/2];
    
    [targetX, targetY] = rotateFrom(baseX, baseY, target);
    
    [targetX, targetY] = rotateTo(targetX, targetY, robot);
    plot(ax, targetY, targetX, 'b');
    
end

function DrawTurretFrame(ax, robot, turret, camera, target)
    
    % Draw a square for the robot base
    baseX = robot.size(1)*[1/2 1/2 -1/2 -1/2 1/2];
    baseY = robot.size(2)*[1/2 -1/2 -1/2 1/2 1/2];
    
%     [robotX, robotY] = rotateFrom(baseX, baseY, robot);
    [robotX, robotY] = rotateTo(baseX, baseY, turret);
    
    plot(ax(1), robotY, robotX, 'k');

    % Draw a circle for the turret
    ang = linspace(0, 2*pi, 20);
    baseX = turret.size*[0 cos(ang)];
    baseY = turret.size*[0 sin(ang)];
    
    plot(ax(1), baseY, baseX, 'Color', [0 0.5 0]);
    
    % Draw a square for the camera
    baseX = camera.size(1)*[1/2 1/2 -1/2 -1/2 1/2 nan [1/2 1/2 0 0 1/2]+1/2];
    baseY = camera.size(2)*[1/2 -1/2 -1/2 1/2 1/2 nan 1/5 -1/5 -1/5 1/5 1/5];
    
    [cameraX, cameraY] = rotateFrom(baseX, baseY, camera);
    
    plot(ax(1), cameraY, cameraX, 'Color', [0.75 0 0]);
    
    % Draw the target
    baseX = target.size(1)*[1/2 1/2 -1/2 -1/2 1/2];
    baseY = target.size(2)*[1/2 -1/2 -1/2 1/2 1/2];
    
    [targetX, targetY] = rotateFrom(baseX, baseY, target);
    [targetX, targetY] = rotateTo(targetX, targetY, robot);
    [targetX, targetY] = rotateTo(targetX, targetY, turret);
    
    plot(ax, targetY, targetX, 'b');
    
end

function DrawCameraFrame(ax, robot, turret, camera, target)
    
    % Draw a square for the robot base
    baseX = robot.size(1)*[1/2 1/2 -1/2 -1/2 1/2];
    baseY = robot.size(2)*[1/2 -1/2 -1/2 1/2 1/2];
    
%     [robotX, robotY] = rotateFrom(baseX, baseY, robot);
    [robotX, robotY] = rotateTo(baseX, baseY, turret);
    [robotX, robotY] = rotateTo(robotX, robotY, camera);
    
    plot(ax(1), robotY, robotX, 'k');

    % Draw a circle for the turret
    ang = linspace(0, 2*pi, 20);
    baseX = turret.size*[0 cos(ang)];
    baseY = turret.size*[0 sin(ang)];
    
    [turretX, turretY] = rotateTo(baseX, baseY, camera);
    
    plot(ax(1), turretY, turretX, 'Color', [0 0.5 0]);
    
    % Draw a square for the camera
    baseX = camera.size(1)*[1/2 1/2 -1/2 -1/2 1/2 nan [1/2 1/2 0 0 1/2]+1/2];
    baseY = camera.size(2)*[1/2 -1/2 -1/2 1/2 1/2 nan 1/5 -1/5 -1/5 1/5 1/5];
    
    plot(ax(1), baseY, baseX, 'Color', [0.75 0 0]);
    
    % Draw the target
    baseX = target.size(1)*[1/2 1/2 -1/2 -1/2 1/2];
    baseY = target.size(2)*[1/2 -1/2 -1/2 1/2 1/2];
    
    [targetX, targetY] = rotateFrom(baseX, baseY, target);
    [targetX, targetY] = rotateTo(targetX, targetY, robot);
    [targetX, targetY] = rotateTo(targetX, targetY, turret);
    [targetX, targetY] = rotateTo(targetX, targetY, camera);
    
    plot(ax, targetY, targetX, 'b');
    
end

function tgt = DrawTarget(ax, target)
     % Draw the 2 squares for the gear target
    baseX = target.size(1)*[1/2 1/2 -1/2 -1/2 1/2];
    baseY = target.size(2)*[1/2 -1/2 -1/2 1/2 1/2];
    %baseX = target.size(1)*[1/2 1/2 -1/2 -1/2 1/2 nan 1/2 1/2 -1/2 -1/2 1/2];
    %baseY = target.size(2)*[1/2 1/2 1/2 1/2 1/2 nan -1/2 -1/2 -1/2 -1/2 -1/2] - [2 0 0 2 2 nan 0 -2 -2 0 0];
    
    [targetX, targetY] = rotateFrom(baseX, baseY, target);
    tgt = target.pos;
    
    plot(ax(1), targetY, targetX, 'b');
end

function [newX, newY] = rotateFrom(x,y, frame)
    newX = x*cos(frame.yaw) - y*sin(frame.yaw) + frame.pos(1);
    newY = x*sin(frame.yaw) + y*cos(frame.yaw) + frame.pos(2);
end

function [newX, newY] = rotateTo(x,y, frame)
    x = x - frame.pos(1);
    y = y - frame.pos(2);
    newX = x*cos(frame.yaw) + y*sin(frame.yaw);
    newY = -x*sin(frame.yaw) + y*cos(frame.yaw);
end

function y = wrapPi(x)
    y = x - sign(x)*2*pi.*fix((abs(x)+pi)/(2*pi));
end