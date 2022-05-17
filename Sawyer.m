clear; clc;
sawyer = loadrobot('rethinkSawyer','DataFormat','row','Gravity',[0 0 -9.81]);

currentRobotConfig = homeConfiguration(sawyer);

numJoints = numel(currentRobotConfig);
endEffector = "right_hand";

timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

startConfig = [0;0;-1.5;0;2.3;0;-2.3;0.5]';
currentRobotConfig = startConfig;

jointInit = currentRobotConfig;

taskInit = getTransform(sawyer,jointInit,endEffector);

taskFinal = trvec2tform([0.7,0.5,0.7])*axang2tform([0 1 0 pi]);


distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];



tsMotionModel = taskSpaceMotionModel('RigidBodyTree',sawyer,'EndEffectorName','right_l6');

tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;
 
q0 = currentRobotConfig; 
qd0 = zeros(size(q0));

[tTask,stateTask] = ode15s(@(t,state) TimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);


show(sawyer,currentRobotConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
    poseNow = getTransform(sawyer,configNow,endEffector);
    show(sawyer,configNow,'PreservePlot',false,'Frames','off');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
    drawnow;
end


function stateDot = TimeBasedTaskInputs(motionModel, timeInterval, initialTform, finalTform, t, state)

%TimeBasedTaskInputs Pass time-varying inputs to the taskSpaceMotionModel derivative
%   Since the taskSpaceMotionModel derivative method is updated at an
%   instant in time, a wrapper function is needed to provide time-varying
%   tracking inputs. This function computes the value of the transform
%   trajectory at an instant in time, t, and provides that input to the
%   derivative of the associated taskSpaceMotionModel at that same instant.
%   The resultant state derivative can be passed to an ODE solver to
%   compute the tracking behavior.

% Copyright 2019 The MathWorks, Inc.

[refPose, refVel] = transformtraj(initialTform, finalTform, timeInterval, t);

stateDot = derivative(motionModel, state, refPose, refVel);

end