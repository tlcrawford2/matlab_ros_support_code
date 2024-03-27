%% Run reset if doing nothing before
%Reset
% Call pickTopDownCan
pickTopDownCan;

% Create a subscriber to /joint_states topic (to get info about where the robot is currently)
jointSub = rossubscriber("/joint_states");

% Receive the data and use it to compute IKs
jointStateMsg = jointSub.LatestMessage;

% receive current robot configuration
jointStateMsg = receive(jointSub,3) ;
% Get the latest joint angles by calling receive

% Update configuration initial guess to correct order
% updated config
initialIKGuess(1) = jointStateMsg.Position(4);  % Shoulder Pan
initialIKGuess(2) = jointStateMsg.Position(3);  % Shoulder Tilt
initialIKGuess(3) = jointStateMsg.Position(1);  % Elbow
initialIKGuess(4) = jointStateMsg.Position(5);  % W1
initialIKGuess(5) = jointStateMsg.Position(6);  % W2
initialIKGuess(6) = jointStateMsg.Position(7);  % W3
show(UR5e,initialIKGuess);

% Set end-effector pose to place above can (only change height - up)
gripperX = 0.8;
gripperY = -0.035;
gripperZ = 0.4;

% Translation & Rotation
gripperTranslation = [gripperY gripperX gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians

% Create homogeneous transform using end-effector pose
tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

% Compute the IKs
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);

% Send joint trajectory goal
UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)];

% Use packing function to fill names and positions appropriately
trajGoal = packTrajGoal(UR5econfig,trajGoal);

% Send to the action server
waitForServer(trajAct);
sendGoalAndWait(trajAct,trajGoal);

% Display end position array
Final_end_effector_position = [gripperTranslation gripperRotation]

