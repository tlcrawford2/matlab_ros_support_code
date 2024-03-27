%% Run reset if doing nothing before 
Reset
%% OPENING GRIPPER
%grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat', 'struct')

%gripGoal = rosmessage(grip_client)

%gripPos = 0.8 %0 is fully closed, 0.8 is fully open

%gripGoal = packGripGoal(gripPos,gripGoal)

%sendGoal(grip_client,gripGoal)

% TRAJECTORY 
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                          'control_msgs/FollowJointTrajectory',...
                          'DataFormat', 'struct') ;

% instantiate the goal
trajGoal = rosmessage(trajAct);

trajAct.FeedbackFcn = []; 

% Joint subscriber
jointSub = rossubscriber("/joint_states");

jointStateMsg = jointSub.LatestMessage;


% Loading robot
UR5e = loadrobot('universalUR5e', DataFormat="row");

tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

% Create Inverse kinematics solver
ik = inverseKinematics("RigidBodyTree",UR5e); 

% configuration weights for IK solver 
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];

% receive current robot configuration
jointStateMsg = receive(jointSub,3) ;

initialIKGuess = homeConfiguration(UR5e);

% order of the names
jointStateMsg.Name;

% updated config
initialIKGuess(1) = jointStateMsg.Position(4);  % Shoulder Pan
initialIKGuess(2) = jointStateMsg.Position(3);  % Shoulder Tilt
initialIKGuess(3) = jointStateMsg.Position(1);  % Elbow
initialIKGuess(4) = jointStateMsg.Position(5);  % W1
initialIKGuess(5) = jointStateMsg.Position(6);  % W2
initialIKGuess(6) = jointStateMsg.Position(7);  % W3
show(UR5e,initialIKGuess);

% positions and translation
gripperX = 0.8;
gripperY = -0.035;
gripperZ = 0.13;

gripperTranslation = [gripperY gripperX gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians


tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform


% Compute the IKs
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)];

% fill names and positions
trajGoal = packTrajGoal(UR5econfig,trajGoal);

% send
sendGoal(trajAct,trajGoal);

% Display end position array
Final_end_effector_position = [gripperTranslation gripperRotation]




