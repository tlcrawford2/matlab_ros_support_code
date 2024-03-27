%% Run reset if doing nothing before


%% Call moveTopDownCan
moveTopDownCan
% Create function to call code from notebook 09

%% Close fingers around rCan3
% Create a client based on the action topic
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','struct');

% Create a goal message
gripGoal = rosmessage(grip_client);

% Create a variable to set displacement of fingers (open)
gripPos = 0.225; % 0 is fully open, 0.8 is fully closed

% Populate the trajectory goal
gripGoal = packGripGoal_struct(gripPos,gripGoal);

% Send ROS action client message
waitForServer(grip_client);
sendGoalAndWait(grip_client,gripGoal);

% Display end position array
Final_end_effector_position = [gripperTranslation gripperRotation]