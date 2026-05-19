function  trajGoal, jointSub = poweron(ip)
    %Iniciliaza o Gazebo
    clear; clc;
    rosIP = ip; 
    rosshutdown;
    pause(2);
    rosinit(rosIP,11311);
    %Inicializa trajAct e gripAct
    [trajAct,trajGoal] = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','struct');
    trajAct.FeedbackFcn = []; 
    trajAct.ResultFcn = []; 
    jointSub = rossubscriber("/joint_states",'DataFormat','struct')
    waitForServer(trajAct);
end