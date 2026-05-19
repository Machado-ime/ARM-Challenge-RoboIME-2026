addpath("C:\Users\roboime\Desktop\RoboIME Second Submission\")
clc;
clear;
close all;
pause(2);
rosIP = "192.168.187.128";
rosshutdown;
pause(2);
rosinit(rosIP,11311);

%cliente Traj
[trajAct,trajGoal] = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','struct');
trajAct.FeedbackFcn = []; 
trajAct.ResultFcn = [];

%Cliente Garra
[gripAct, gripGoal] = rosactionclient('/gripper_controller/follow_joint_trajectory', 'control_msgs/FollowJointTrajectory', 'DataFormat', 'struct');
gripAct.FeedbackFcn = [];
gripAct.ResultFcn = [];
waitForServer(gripAct);

%clente depth.camera
pcSub = rossubscriber('/camera/depth/points', 'sensor_msgs/PointCloud2', 'DataFormat', 'struct');
pause(1);
%%
% Load the UR5e robot
UR5e = loadrobot("universalUR5e");

% Get default (home) configuration for ur5e
config = homeConfiguration(UR5e);

% Modify the joint positions
config(1).JointPosition = 0;
config(2).JointPosition = -pi/2;
config(3).JointPosition = 0;
config(4).JointPosition = 0;
config(5).JointPosition = -pi/2;
config(6).JointPosition = 0;

% === fixando juntas ===

% wrist_2_fixed
newJoint = rigidBodyJoint('wrist_2_fixed','fixed');
oldBody_wrist_2_link = getTransform(UR5e, config, 'wrist_2_link', 'wrist_1_link');
setFixedTransform(newJoint, oldBody_wrist_2_link);
replaceJoint(UR5e, 'wrist_2_link', newJoint);

% % wrist_3_fixed
% newJoint = rigidBodyJoint('wrist_3_fixed','fixed');
% oldBody_wrist_3_link = getBody(UR5e, 'wrist_3_link');
% setFixedTransform(newJoint, oldBody_wrist_3_link.Joint.JointToParentTransform);% Copia a transformação original da junta atual
% replaceJoint(UR5e, 'wrist_3_link', newJoint);

% === Alterando Limites ===

body = getBody(UR5e, 'shoulder_link');
body.Joint.PositionLimits = [-pi/2, pi/2];
replaceBody(UR5e, 'shoulder_link', body);

body = getBody(UR5e, 'upper_arm_link');
body.Joint.PositionLimits = [-pi, 0];
replaceBody(UR5e, 'upper_arm_link', body);

body = getBody(UR5e, 'forearm_link');
body.Joint.PositionLimits = [0, pi];
replaceBody(UR5e, 'forearm_link', body);

body = getBody(UR5e, 'wrist_1_link');
body.Joint.PositionLimits = [-pi/2, pi/2];
replaceBody(UR5e, 'wrist_1_link', body);
%%
pos_atual = [0,0,0,0,0];
pos_garra = [0.8 0 0.5];

config = cinematica_inversa_generica(pos_garra,UR5e,pos_atual);
mover_para(config, trajGoal, trajAct);
pos_atual = config;
pause(10);
[tipo_Objeto, pos_objeto] = detector_pt(pos_garra, pcSub);