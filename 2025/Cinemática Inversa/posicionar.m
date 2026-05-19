% === Configuração do ROS ===
clc; close all;
% setenv('ROS_MASTER_URI','http://192.168.245.132:11311');
% setenv('ROS_IP','192.168.187.128');
rosshutdown;
rosinit('192.168.187.128', 11311);

%cliente Traj
[trajAct,trajGoal] = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','struct');
trajAct.FeedbackFcn = []; 
trajAct.ResultFcn = [];

%Cliente Garra
[gripAct, gripGoal] = rosactionclient('/gripper_controller/follow_joint_trajectory', 'control_msgs/FollowJointTrajectory', 'DataFormat', 'struct');
gripAct.FeedbackFcn = [];
gripAct.ResultFcn = [];
waitForServer(gripAct);
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
ang_atual = [ 0   0    0   0];
pos = [0.3 0 0.6];
config = cinematica_inversa_generica(pos,UR5e,ang_atual);
% ang_atual = config;

mensagem = zeros(1,6);
mensagem(1:4) = config(1:4);
mensagem(2) = mensagem(2) + pi/2;
mover_para(mensagem, trajGoal, trajAct)
pause(20);