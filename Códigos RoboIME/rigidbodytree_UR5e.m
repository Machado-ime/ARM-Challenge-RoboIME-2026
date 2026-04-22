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
% === Cinemática Inversa ===
ang_atual = [ 0.3729   -1.5888    1.6233   -1.5708];
pos = [0.4,0.3,0.4];
config = cinematica_inversa_generica(pos,UR5e,ang_atual);
ang_atual = config;

% %%
% % === Avaliando ===
% showdetails(UR5e)
% 
% % Percorre todos os corpos e imprime apenas as juntas revolute
% for i = 1:numel(UR5e.Bodies)
%     body = UR5e.Bodies{i};
%     joint = body.Joint;
% 
%     if strcmp(joint.Type, 'revolute')
%         fprintf("Joint: %-25s | Axis: [%g %g %g] | Limits: [%g, %g] | Home: %.2f\n", ...
%             joint.Name, ...
%             joint.JointAxis, ...
%             joint.PositionLimits(1), joint.PositionLimits(2), ...
%             joint.HomePosition);
%     end
% end
