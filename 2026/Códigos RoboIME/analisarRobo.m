%% 1. Carrega o robô
UR5e = loadrobot("universalUR5e");

%% 2. Config com 6 juntas (antes da fixação)
config = homeConfiguration(UR5e);
config(1).JointPosition = 0;        % shoulder_pan
config(2).JointPosition = -pi/2;    % shoulder_lift
config(3).JointPosition = 0;        % elbow
config(4).JointPosition = 0;        % wrist_1
config(5).JointPosition = -pi/2;    % wrist_2  <-- será fixada
config(6).JointPosition = pi/2;     % wrist_3

% wrist_2_fixed
newJoint = rigidBodyJoint('wrist_2_fixed','fixed');
oldBody_wrist_2_link = getTransform(UR5e, config, 'wrist_2_link', 'wrist_1_link');
setFixedTransform(newJoint, oldBody_wrist_2_link);
replaceJoint(UR5e, 'wrist_2_link', newJoint);


%% === Alterando Limites ===

% Alterando limite da shoulder_pan_joint
body = getBody(UR5e, 'upper_arm_link');
body.Joint.PositionLimits = [-pi, 0];
replaceBody(UR5e, 'upper_arm_link', body);

body = getBody(UR5e, 'forearm_link');
body.Joint.PositionLimits = [0, pi];
replaceBody(UR5e, 'forearm_link', body);

body = getBody(UR5e, 'wrist_3_link');
body.Joint.PositionLimits = [-pi/2, pi/2];
replaceBody(UR5e, 'wrist_3_link', body);

%% 4. Nova config com 5 juntas
configNew = homeConfiguration(UR5e);
configNew(1).JointPosition = 0;        % shoulder_pan
configNew(2).JointPosition = -pi/2;    % shoulder_lift
configNew(3).JointPosition = 0;        % elbow
configNew(4).JointPosition = 0;        % wrist_1
configNew(5).JointPosition = pi/2;     % wrist_3

%% 5. Tabela de Limites das Juntas Não-Fixas
jointNames = {};
lowerLim   = [];
upperLim   = [];
types      = {};
count      = 0;

for i = 1:UR5e.NumBodies
    jnt = UR5e.Bodies{i}.Joint;
    if ~strcmp(jnt.Type, 'fixed')
        count = count + 1;
        jointNames{count} = jnt.Name;
        lowerLim(count)   = jnt.PositionLimits(1);
        upperLim(count)   = jnt.PositionLimits(2);
        types{count}      = jnt.Type;
    end
end

TabelaLimites = table(jointNames', types', ...
    lowerLim', upperLim', ...
    rad2deg(lowerLim)', rad2deg(upperLim)', ...
    'VariableNames', {'Junta', 'Tipo', 'Min_Rad', 'Max_Rad', 'Min_Deg', 'Max_Deg'});

fprintf('\n=== LIMITES DAS JUNTAS (pós-fixação) ===\n');
disp(TabelaLimites);

%% 6. Diagnóstico completo
fprintf('\n=== DETALHES DA ESTRUTURA ===\n');
showdetails(UR5e);

%% 7. Visualização
figure('Name', 'UR5e com wrist_2 fixada', 'Color', [1 1 1]);
show(UR5e, configNew, 'Frames', 'on', 'Visuals', 'on');
title('UR5e: wrist\_2\_link fixada em -90°');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal; view(3);