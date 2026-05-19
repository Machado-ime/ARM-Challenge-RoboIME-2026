function [config, pos_atual] = cinematica_inversa_generica(pos, chute, peso_pos, peso_ori, angulo)

% Load the UR5e robot
UR5e = loadrobot("universalUR5e");

% % Get default (home) configuration for ur5e
config = homeConfiguration(UR5e);

% Modify the joint positions
config(1).JointPosition = 0;
config(2).JointPosition = -pi/2;
config(3).JointPosition = 0;
config(4).JointPosition = 0;
config(5).JointPosition = -pi/2;
config(6).JointPosition = pi/2;

% === fixando juntas ===

% wrist_2_fixed
newJoint = rigidBodyJoint('wrist_2_fixed','fixed');
oldBody_wrist_2_link = getTransform(UR5e, config, 'wrist_2_link', 'wrist_1_link');
setFixedTransform(newJoint, oldBody_wrist_2_link);
replaceJoint(UR5e, 'wrist_2_link', newJoint);


% === Alterando Limites ===

% Alterando limite da shoulder_pan_joint
body = getBody(UR5e, 'upper_arm_link');
body.Joint.PositionLimits = [-pi, 0];
replaceBody(UR5e, 'upper_arm_link', body);

body = getBody(UR5e, 'forearm_link');
body.Joint.PositionLimits = [0, pi];
replaceBody(UR5e, 'forearm_link', body);

% body = getBody(UR5e, 'wrist_3_link');
% body.Joint.PositionLimits = [-pi, pi];
% replaceBody(UR5e, 'wrist_3_link', body);

% % Get default (home) configuration for ur5e
config = homeConfiguration(UR5e);

% Modify the joint positions
config(1).JointPosition = 0;
config(2).JointPosition = -pi/2;
config(3).JointPosition = 0;
config(4).JointPosition = 0;
config(5).JointPosition = 0;


% Cria o resolvedor GIK com restrições específicas
gik = generalizedInverseKinematics("RigidBodyTree", UR5e, "ConstraintInputs", {"position","orientation"});
gik.SolverParameters.MaxIterations = 500;
gik.SolverParameters.SolutionTolerance = 1e-4;
gik.SolverParameters.GradientTolerance = 1e-8;

% Cria a restrição de posição
posTarget = constraintPositionTarget("tool0");
posTarget.TargetPosition = pos;
posTarget.Weights = peso_pos;  % peso total para X Y Z

% Cria a restrição de orientação
oriTarget = constraintOrientationTarget("tool0");
oriTarget.TargetOrientation = eul2quat([angulo -pi 0]);oriTarget.Weights = peso_ori;  % zero se quiser ignorar a orientação

% Chute inicial
initialGuess = homeConfiguration(UR5e);
for i = 1:numel(chute)
    initialGuess(i).JointPosition = chute(i);
end

% Resolve a cinemática inversa
[configSoln, solnInfo] = gik(initialGuess, posTarget, oriTarget);

% Extrai resultado
config = [configSoln(1).JointPosition configSoln(2).JointPosition configSoln(3).JointPosition configSoln(4).JointPosition -pi/2 configSoln(5).JointPosition];

% Diagnóstico de Posição e Orientação
T = getTransform(UR5e, configSoln, "tool0");
pos_atual = tform2trvec(T);
quat_atual = tform2quat(T);

% Converte para Euler (Z-Y-X) em graus para facilitar a leitura humana
euler_atual = rad2deg(quat2eul(quat_atual)); 

fprintf("\n--- Diagnóstico Tool0 ---\n");
disp("Status do Solver: " + solnInfo.Status);
fprintf("Posição:    X = %.4f | Y = %.4f | Z = %.4f (m)\n", pos_atual(1), pos_atual(2), pos_atual(3));
fprintf("Orientação: Z = %.2f°   | Y = %.2f°   | X = %.2f° (Euler)\n", euler_atual(1), euler_atual(2), euler_atual(3));
disp("Configuração de Juntas (rad):");
disp(config); 

end




% function [config, pos_atual] = cinematica_inversa_generica(pos, chute, peso_pos, peso_ori)
% 
%     persistent UR5e;
%     persistent gik;
% 
%     config    = zeros(6, 1);
%     pos_atual = zeros(1, 3);
% 
%     if isempty(UR5e)
% 
%         UR5e = loadrobot("universalUR5e");
% 
%         % Define config auxiliar para o getTransform
%         configAux = homeConfiguration(UR5e);
%         configAux(1).JointPosition = 0;
%         configAux(2).JointPosition = -pi/2;
%         configAux(3).JointPosition = 0;
%         configAux(4).JointPosition = 0;
%         configAux(5).JointPosition = -pi/2;
%         configAux(6).JointPosition = pi/2;
% 
%         % Fixa wrist_2
%         newJoint = rigidBodyJoint('wrist_2_fixed', 'fixed');
%         oldBody  = getTransform(UR5e, configAux, 'wrist_2_link', 'wrist_1_link');
%         setFixedTransform(newJoint, oldBody);
%         replaceJoint(UR5e, 'wrist_2_link', newJoint);
% 
%         % Limites das juntas
%         body = getBody(UR5e, 'upper_arm_link');
%         body.Joint.PositionLimits = [-pi, 0];
%         replaceBody(UR5e, 'upper_arm_link', body);
% 
%         body = getBody(UR5e, 'forearm_link');
%         body.Joint.PositionLimits = [0, pi];
%         replaceBody(UR5e, 'forearm_link', body);
% 
%         % Solver
%         gik = generalizedInverseKinematics("RigidBodyTree", UR5e, "ConstraintInputs", {"position","orientation"});
%         gik.SolverParameters.MaxIterations     = 5000;
%         gik.SolverParameters.SolutionTolerance = 1e-4;
%         gik.SolverParameters.GradientTolerance = 1e-8;
% 
%     end
% 
%     % Chute inicial
%     initialGuess = homeConfiguration(UR5e);
%     for i = 1:5
%         initialGuess(i).JointPosition = chute(i);
%     end
% 
%     % Restrições
%     posTarget = constraintPositionTarget("tool0");
%     posTarget.TargetPosition = pos;
%     posTarget.Weights = peso_pos;
% 
%     oriTarget = constraintOrientationTarget("tool0");
%     oriTarget.TargetOrientation = eul2quat([0 pi 0]);
%     oriTarget.Weights = peso_ori;
% 
%     % Resolve IK
%     [configSoln, solnInfo] = gik(initialGuess, posTarget, oriTarget);
% 
%     % Extrai resultado — 6 juntas com wrist_2 fixada em -pi/2
%     config = [configSoln(1).JointPosition
%               configSoln(2).JointPosition
%               configSoln(3).JointPosition
%               configSoln(4).JointPosition
%               -pi/2
%               configSoln(5).JointPosition];
% 
%     % Diagnóstico
%     disp(solnInfo.Status);
%     T = getTransform(UR5e, configSoln, "tool0");
%     pos_atual = tform2trvec(T);
%     fprintf("X = %.4f m | Y = %.4f m | Z = %.4f m\n", pos_atual(1), pos_atual(2), pos_atual(3));
% 
% end