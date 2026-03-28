function config = cinematica_inversa_generica(pos,chute)

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
config(6).JointPosition = 0;

% % === Alterando Limites ===

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

% Cria o resolvedor GIK com restrições específicas
gik = generalizedInverseKinematics("RigidBodyTree", UR5e, "ConstraintInputs", {"position","orientation"});

% Cria a restrição de posição
posTarget = constraintPositionTarget("tool0");
posTarget.TargetPosition = pos;
posTarget.Weights = 1;  % peso total para X Y Z

% Cria a restrição de orientação
oriTarget = constraintOrientationTarget("tool0");
oriTarget.TargetOrientation = eul2quat([0 pi 0]);  % rot: ZYX
oriTarget.Weights = 1;  % zero se quiser ignorar a orientação

% Chute inicial
initialGuess = homeConfiguration(UR5e);
for i = 1:numel(chute)
    initialGuess(i).JointPosition = chute(i);
end

% Resolve a cinemática inversa
[configSoln, solnInfo] = gik(initialGuess, posTarget, oriTarget);

% Extrai resultado
config = [configSoln(1).JointPosition configSoln(2).JointPosition configSoln(3).JointPosition configSoln(4).JointPosition configSoln(5).JointPosition configSoln(6).JointPosition];

% Visualiza
show(UR5e, configSoln);

% Diagnóstico
disp(config);
disp(solnInfo.Status);
T = getTransform(UR5e, configSoln, "tool0");
pos_final = tform2trvec(T);
fprintf("X = %.4f m | Y = %.4f m | Z = %.4f m\n", pos_final(1), pos_final(2), pos_final(3));

end