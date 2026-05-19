function config = cinematica_inversa_generica(pos,UR5e,chute)

% Cria o resolvedor GIK com restrições específicas
gik = generalizedInverseKinematics("RigidBodyTree", UR5e, ...
    "ConstraintInputs", {"position","orientation"});

% Cria a restrição de posição
posTarget = constraintPositionTarget("tool0");
posTarget.TargetPosition = pos;
posTarget.Weights = 1;  % peso total para X Y Z

% Cria a restrição de orientação
oriTarget = constraintOrientationTarget("tool0");
oriTarget.TargetOrientation = eul2quat([0 pi 0]);  % rot: ZYX
oriTarget.Weights = 1;  % zero se quiser ignorar a orientação

% maxTentativas = 20;
% tentativa = 0;
% sucesso = false;
% 
% while tentativa < maxTentativas && ~sucesso
%     tentativa = tentativa + 1;
% 
% % Chute inicial
% initialGuess = homeConfiguration(UR5e);
% for i = 1:numel(chute)
%     initialGuess(i).JointPosition = chute(i);
% end
% 
% % Resolve a cinemática inversa
% [configSoln, solnInfo] = gik(initialGuess, posTarget, oriTarget);
% 
% % Extrai resultado
% config = [configSoln(1).JointPosition configSoln(2).JointPosition ...
%           configSoln(3).JointPosition configSoln(4).JointPosition];
% 
% chute = config;
% chute = chute +  0.1*randn(size(chute));% Adiciona pequena variação aleatória ao chute
% 
% disp(config);
% 
%     if strcmp(solnInfo.Status, 'success')
%         sucesso = true;
%     end
% end

% Chute inicial
initialGuess = homeConfiguration(UR5e);
for i = 1:numel(chute)
    initialGuess(i).JointPosition = chute(i);
end

% Resolve a cinemática inversa
[configSoln, solnInfo] = gik(initialGuess, posTarget, oriTarget);

% Extrai resultado
config = [configSoln(1).JointPosition configSoln(2).JointPosition ...
          configSoln(3).JointPosition configSoln(4).JointPosition];

% Visualiza
show(UR5e, configSoln);

% Diagnóstico
disp(config);
disp(solnInfo.Status);
T = getTransform(UR5e, configSoln, "tool0");
pos_final = tform2trvec(T);
fprintf("X = %.4f m | Y = %.4f m | Z = %.4f m\n", pos_final(1), pos_final(2), pos_final(3));
end
