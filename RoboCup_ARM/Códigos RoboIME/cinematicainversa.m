function config = cinematicainversa(pos,UR5e,chute)

%cinematica
ik = inverseKinematics("RigidBodyTree", UR5e);

%Rotação do ponto final
rot = [0 pi 0];

% Pesos para o resolvedor de cinemática inversa
    ikWeights = [0. 0 0 1 1 1]; 
    
    % Configuração inicial guess
    initialIKGuess = homeConfiguration(UR5e);
    initialIKGuess(1).JointPosition = chute(1);
    initialIKGuess(2).JointPosition = chute(2);
    initialIKGuess(3).JointPosition = chute(3);
    initialIKGuess(4).JointPosition = chute(4);

    % Ajustes nas transformações das juntas
    gripperRotation = rot; %padrão [-pi/2 -pi 0] 
    tform = eul2tform(gripperRotation);
    tform(1:3,4) = pos'; % Posição desejada
    
    % Resolve cinemática inversa
    [configSoln, solnInfo] = ik('tool0', tform, ikWeights, initialIKGuess);
    
    % Organiza as juntas na ordem [3 2 1 4 5 6]
     show(UR5e,configSoln);

    config = [configSoln(1).JointPosition configSoln(2).JointPosition configSoln(3).JointPosition...
              configSoln(4).JointPosition];
    disp(config);
    disp(solnInfo.Status);
    
    T = getTransform(UR5e, configSoln, 'tool0');
pos_final = tform2trvec(T);
disp("Coordenadas do corpo 'tool0':");
fprintf("X = %.4f m\nY = %.4f m\nZ = %.4f m\n", pos_final(1), pos_final(2), pos_final(3));

end