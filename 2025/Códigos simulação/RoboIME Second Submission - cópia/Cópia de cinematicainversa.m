function config = cinematicainversa(rot, UR5e, ik, pos, chute)
    % Pesos para o resolvedor de cinemática inversa
    ikWeights = [0.25 0.25 0.25 0.1 0.1 0.1]; 
    
    % Configuração inicial guess
    initialIKGuess = homeConfiguration(UR5e);
    initialIKGuess(1).JointPosition = chute(1);
    initialIKGuess(2).JointPosition = chute(2);
    initialIKGuess(3),JointPosition = chute(3)
    initialIKGuess(4).JointPosition = chute(4);
    initialIKGuess(2).JointPosition = chute(2);
    initialIKGuess(5).JointPosition = chute(5);
    initialIKGuess(6).JointPosition = chute(6); 
    % Ajustes nas transformações das juntas
    % Orientação desejada do gripper [Z Y X] em radianos
    gripperRotation = rot; %padrão [-pi/2 -pi 0] 
    tform = eul2tform(gripperRotation);
    tform(1:3,4) = pos'; % Posição desejada
    
    % Resolve cinemática inversa
    [configSoln, solnInfo] = ik('tool0', tform, ikWeights, initialIKGuess);
    
    % Organiza as juntas na ordem [3 2 1 4 5 6]
    config = [configSoln(3).JointPosition configSoln(2).JointPosition configSoln(1).JointPosition...
              configSoln(4).JointPosition configSoln(5).JointPosition configSoln(6).JointPosition];
    
    % Corrige os ângulos para o intervalo [-π, π]
    for i = 1:6
        while config(i) > pi
            config(i) = config(i) - 2*pi;
        end
        while config(i) < -pi
            config(i) = config(i) + 2*pi;
        end
    end
config
end