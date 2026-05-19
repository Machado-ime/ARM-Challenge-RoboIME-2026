function T = cinematica_direta_ur5e(q)
%
% q -> vetor de ângulos das juntas (6x1) em radianos
%
% T -> matriz de transformação homogênea 4x4 (pose do efetuador)

    robot = loadrobot("universalUR5e", DataFormat="column");

    T = getTransform(robot, q, "tool0");

end