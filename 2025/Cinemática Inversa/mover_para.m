function mover_para(ang, trajGoal, trajAct)
  
    % Define nomes das juntas na ordem esperada
    trajGoal.Trajectory.JointNames = {'elbow_joint','shoulder_lift_joint','shoulder_pan_joint',...
                                      'wrist_1_joint','wrist_2_joint','wrist_3_joint'};

    % Reorganiza os ângulos se necessário (de acordo com a ordem acima)
    config = ang([3 2 1 4 5 6]); % Ajusta a ordem para coincidir com JointNames

    % Cria ponto da trajetória
    trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint','DataFormat','struct');
    trajPts.TimeFromStart = rosduration(5, 'DataFormat', 'struct'); % Tempo total da trajetória
    trajPts.Positions = config;
    trajPts.Velocities = zeros(1, 6);
    trajPts.Accelerations = zeros(1, 6);
    trajPts.Effort = zeros(1, 6);

    % Adiciona tolerância para cada junta
    for i = 1:6
        tol = rosmessage('control_msgs/JointTolerance','DataFormat','struct');
        tol.Name = trajGoal.Trajectory.JointNames{i};
        tol.Position = 0;
        tol.Velocity = 0;
        tol.Acceleration = 0;
        trajGoal.GoalTolerance(i) = tol;
    end

    % Adiciona ponto à trajetória e envia
    trajGoal.Trajectory.Points = trajPts;
    sendGoal(trajAct, trajGoal);
    
end
