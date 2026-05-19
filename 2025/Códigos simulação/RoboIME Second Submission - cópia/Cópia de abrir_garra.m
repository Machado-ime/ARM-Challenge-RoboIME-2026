function abrir_garra(abertura, gripGoal, gripAct)
    gripPos = abertura; % 0 --> aberto ; 1 --> fechado
    gripGoal = packGripGoal(gripPos, gripGoal);
    cancelAllGoals(gripAct);
    pause(0.1);
    sendGoal(gripAct, gripGoal);
end
