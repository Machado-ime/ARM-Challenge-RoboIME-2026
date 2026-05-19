function config = ajuste(config, ind, inc, pauseduration, jointSub, trajGoal, trajAct)  % rotações incrementais
    config(ind) = config(ind) + inc;
    jointStateMsg = receive(jointSub,100); 
    trajGoal = packTrajGoal(config,trajGoal);
    sendGoal(trajAct,trajGoal); 
    pause(pauseduration);
end