function config_final = rotelem(config, k, ord, jointSub, trajGoal, trajAct)  % realiza as rotações corrigidas uma por vez até o ponto
ind_theta = [3 2 1 4 5 6] % ind_theta(i) == índice que o thetai precisa ser devolvido;
config_l = [0 0 0 0 0 0];
config_l(ind_theta(ord(1))) = k(1)*config(ind_theta(ord(1)))  % cada valor de k é um coeficiente de ajuste das rotações
    for i = 1:5
        jointStateMsg = receive(jointSub,100); % recebe a configuração atual do robô
        trajGoal = packTrajGoal(config_l,trajGoal);
        sendGoal(trajAct,trajGoal); 
        pause(3);
        config_l(ind_theta(ord(i+1))) = k(i+1)*config(ind_theta(ord(i+1)));
    end
config_final = config_l
end