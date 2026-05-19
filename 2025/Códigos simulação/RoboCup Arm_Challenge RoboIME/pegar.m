function pegar(gripGoal, gripAct)
abertura = 0.0;

while true
jointSub = rossubscriber("/gripper_controller/state",'DataFormat','struct');
jointStateMsg = receive(jointSub,10);


    % Verifica se o esforço não está vazio
    if ~isempty(jointStateMsg.Actual.Effort)
        disp('Esforço detectado. Encerrando o loop.');
        break;
    end

    % Incrementa a abertura
    abertura = abertura + 0.01;

    % Chama a função para abrir a garra
    abrir_garra(abertura, gripGoal, gripAct);
end
end