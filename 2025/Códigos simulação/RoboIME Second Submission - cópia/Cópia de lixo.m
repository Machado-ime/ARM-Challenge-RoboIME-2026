function lixo(tipo,trajGoal, trajAct,gripGoal, gripAct)

% tipos
% 1 - Can
% 2 - Bottle
% 3 - Spam
% 4 - Markers
% 5 - Green and Purple cubes
% 6 - Blue and Red cubes

ang_lixeira_verde = [2.3199 - pi/8 , 0,pi/2,-pi/2,0,0];
ang_lixeira_azul = [-2.3199 - pi/12, 0,pi/2,-pi/2,0,0];

% posicao_inicial(trajGoal, trajAct, gripGoal, gripAct)
pause(5);
if tipo == 1 || tipo == 3 || tipo == 5
mover_para(ang_lixeira_verde, trajGoal, trajAct);
elseif tipo == 2 || tipo == 4 || tipo == 6
mover_para(ang_lixeira_azul, trajGoal, trajAct)
else
disp('objeto não tipificado')
end
pause(15);
abrir_garra(0, gripGoal, gripAct)
end