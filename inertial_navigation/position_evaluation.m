% anterior
salvar_nova_pos = false;
nova_comparacao = false;
if nova_comparacao == false
    load("POSICA0_teste.mat"); % load into a struct to avoid creating variables in workspace
    % atual
    POSICAO_teste_atual(:,1) = out.Y.signals.values(:,10); % x - N
    POSICAO_teste_atual(:,2) = out.Y.signals.values(:,11); % y - E
    POSICAO_teste_atual(:,3) = out.Y.signals.values(:,12); % z - D
    % erro entre atual e anterior
    erro1 = POSICAO_teste(:,1) - POSICAO_teste_atual(:,1);
    erro2 = POSICAO_teste(:,2) - POSICAO_teste_atual(:,2);
    erro3 = POSICAO_teste(:,3) - POSICAO_teste_atual(:,3);
    % mostrando erros
    hold off
    plot(erro1, 'r', 'DisplayName', 'Erro X');
    plot(erro2, 'g', 'DisplayName', 'Erro Y');
    plot(erro3, 'b', 'DisplayName', 'Erro Z');
    legend show
    hold off
    erro_entre_posicao_teste = mean(norm(erro1) + norm(erro2) + norm(erro3));
    fprintf("\n erro médio: %.4f \n", erro_entre_posicao_teste);
    clear erro1 erro2 erro3 erro_entre_posicao_teste;
    if salvar_nova_pos == true, save POSICA0_teste.mat POSICAO_teste_atual -mat; end
    clear salvar_nova_pos POSICAO_teste POSICAO_teste_atual nova_comparacao;
else
    POSICAO_teste(:,1) = out.Y.signals.values(:,10); % x - N
    POSICAO_teste(:,2) = out.Y.signals.values(:,11); % y - E
    POSICAO_teste(:,3) = out.Y.signals.values(:,12); % z - D
    save("POSICA0_teste.mat","POSICAO_teste","-mat")
    disp("File doesn't exist. Created POSICA0_teste.mat. Re-run the " + ...
    "simulation and then re-run the script in order to compare " + ...
    "        curves.")
    clear salvar_nova_pos POSICAO_teste nova_comparacao;
end