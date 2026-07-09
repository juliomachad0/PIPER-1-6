%% ========== Limpar conexao anterior ==========
fprintf('\n---------------------ins_init_xplane:----------------------')
fprintf('\n------------------------- X-plane -------------------------')
global GlobalSocket;

import XPlaneConnect.*; % Importa a biblioteca para o escopo atual

if ~isempty(GlobalSocket)
    try
        closeUDP(GlobalSocket);
        fprintf('\nins_init_xplane: Conexao anterior fechada.\n');
    catch ME
        fprintf(['\nins_init_xplane: Erro ao fechar a conexão: ' ...
            '\nError: %s\n'], ME.message);
    end
    GlobalSocket = []; % Limpa após fechar
end

% AGORA CRIAMOS OU REINICIAMOS A CONEXÃO
fprintf('\nins_init_xplane: Iniciando/Testando conexão UDP...');

try
    % 1. Inicializa o soquete antes de testar 
    % (Ajuste o IP/Porta se necessário)
    % Se a função openUDP() não receber argumentos, ela usa os padrões 
    % (127.0.0.1, 49009)
    GlobalSocket = openUDP(); 

    % 2. Tenta ler a altitude AGL para validar o teste
    drefs = {'sim/flightmodel/position/y_agl'};
    result = double(getDREFs(drefs, GlobalSocket));

    ALG_ins_init_xplane_test_connection = result(1);
    fprintf('\nins_init_xplane: Readed AGL: %.2f m\n', ...
        ALG_ins_init_xplane_test_connection);
    fprintf('\nins_init_xplane: Conexão bem sucedida, continuando...\n');
    clear ALG_ins_init_xplane_test_connection

catch ME
    % Limpa o soquete mal sucedido para não travar a próxima execução
    GlobalSocket = []; 

    fprintf(['\nins_init_xplane: Erro na leitura de AGL, ' ...
        'falha no teste de conexão.\nError: %s\n' ...
        '\nins_init_xplane: Encerrando simulação.\n'], ME.message);
    error('ins_init_xplane:SimulacaoEncerrada', ...
        'ins_init_xplane: Simulação encerrada por falta de conexão.');
end

%% ========== Parametros do X-Plane ==========
Ts_xplane = 0.05;  % Sample time: 20 Hz (adjust if necessary)
assignin('base', 'Ts_xplane', Ts_xplane);
fprintf('\n-----------------------------------------------------------\n')
