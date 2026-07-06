%% ========== Paths do XPlaneConnect ==========
addpath(fullfile(rootDir, 'xplane', 'Xplane_Interface', 'interface'));
addpath(fullfile(rootDir, 'xplane', 'XPlaneConnect-master', 'MATLAB'));


%% ========== Limpar conexao anterior ==========
disp("-----------------------------------------------------------")
disp("------------------------- X-plane -------------------------")
global GlobalSocket;
if ~isempty(GlobalSocket)
    try
        import XPlaneConnect.*;
        closeUDP(GlobalSocket);
        disp('inicializar_xplane: Conexao anterior fechada.');
    catch ME
        disp(['Erro ao fechar a conexão: ', ME.message]);
    end
else
    disp('Inicialização X-plane: Nenhuma conexão anterior encontrada.');
end
GlobalSocket = [];

%% ========== Parametros do X-Plane ==========
Ts_xplane = 0.05;  % Sample time: 20 Hz (ajustar se necessario)
disp("-----------------------------------------------------------")