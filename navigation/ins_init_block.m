%% INICIANDO SENSORES
ins_init_sensors(); % load variables para o modelo e os dados dos sensores
%% ========== DBN / Inertial Navigation Initialization ==========
DBN_params = struct();
DBN_params.initialized = false;
assignin('base', 'DBN_params', DBN_params);
% Placeholder. O DBN_params real será criado por init_DBN,
% chamado dentro de ins_initial_state_xplane.m após reposicionar o X-Plane.
% Essa struct também guarda os parametros de inicialização para os EKFs
% Os EKFs também são inicializados em ins_initial_state_xplane.m 
%% INFORMANDO
fprintf("\nins_init_block: DBN_params carregado no path.\n" + ...
    "A inicialização real ocorrerá via:\ninit_DBN, init_EKF_DI, " + ...
    "init_EKF_INDI após posicionamento inicial no X-Plane.\n");