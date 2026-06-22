function scr_aux_wp(WPs_user, R_accept, t_stop, h_eq)
%% scr_aux_wp - Rotina auxiliar de preparacao e execucao da simulacao
% Chamada por scr_waypoints.m. Centraliza toda a logica que gui_waypoints.m
% executa em runSimulation: rodar inicializar.m, importar parametros,
% montar WPs (com WP1 fixo na origem e interpolacao de altitude),
% atribuir variaveis no base workspace, executar o Simulink e plotar.
%
% Argumentos:
%   WPs_user : matriz Nx4 [N, E, h, vel] com os waypoints WP2, WP3, ...
%              (WP1 na origem e inserido automaticamente).
%   R_accept : raio de aceitacao dos waypoints (m).
%   t_stop   : tempo de parada da simulacao (s).
%   h_eq     : altitude de equilibrio / altitude de WP1 (m).

    %% ========== Inicializacao ==========
    % Descobrir raiz do projeto (pai de guiagem/)
    rootDir = fileparts(fileparts(mfilename('fullpath')));

    % Rodar inicializar.m no base workspace
    oldDir = pwd;
    cd(rootDir);
    evalin('base', 'inicializar');
    cd(oldDir);

    % Importar variaveis do base workspace (carregadas por inicializar.m)
    par_aero  = evalin('base', 'par_aero');
    par_prop  = evalin('base', 'par_prop');
    par_gen   = evalin('base', 'par_gen');
    Xe        = evalin('base', 'Xe');
    Ue        = evalin('base', 'Ue');
    Xe_init   = evalin('base', 'Xe_init');
    C_alt     = evalin('base', 'C_alt');
    C_theta   = evalin('base', 'C_theta');
    C_vel     = evalin('base', 'C_vel');
    C_phi     = evalin('base', 'C_phi');
    Kq        = evalin('base', 'Kq');
    Kp        = evalin('base', 'Kp');
    Kr        = evalin('base', 'Kr');
    INPUTS    = evalin('base', 'INPUTS');
    TrimInput = evalin('base', 'TrimInput');
    Kp_sas    = evalin('base', 'Kp_sas');

    % Valor padrao de velocidade para WP1 (espelha gui_waypoints.m)
    v_wp1 = 15;

    %% ========== Montar matriz de waypoints ==========
    % WP1 fixo na origem
    wp_data = [0, 0, h_eq, v_wp1];

    % Adicionar WPs do usuario (WP2 em diante)
    if ~isempty(WPs_user)
        wp_data = [wp_data; WPs_user];
    end

    if size(wp_data, 1) < 2
        error('scr_aux_wp:WPinsuficientes', ...
              'Defina pelo menos 1 waypoint em WPs (alem do WP1 automatico).');
    end

    %% ========== Interpolar waypoints com gradiente de altitude ==========
    % Espelha a logica de gui_waypoints.m para evitar variacoes bruscas.
    max_dalt = 30;  % maxima variacao de altitude por trecho (m)
    wp_interp = wp_data(1, :);
    for i = 2:size(wp_data, 1)
        dalt = abs(wp_data(i,3) - wp_data(i-1,3));
        if dalt > max_dalt
            n_sub = ceil(dalt / max_dalt);
            for k = 1:n_sub-1
                frac = k / n_sub;
                wp_mid = wp_data(i-1,:) + frac * (wp_data(i,:) - wp_data(i-1,:));
                wp_interp(end+1, :) = wp_mid;
            end
        end
        wp_interp(end+1, :) = wp_data(i, :);
    end

    %% ========== Atribuir parametros no base workspace ==========
    % Parametros do modelo
    assignin('base', 'par_aero', par_aero);
    assignin('base', 'par_prop', par_prop);
    assignin('base', 'par_gen',  par_gen);
    assignin('base', 'Xe',       Xe);
    assignin('base', 'Ue',       Ue);
    assignin('base', 'Xe_init',  Xe_init);

    % Ganhos PID
    assignin('base', 'C_alt',   C_alt);
    assignin('base', 'C_theta', C_theta);
    assignin('base', 'C_vel',   C_vel);
    assignin('base', 'C_phi',   C_phi);
    assignin('base', 'Kq',      Kq);
    assignin('base', 'Kp',      Kp);
    assignin('base', 'Kr',      Kr);

    % Compatibilidade
    assignin('base', 'INPUTS',    INPUTS);
    assignin('base', 'TrimInput', TrimInput);
    assignin('base', 'Kp_sas',   Kp_sas);

    % Waypoints (interpolados) e raio de aceitacao
    assignin('base', 'WPs',      wp_interp);
    assignin('base', 'R_accept', R_accept);

    fprintf('scr_aux_wp: %d WPs originais, %d apos interpolacao (max_dalt=%d m).\n', ...
        size(wp_data, 1), size(wp_interp, 1), max_dalt);
    fprintf('scr_aux_wp: Simulando por %.0f s com R_accept=%.0f m.\n', ...
        t_stop, R_accept);

    %% ========== Executar Simulink ==========
    modelName  = 'NL_guidance';
    modelPath  = fullfile(rootDir, 'guiagem', [modelName '.slx']);
    load_system(modelPath);
    set_param(modelName, 'StopTime', num2str(t_stop));

    out = sim(modelName);
    assignin('base', 'out', out);

    % Salvar tambem a versao NAO interpolada para referencia nos plots
    assignin('base', 'WPs',      wp_data);
    assignin('base', 'R_accept', R_accept);

    fprintf('scr_aux_wp: Simulacao concluida.\n');

    %% ========== Plotar resultados ==========
    close all;
    evalin('base', 'plot3d_voo');
    evalin('base', 'plot3d_voo_xplane');
end
