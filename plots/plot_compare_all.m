%% plot_compare_navigation_all.m
% Comparação configurável entre:
%   Modelo Matemático
%   XPlane referência
%   DBN sem modelagem de erro
%   DBN com modelagem de erro
%   EKF Direto sem modelagem de erro
%   EKF Direto com modelagem de erro
%   EKF Indireto sem modelagem de erro
%   EKF Indireto com modelagem de erro
%
% Uso:
%   Ajuste as flags abaixo e rode após a simulação:
%       >> plot_compare_navigation_all
%
% Convenções esperadas:
%   XPlaneSimulationData:
%       col 1  -> VT [m/s]
%       col 4  -> altitude [m]
%       col 9  -> Norte [m]
%       col 10 -> Leste [m]
%
%   DBN/EKF *_data:
%       col 1:3   -> Euler [phi theta psi] [rad]
%       col 4:6   -> Velocidade NED [vN vE vD] [m/s]
%       col 7:9   -> Posição NED [N E D] [m]
%       col 10:12 -> Aceleração NED [aN aE aD] [m/s²]
%       col 13:16 -> Quaternion, se existir
%
%   Modelo out.Y:
%       col 10 -> Norte [m]
%       col 11 -> Leste [m]
%       col 12 -> altitude [m]
%       col 1:3 -> velocidades no corpo, usadas para VT_model se existirem

fprintf('\n========== COMPARAÇÃO CONFIGURÁVEL DE NAVEGAÇÃO ==========' );
fprintf('\n');

%% ===================== ESCOLHA DOS MODELOS =====================
use_modelo        = false;
use_xplane_ref    = true;
use_dbn           = true;
use_dbn_em        = true;
use_ekf_di        = false;
use_ekf_di_em     = false;
use_ekf_indi      = false;
use_ekf_indi_em   = false;

%% ===================== CONFIGURAÇÕES VISUAIS ====================
colors.modelo      = [0.000 0.250 1.000];
colors.xplane      = [1.000 0.000 0.000];
colors.dbn         = [0.000 0.650 0.000];
colors.dbn_em      = [0.000 0.700 0.700];
colors.ekf_di      = [0.850 0.325 0.098];
colors.ekf_di_em   = [0.494 0.184 0.556];
colors.ekf_indi    = [0.929 0.694 0.125];
colors.ekf_indi_em = [0.250 0.250 0.250];

lineWidth = 1.4;

%% ===================== CARREGAR SÉRIES ==========================
series = struct('name', {}, 'key', {}, 't', {}, 'N', {}, 'E', {}, 'alt', {}, ...
                'vel', {}, 'VT', {}, 'acc', {}, 'euler', {}, 'color', {});

% Modelo Matemático
if use_modelo
    try
        data = out.Y.signals.values;
        t = out.Y.time;
        s.name = 'Modelo';
        s.key = 'modelo';
        s.t = t;
        s.N = data(:,10);
        s.E = data(:,11);
        s.alt = data(:,12);
        s.vel = [];
        s.acc = [];
        s.euler = [];
        if size(data,2) >= 3
            s.VT = sqrt(data(:,1).^2 + data(:,2).^2 + data(:,3).^2);
        else
            s.VT = [];
        end
        s.color = colors.modelo;
        series(end+1) = s; %#ok<SAGROW>
    catch
        warning('Dados do Modelo out.Y não encontrados.');
    end
end

% XPlane referência
if use_xplane_ref
    try
        data = out.XplaneSimulationData.signals.values;
        t = out.XplaneSimulationData.time;
        s.name = 'XPlane';
        s.key = 'xplane';
        s.t = t;
        s.N = data(:,9);
        s.E = data(:,10);
        s.alt = data(:,4);
        s.vel = [];
        s.VT = data(:,1);
        s.acc = [];
        s.euler = [];
        s.color = colors.xplane;
        series(end+1) = s; %#ok<SAGROW>
    catch
        warning('Dados out.XplaneSimulationData não encontrados.');
    end
end

% DBN sem modelagem de erro: aceita DBN_Data ou BDN_data para compatibilidade
if use_dbn
    s = load_nav_solution({'DBN_Data','BDN_data'}, 'DBN', 'dbn', colors.dbn);
    if ~isempty(s.name), series(end+1) = s; end %#ok<SAGROW>
end

% DBN com modelagem de erro
if use_dbn_em
    s = load_nav_solution({'DBN_data_with_error'}, 'DBN EM', 'dbn_em', colors.dbn_em);
    if ~isempty(s.name), series(end+1) = s; end %#ok<SAGROW>
end

% EKF Direto sem modelagem de erro
if use_ekf_di
    s = load_nav_solution({'EKF_DI'}, 'EKF DI', 'ekf_di', colors.ekf_di);
    if ~isempty(s.name), series(end+1) = s; end %#ok<SAGROW>
end

% EKF Direto com modelagem de erro
if use_ekf_di_em
    s = load_nav_solution({'EKF_DI_EM'}, 'EKF DI EM', 'ekf_di_em', colors.ekf_di_em);
    if ~isempty(s.name), series(end+1) = s; end %#ok<SAGROW>
end

% EKF Indireto sem modelagem de erro
if use_ekf_indi
    s = load_nav_solution({'EKF_INDI'}, 'EKF INDI', 'ekf_indi', colors.ekf_indi);
    if ~isempty(s.name), series(end+1) = s; end %#ok<SAGROW>
end

% EKF Indireto com modelagem de erro
if use_ekf_indi_em
    s = load_nav_solution({'EKF_INDI_EM'}, 'EKF INDI EM', 'ekf_indi_em', colors.ekf_indi_em);
    if ~isempty(s.name), series(end+1) = s; end %#ok<SAGROW>
end

if isempty(series)
    error('Nenhuma série foi carregada. Verifique flags e nomes em out.');
end

%% ===================== REFERÊNCIA PARA ERROS =====================
idx_ref = find(strcmp({series.key}, 'xplane'), 1);
if isempty(idx_ref)
    warning('XPlane não está ativo/disponível. Subplots de erro não serão gerados contra XPlane.');
end

%% ===================== FIGURA PRINCIPAL ==========================
figure('Name','Comparação Navegação - Todos os Modelos', ...
       'Position',[80 40 1650 950]);

%% 1 - Trajetória 3D
subplot(3,2,1)
hold on
for k = 1:numel(series)
    plot3(series(k).E, series(k).N, series(k).alt, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end
plot_waypoints_3d();
grid on
axis equal
xlabel('Leste [m]')
ylabel('Norte [m]')
zlabel('Altitude [m]')
title('Trajetória 3D')
legend(build_legend(series, true), 'Location','best')
view(30,25)
hold off

%% 2 - Vista superior
subplot(3,2,2)
hold on
for k = 1:numel(series)
    plot(series(k).E, series(k).N, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end
plot_waypoints_2d();
grid on
axis equal
xlabel('Leste [m]')
ylabel('Norte [m]')
title('Vista Superior')
legend(build_legend(series, true), 'Location','best')
hold off

%% 3 - Altitude
subplot(3,2,3)
hold on
for k = 1:numel(series)
    if ~isempty(series(k).t)
        plot(series(k).t, series(k).alt, ...
            'Color', series(k).color, 'LineWidth', lineWidth);
    end
end
plot_altitude_waypoints();
grid on
xlabel('Tempo [s]')
ylabel('Altitude [m]')
title('Altitude')
legend(build_legend(series, false), 'Location','best')
hold off

%% 4 - Velocidade escalar
subplot(3,2,4)
hold on
for k = 1:numel(series)
    if ~isempty(series(k).VT) && ~isempty(series(k).t)
        plot(series(k).t, series(k).VT, ...
            'Color', series(k).color, 'LineWidth', lineWidth);
    end
end
grid on
xlabel('Tempo [s]')
ylabel('Velocidade [m/s]')
title('Velocidade Escalar')
legend(build_legend_with_vt(series), 'Location','best')
hold off

%% 5 - Erro horizontal contra XPlane
subplot(3,2,5)
hold on
if ~isempty(idx_ref)
    ref = series(idx_ref);
    for k = 1:numel(series)
        if k == idx_ref
            continue;
        end
        [t_common, N_ref_i, N_i] = align_by_time(ref.t, ref.N, series(k).t, series(k).N);
        [~,        E_ref_i, E_i] = align_by_time(ref.t, ref.E, series(k).t, series(k).E);
        if ~isempty(t_common)
            erro_h = sqrt((N_ref_i - N_i).^2 + (E_ref_i - E_i).^2);
            plot(t_common, erro_h, 'Color', series(k).color, 'LineWidth', lineWidth);
        end
    end
    grid on
    xlabel('Tempo [s]')
    ylabel('Erro horizontal [m]')
    title('Erro Horizontal vs XPlane')
    legend(build_error_legend(series, idx_ref), 'Location','best')
else
    text(0.1,0.5,'XPlane indisponível para erro horizontal','Units','normalized')
    axis off
end
hold off

%% 6 - Erro de altitude contra XPlane
subplot(3,2,6)
hold on
if ~isempty(idx_ref)
    ref = series(idx_ref);
    for k = 1:numel(series)
        if k == idx_ref
            continue;
        end
        [t_common, alt_ref_i, alt_i] = align_by_time(ref.t, ref.alt, series(k).t, series(k).alt);
        if ~isempty(t_common)
            erro_alt = alt_ref_i - alt_i;
            plot(t_common, erro_alt, 'Color', series(k).color, 'LineWidth', lineWidth);
        end
    end
    grid on
    xlabel('Tempo [s]')
    ylabel('Erro de altitude [m]')
    title('Erro de Altitude vs XPlane')
    legend(build_error_legend(series, idx_ref), 'Location','best')
else
    text(0.1,0.5,'XPlane indisponível para erro de altitude','Units','normalized')
    axis off
end
hold off

sgtitle('Comparação Modelo Matemático x XPlane x DBN x EKFs')

%% ===================== ESTATÍSTICAS ===============================
fprintf('\n--- Séries carregadas ---\n');
for k = 1:numel(series)
    fprintf('  %-12s | N0=%8.2f E0=%8.2f Alt0=%8.2f | Nf=%8.2f Ef=%8.2f Altf=%8.2f\n', ...
        series(k).name, series(k).N(1), series(k).E(1), series(k).alt(1), ...
        series(k).N(end), series(k).E(end), series(k).alt(end));
end

fprintf('\n========== FIM DA COMPARAÇÃO CONFIGURÁVEL ==========' );
fprintf('\n');

%% ========================================================================
% FUNÇÕES LOCAIS
%% ========================================================================

function s = load_nav_solution(fieldNames, displayName, keyName, colorValue)
    s = struct('name', '', 'key', '', 't', [], 'N', [], 'E', [], 'alt', [], ...
               'vel', [], 'VT', [], 'acc', [], 'euler', [], 'color', []);

    found = false;
    for ii = 1:numel(fieldNames)
        fname = fieldNames{ii};
        try
            sig = evalin('base', sprintf('out.%s', fname));
            found = true;
            break;
        catch
        end
    end

    if ~found
        warning('Dados %s não encontrados em out.', displayName);
        return;
    end

    data = sig.signals.values;
    t = sig.time;

    if size(data,2) < 9
        warning('Dados %s encontrados, mas possuem menos de 9 colunas.', displayName);
        return;
    end

    euler = [];
    vel = [];
    acc = [];
    VT = [];

    if size(data,2) >= 3
        euler = data(:,1:3);
    end

    if size(data,2) >= 6
        vel = data(:,4:6);
        VT = sqrt(vel(:,1).^2 + vel(:,2).^2 + vel(:,3).^2);
    end

    pos = data(:,7:9);
    N = pos(:,1);
    E = pos(:,2);
    D = pos(:,3);

    if exist('DBN_params','var') && isfield(DBN_params,'altitude0') && isfield(DBN_params,'pos0_ned')
        altitude0 = DBN_params.altitude0;
        D0 = DBN_params.pos0_ned(3);
        alt = altitude0 - (D - D0);
    else
        alt = -D;
    end

    if size(data,2) >= 12
        acc = data(:,10:12);
    end

    s.name = displayName;
    s.key = keyName;
    s.t = t;
    s.N = N;
    s.E = E;
    s.alt = alt;
    s.vel = vel;
    s.VT = VT;
    s.acc = acc;
    s.euler = euler;
    s.color = colorValue;
end

function [t_common, y1i, y2i] = align_by_time(t1, y1, t2, y2)
    t_common = [];
    y1i = [];
    y2i = [];

    if isempty(t1) || isempty(t2) || isempty(y1) || isempty(y2)
        return;
    end

    t_start = max(t1(1), t2(1));
    t_end   = min(t1(end), t2(end));

    if t_end <= t_start
        return;
    end

    nPts = min([numel(t1), numel(t2), 5000]);
    t_common = linspace(t_start, t_end, nPts).';

    y1i = interp1(t1, y1, t_common, 'linear', 'extrap');
    y2i = interp1(t2, y2, t_common, 'linear', 'extrap');
end

function labels = build_legend(series, includeWPs)
    labels = cell(1, numel(series) + double(includeWPs && exist('WPs','var')));
    for kk = 1:numel(series)
        labels{kk} = series(kk).name;
    end
    if includeWPs && exist('WPs','var')
        labels{end} = 'Waypoints';
    end
end

function labels = build_legend_with_vt(series)
    labels = {};
    for kk = 1:numel(series)
        if ~isempty(series(kk).VT)
            if strcmp(series(kk).key, 'xplane')
                labels{end+1} = 'XPlane true airspeed'; %#ok<AGROW>
            else
                labels{end+1} = series(kk).name; %#ok<AGROW>
            end
        end
    end
end

function labels = build_error_legend(series, idx_ref)
    labels = {};
    for kk = 1:numel(series)
        if kk ~= idx_ref
            labels{end+1} = series(kk).name; %#ok<AGROW>
        end
    end
end

function plot_waypoints_3d()
    if exist('WPs','var')
        plot3(WPs(:,2), WPs(:,1), WPs(:,3), 'ks', ...
            'MarkerSize', 8, 'MarkerFaceColor', 'y');
    end
end

function plot_waypoints_2d()
    if exist('WPs','var')
        plot(WPs(:,2), WPs(:,1), 'ks', ...
            'MarkerSize', 8, 'MarkerFaceColor', 'y');

        if exist('R_accept','var')
            th = linspace(0, 2*pi, 100);
            for jj = 1:size(WPs,1)
                plot(WPs(jj,2) + R_accept*cos(th), ...
                     WPs(jj,1) + R_accept*sin(th), ...
                     'k--', 'LineWidth', 0.5);
            end
        end
    end
end

function plot_altitude_waypoints()
    if exist('WPs','var')
        alt_wps = unique(WPs(:,3));
        for jj = 1:length(alt_wps)
            yline(alt_wps(jj), 'k--', sprintf('%.0f m', alt_wps(jj)), ...
                'LineWidth', 0.6, 'LabelHorizontalAlignment', 'left');
        end
    end
end
