%% plot_compare_all.m
% Comparação configurável entre:
%   Modelo Matemático
%   XPlane referência
%   DBN sem modelagem de erro
%   DBN com modelagem de erro
%   EKF Direto sem modelagem de erro
%   EKF Direto com modelagem de erro
%   EKF Indireto sem modelagem de erro
%   EKF Indireto com modelagem de erro

fprintf('\n========== COMPARAÇÃO CONFIGURÁVEL DE NAVEGAÇÃO ==========\n');

%% ===================== ESCOLHA DOS MODELOS =====================

use_modelo        = false;
use_xplane_ref    = true;

use_dbn           = false;
use_dbn_em        = false;

use_ekf_di        = true;
use_ekf_di_em     = true;

use_ekf_indi      = true;
use_ekf_indi_em   = true;

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

%% ===================== CARREGAR DADOS AUXILIARES =================

WPs_local = [];
R_accept_local = [];

try
    WPs_local = evalin('base','WPs');
catch
end

try
    R_accept_local = evalin('base','R_accept');
catch
end

%% ===================== CARREGAR SÉRIES ==========================

series = struct('name', {}, 'key', {}, 't', {}, 'N', {}, 'E', {}, 'alt', {}, ...
                'vel', {}, 'VT', {}, 'acc', {}, 'euler', {}, 'xhat', {}, ...
                'color', {});

%% Modelo Matemático

if use_modelo
    s = load_model_solution('Y', 'Modelo', 'modelo', colors.modelo);

    if ~isempty(s.name)
        series(end+1) = s; %#ok<SAGROW>
    end
end

%% XPlane referência

if use_xplane_ref
    s = load_xplane_solution('XplaneSimulationData', 'XPlane', 'xplane', colors.xplane);

    if ~isempty(s.name)
        series(end+1) = s; %#ok<SAGROW>
    end
end

%% DBN sem modelagem de erro
% Mantém a leitura antiga do DBN:
% 1:3 Euler
% 4:6 Vel
% 7:9 Pos NED [N E D]
% 10:12 Acc

if use_dbn
    s = load_nav_solution({'DBN_Data','DBN_data','BDN_data'}, ...
        'DBN', 'dbn', colors.dbn);

    if ~isempty(s.name)
        series(end+1) = s; %#ok<SAGROW>
    end
end

%% DBN com modelagem de erro
% Mantém a mesma convenção do DBN sem erro.

if use_dbn_em
    s = load_nav_solution({'DBN_data_with_error','DBN_Data_with_error'}, ...
        'DBN EM', 'dbn_em', colors.dbn_em);

    if ~isempty(s.name)
        series(end+1) = s; %#ok<SAGROW>
    end
end

%% EKF Direto sem modelagem de erro
% Novo padrão:
% 1:3 pos_out
% 4:6 euler_out
% 7:9 vel_out
% 10:12 acc_n_out
% 13:33 xhat_out

if use_ekf_di
    s = load_ekf_data_solution({'EKF_DI_data','EKF_DI'}, ...
        'EKF DI', 'ekf_di', colors.ekf_di);

    if ~isempty(s.name)
        series(end+1) = s; %#ok<SAGROW>
    end
end

%% EKF Direto com modelagem de erro

if use_ekf_di_em
    s = load_ekf_data_solution({'EKF_DI_EM_data','EKF_DI_EM'}, ...
        'EKF DI EM', 'ekf_di_em', colors.ekf_di_em);

    if ~isempty(s.name)
        series(end+1) = s; %#ok<SAGROW>
    end
end

%% EKF Indireto sem modelagem de erro

if use_ekf_indi
    s = load_ekf_data_solution({'EKF_INDI_data','EKF_INDI'}, ...
        'EKF INDI', 'ekf_indi', colors.ekf_indi);

    if ~isempty(s.name)
        series(end+1) = s; %#ok<SAGROW>
    end
end

%% EKF Indireto com modelagem de erro

if use_ekf_indi_em
    s = load_ekf_data_solution({'EKF_INDI_EM_data','EKF_INDI_EM'}, ...
        'EKF INDI EM', 'ekf_indi_em', colors.ekf_indi_em);

    if ~isempty(s.name)
        series(end+1) = s; %#ok<SAGROW>
    end
end

%% Verificação

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

plot_waypoints_3d(WPs_local);

grid on
axis equal
xlabel('Leste [m]')
ylabel('Norte [m]')
zlabel('Altitude [m]')
title('Trajetória 3D')
legend(build_legend(series, WPs_local), 'Location','best')
view(30,25)
hold off

%% 2 - Vista Superior

subplot(3,2,2)
hold on

for k = 1:numel(series)
    plot(series(k).E, series(k).N, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_waypoints_2d(WPs_local, R_accept_local);

grid on
axis equal
xlabel('Leste [m]')
ylabel('Norte [m]')
title('Vista Superior')
legend(build_legend(series, WPs_local), 'Location','best')
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

plot_altitude_waypoints(WPs_local);

grid on
xlabel('Tempo [s]')
ylabel('Altitude [m]')
title('Altitude')
legend({series.name}, 'Location','best')
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

            plot(t_common, erro_h, ...
                'Color', series(k).color, 'LineWidth', lineWidth);
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

            plot(t_common, erro_alt, ...
                'Color', series(k).color, 'LineWidth', lineWidth);
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

sgtitle('XPlane x EKFs')

%% ===================== ESTATÍSTICAS ===============================

fprintf('\n--- Séries carregadas ---\n');

for k = 1:numel(series)
    fprintf('  %-12s | N0=%8.2f E0=%8.2f Alt0=%8.2f | Nf=%8.2f Ef=%8.2f Altf=%8.2f\n', ...
        series(k).name, ...
        series(k).N(1), series(k).E(1), series(k).alt(1), ...
        series(k).N(end), series(k).E(end), series(k).alt(end));
end

fprintf('\n========== FIM DA COMPARAÇÃO CONFIGURÁVEL ==========\n');

%% ========================================================================
% FUNÇÕES LOCAIS
%% ========================================================================

function s = load_model_solution(fieldName, displayName, keyName, colorValue)

    s = empty_series();

    try
        [data, t] = get_out_data(fieldName);
    catch
        warning('Dados do Modelo out.%s não encontrados.', fieldName);
        return;
    end

    if size(data,2) < 12
        warning('Dados do Modelo possuem menos de 12 colunas.');
        return;
    end

    s.name = displayName;
    s.key = keyName;
    s.t = t;

    s.N = data(:,10);
    s.E = data(:,11);
    s.alt = data(:,12);

    s.euler = [];
    s.vel = [];
    s.acc = [];
    s.xhat = [];

    if size(data,2) >= 3
        s.VT = sqrt(data(:,1).^2 + data(:,2).^2 + data(:,3).^2);
    else
        s.VT = [];
    end

    s.color = colorValue;

end

function s = load_xplane_solution(fieldName, displayName, keyName, colorValue)

    s = empty_series();

    try
        [data, t] = get_out_data(fieldName);
    catch
        warning('Dados out.%s não encontrados.', fieldName);
        return;
    end

    if size(data,2) < 10
        warning('Dados XPlane possuem menos de 10 colunas.');
        return;
    end

    s.name = displayName;
    s.key = keyName;
    s.t = t;

    s.N = data(:,9);
    s.E = data(:,10);
    s.alt = data(:,4);

    s.vel = [];
    s.VT = data(:,1);
    s.acc = [];
    s.euler = [];
    s.xhat = [];
    s.color = colorValue;

end

function s = load_nav_solution(fieldNames, displayName, keyName, colorValue)

    s = empty_series();

    found = false;

    for ii = 1:numel(fieldNames)

        fname = fieldNames{ii};

        try
            [data, t] = get_out_data(fname);
            found = true;
            break;
        catch
        end
    end

    if ~found
        warning('Dados %s não encontrados em out.', displayName);
        return;
    end

    if size(data,2) < 9
        warning('Dados %s encontrados, mas possuem menos de 9 colunas.', displayName);
        return;
    end

    % DBN antigo:
    % 1:3   euler
    % 4:6   vel
    % 7:9   pos [N E D]
    % 10:12 acc

    s.name = displayName;
    s.key = keyName;
    s.t = t;

    s.euler = data(:,1:3);
    s.vel   = data(:,4:6);

    pos = data(:,7:9);

    s.N = pos(:,1);
    s.E = pos(:,2);

    D = pos(:,3);
    s.alt = -D;

    s.VT = sqrt(s.vel(:,1).^2 + s.vel(:,2).^2 + s.vel(:,3).^2);

    if size(data,2) >= 12
        s.acc = data(:,10:12);
    else
        s.acc = [];
    end

    s.xhat = [];
    s.color = colorValue;

end

function s = load_ekf_data_solution(fieldNames, displayName, keyName, colorValue)

    s = empty_series();

    found = false;

    for ii = 1:numel(fieldNames)

        fname = fieldNames{ii};

        try
            [data, t] = get_out_data(fname);
            found = true;
            break;
        catch
        end
    end

    if ~found
        warning('Dados %s não encontrados em out.', displayName);
        return;
    end

    if size(data,2) < 12
        warning('Dados %s encontrados, mas possuem menos de 12 colunas.', displayName);
        return;
    end

    % EKF novo:
    % 1:3    pos_out   = [N E altitude]
    % 4:6    euler_out = [phi theta psi]
    % 7:9    vel_out   = [vN vE vD]
    % 10:12  acc_n_out = [aN aE aD]
    % 13:33  xhat_out  = estados completos

    pos   = data(:,1:3);
    euler = data(:,4:6);
    vel   = data(:,7:9);
    acc   = data(:,10:12);

    s.name = displayName;
    s.key = keyName;
    s.t = t;

    s.N = pos(:,1);
    s.E = pos(:,2);
    s.alt = pos(:,3);

    s.euler = euler;
    s.vel = vel;
    s.acc = acc;

    s.VT = sqrt(vel(:,1).^2 + vel(:,2).^2 + vel(:,3).^2);

    if size(data,2) >= 33
        s.xhat = data(:,13:33);
    else
        s.xhat = [];
    end

    s.color = colorValue;

end

function [data, t] = get_out_data(fieldName)

    outObj = evalin('base','out');

    try
        sig = outObj.(fieldName);
    catch
        sig = outObj.get(fieldName);
    end

    if isstruct(sig) && isfield(sig,'signals')
        data = sig.signals.values;
        t = sig.time;
    elseif isa(sig,'timeseries')
        data = sig.Data;
        t = sig.Time;
    else
        error('Formato não reconhecido para out.%s.', fieldName);
    end

end

function s = empty_series()

    s = struct( ...
        'name', '', ...
        'key', '', ...
        't', [], ...
        'N', [], ...
        'E', [], ...
        'alt', [], ...
        'vel', [], ...
        'VT', [], ...
        'acc', [], ...
        'euler', [], ...
        'xhat', [], ...
        'color', [] ...
    );

end

function [t_common, y1i, y2i] = align_by_time(t1, y1, t2, y2)

    t_common = [];
    y1i = [];
    y2i = [];

    if isempty(t1) || isempty(t2) || isempty(y1) || isempty(y2)
        return;
    end

    t1 = t1(:);
    t2 = t2(:);
    y1 = y1(:);
    y2 = y2(:);

    [t1, ia] = unique(t1,'stable');
    y1 = y1(ia);

    [t2, ib] = unique(t2,'stable');
    y2 = y2(ib);

    t_start = max(t1(1), t2(1));
    t_end   = min(t1(end), t2(end));

    if t_end <= t_start
        return;
    end

    nPts = min([numel(t1), numel(t2), 5000]);

    t_common = linspace(t_start, t_end, nPts).';

    y1i = interp1(t1, y1, t_common, 'linear');
    y2i = interp1(t2, y2, t_common, 'linear');

end

function labels = build_legend(series, WPs_local)

    labels = cell(1,numel(series));

    for kk = 1:numel(series)
        labels{kk} = series(kk).name;
    end

    if ~isempty(WPs_local)
        labels{end+1} = 'Waypoints';
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

function plot_waypoints_3d(WPs_local)

    if ~isempty(WPs_local)
        plot3(WPs_local(:,2), WPs_local(:,1), WPs_local(:,3), ...
            'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
    end

end

function plot_waypoints_2d(WPs_local, R_accept_local)

    if ~isempty(WPs_local)

        plot(WPs_local(:,2), WPs_local(:,1), ...
            'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'y');

        if ~isempty(R_accept_local)

            th = linspace(0, 2*pi, 100);

            for jj = 1:size(WPs_local,1)
                plot(WPs_local(jj,2) + R_accept_local*cos(th), ...
                     WPs_local(jj,1) + R_accept_local*sin(th), ...
                     'k--', 'LineWidth', 0.5);
            end
        end
    end

end

function plot_altitude_waypoints(WPs_local)

    if ~isempty(WPs_local)

        alt_wps = unique(WPs_local(:,3));

        for jj = 1:length(alt_wps)
            yline(alt_wps(jj), 'k--', sprintf('%.0f m', alt_wps(jj)), ...
                'LineWidth', 0.6, 'LabelHorizontalAlignment', 'left');
        end
    end

end