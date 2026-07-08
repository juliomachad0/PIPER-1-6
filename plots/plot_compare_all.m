%% plot_compare_all.m
% Comparação configurável entre XPlane, DBN e EKFs

fprintf('\n========== COMPARAÇÃO CONFIGURÁVEL DE NAVEGAÇÃO ==========\n');

%% ===================== ESCOLHA DOS MODELOS =====================

use_modelo      = false;
use_xplane_ref  = true;

use_dbn         = false;
use_dbn_em      = false;

use_ekf_di      = true;
use_ekf_di_em   =false;

use_ekf_indi    = false;
use_ekf_indi_em = false;

%% ===================== CORES =====================

colors.modelo      = [0.000 0.250 1.000];
colors.xplane      = [1.000 0.000 0.000];

colors.dbn         = [0.000 0.650 0.000];
colors.dbn_em      = [0.000 0.700 0.700];

colors.ekf_di      = [0.850 0.325 0.098];
colors.ekf_di_em   = [0.494 0.184 0.556];

colors.ekf_indi    = [0.929 0.694 0.125];
colors.ekf_indi_em = [0.250 0.250 0.250];

lineWidth = 1.4;

%% ===================== WAYPOINTS =====================

try
    WPs_local = WPs;
catch
    WPs_local = [];
end

try
    R_accept_local = R_accept;
catch
    R_accept_local = [];
end

%% ===================== CARREGAR SÉRIES =====================

series = struct( ...
    'name', {}, ...
    'key', {}, ...
    't', {}, ...
    'N', {}, ...
    'E', {}, ...
    'alt', {}, ...
    'vel', {}, ...
    'VT', {}, ...
    'acc', {}, ...
    'euler', {}, ...
    'xhat', {}, ...
    'color', {} ...
);

%% Modelo matemático
if use_modelo
    try
        data = out.Y.signals.values;
        t = out.Y.time;

        s = new_series('Modelo', 'modelo', colors.modelo);
        s.t = t;
        s.N = data(:,10);
        s.E = data(:,11);
        s.alt = data(:,12);

        if size(data,2) >= 3
            s.VT = sqrt(data(:,1).^2 + data(:,2).^2 + data(:,3).^2);
        end

        series(end+1) = s;
    catch
        warning('Modelo out.Y não encontrado.');
    end
end

%% XPlane
if use_xplane_ref
    try
        data = out.XplaneSimulationData.signals.values;
        t = out.XplaneSimulationData.time;

        s = new_series('XPlane', 'xplane', colors.xplane);
        s.t = t;
        s.VT = data(:,1);
        s.alt = data(:,4);
        s.N = data(:,9);
        s.E = data(:,10);

        series(end+1) = s;
    catch
        warning('XPlane out.XplaneSimulationData não encontrado.');
    end
end

%% DBN sem erro
if use_dbn
    try
        data = get_first_available({'DBN_Data','DBN_data','BDN_data'});
        t = get_time_first_available({'DBN_Data','DBN_data','BDN_data'});

        s = read_dbn_format(data, t, 'DBN', 'dbn', colors.dbn);

        series(end+1) = s;
    catch
        warning('DBN não encontrado.');
    end
end

%% DBN com erro
if use_dbn_em
    try
        data = get_first_available({'DBN_data_with_error','DBN_Data_with_error'});
        t = get_time_first_available({'DBN_data_with_error','DBN_Data_with_error'});

        s = read_dbn_format(data, t, 'DBN EM', 'dbn_em', colors.dbn_em);

        series(end+1) = s;
    catch
        warning('DBN com erro não encontrado.');
    end
end

%% EKF direto sem erro
if use_ekf_di
    try
        data = get_first_available({'EKF_DI_data','EKF_DI'});
        t = get_time_first_available({'EKF_DI_data','EKF_DI'});

        s = read_ekf_format(data, t, 'EKF DI', 'ekf_di', colors.ekf_di);

        series(end+1) = s;
    catch
        warning('EKF_DI não encontrado.');
    end
end

%% EKF direto com erro
if use_ekf_di_em
    try
        data = get_first_available({'EKF_DI_EM_data','EKF_DI_EM'});
        t = get_time_first_available({'EKF_DI_EM_data','EKF_DI_EM'});

        s = read_ekf_format(data, t, 'EKF DI EM', 'ekf_di_em', colors.ekf_di_em);

        series(end+1) = s;
    catch
        warning('EKF_DI_EM não encontrado.');
    end
end

%% EKF indireto sem erro
if use_ekf_indi
    try
        data = get_first_available({'EKF_INDI_data','EKF_INDI'});
        t = get_time_first_available({'EKF_INDI_data','EKF_INDI'});

        s = read_ekf_format(data, t, 'EKF INDI', 'ekf_indi', colors.ekf_indi);

        series(end+1) = s;
    catch
        warning('EKF_INDI não encontrado.');
    end
end

%% EKF indireto com erro
if use_ekf_indi_em
    try
        data = get_first_available({'EKF_INDI_EM_data','EKF_INDI_EM'});
        t = get_time_first_available({'EKF_INDI_EM_data','EKF_INDI_EM'});

        s = read_ekf_format(data, t, 'EKF INDI EM', 'ekf_indi_em', colors.ekf_indi_em);

        series(end+1) = s;
    catch
        warning('EKF_INDI_EM não encontrado.');
    end
end

%% Verificação

if isempty(series)
    error('Nenhuma série carregada.');
end

idx_ref = find(strcmp({series.key}, 'xplane'), 1);

if isempty(idx_ref)
    warning('XPlane não foi carregado. Erros contra referência não serão plotados.');
end

%% ===================== FIGURA =====================

figure('Name','Comparação Navegação', 'Position',[80 40 1650 950]);

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

%% 2 - Vista superior

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
    plot(series(k).t, series(k).alt, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
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
    if ~isempty(series(k).VT)
        plot(series(k).t, series(k).VT, ...
            'Color', series(k).color, 'LineWidth', lineWidth);
    end
end

grid on
xlabel('Tempo [s]')
ylabel('Velocidade [m/s]')
title('Velocidade Escalar')
legend(build_vt_legend(series), 'Location','best')
hold off

%% 5 - Erro horizontal vs XPlane

subplot(3,2,5)
hold on

if ~isempty(idx_ref)

    ref = series(idx_ref);

    for k = 1:numel(series)

        if k == idx_ref
            continue;
        end

        [tc, Nref, Nk] = align_by_time(ref.t, ref.N, series(k).t, series(k).N);
        [~,  Eref, Ek] = align_by_time(ref.t, ref.E, series(k).t, series(k).E);

        erro_h = sqrt((Nref - Nk).^2 + (Eref - Ek).^2);

        plot(tc, erro_h, ...
            'Color', series(k).color, 'LineWidth', lineWidth);
    end

    yline(0, 'k--', 'XPlane ref');

    grid on
    xlabel('Tempo [s]')
    ylabel('Erro horizontal [m]')
    title('Erro Horizontal vs XPlane')
    legend(build_error_legend(series, idx_ref), 'Location','best')

end

hold off

%% 6 - Erro altitude vs XPlane

subplot(3,2,6)
hold on

if ~isempty(idx_ref)

    ref = series(idx_ref);

    for k = 1:numel(series)

        if k == idx_ref
            continue;
        end

        [tc, alt_ref, alt_k] = align_by_time(ref.t, ref.alt, series(k).t, series(k).alt);

        erro_alt = alt_ref - alt_k;

        plot(tc, erro_alt, ...
            'Color', series(k).color, 'LineWidth', lineWidth);
    end

    yline(0, 'k--', 'XPlane ref');

    grid on
    xlabel('Tempo [s]')
    ylabel('Erro altitude [m]')
    title('Erro de Altitude vs XPlane')
    legend(build_error_legend(series, idx_ref), 'Location','best')

end

hold off

sgtitle('XPlane x EKFs')

%% ===================== ESTATÍSTICAS =====================

fprintf('\n--- Séries carregadas ---\n');

for k = 1:numel(series)
    fprintf('%-12s | N0=%8.2f E0=%8.2f Alt0=%8.2f | Nf=%8.2f Ef=%8.2f Altf=%8.2f\n', ...
        series(k).name, ...
        series(k).N(1), series(k).E(1), series(k).alt(1), ...
        series(k).N(end), series(k).E(end), series(k).alt(end));
end

fprintf('\n========== FIM DA COMPARAÇÃO ==========\n');

%% ========================================================================
% FUNÇÕES LOCAIS
%% ========================================================================

function s = new_series(name, key, color)

    s.name = name;
    s.key = key;
    s.t = [];
    s.N = [];
    s.E = [];
    s.alt = [];
    s.vel = [];
    s.VT = [];
    s.acc = [];
    s.euler = [];
    s.xhat = [];
    s.color = color;

end

function s = read_dbn_format(data, t, name, key, color)

    s = new_series(name, key, color);

    s.t = t;

    s.euler = data(:,1:3);
    s.vel   = data(:,4:6);

    pos = data(:,7:9);

    s.N = pos(:,1);
    s.E = pos(:,2);
    s.alt = -pos(:,3);

    if size(data,2) >= 12
        s.acc = data(:,10:12);
    end

    s.VT = sqrt(s.vel(:,1).^2 + s.vel(:,2).^2 + s.vel(:,3).^2);

end

function s = read_ekf_format(data, t, name, key, color)

    s = new_series(name, key, color);

    s.t = t;

    % Novo formato dos EKFs:
    % 1:3    pos_out   = [N E altitude]
    % 4:6    euler_out
    % 7:9    vel_out
    % 10:12  acc_n_out
    % 13:33  xhat_out

    pos = data(:,1:3);

    s.N = pos(:,1);
    s.E = pos(:,2);
    s.alt = pos(:,3);

    s.euler = data(:,4:6);
    s.vel   = data(:,7:9);
    s.acc   = data(:,10:12);

    s.VT = sqrt(s.vel(:,1).^2 + s.vel(:,2).^2 + s.vel(:,3).^2);

    if size(data,2) >= 33
        s.xhat = data(:,13:33);
    end

end

function data = get_first_available(names)

    for i = 1:numel(names)

        name_i = names{i};

        try
            sig = evalin('base', ['out.' name_i]);
            data = sig.signals.values;
            return;
        catch
        end
    end

    error('Nenhum dos sinais solicitados foi encontrado.');

end

function t = get_time_first_available(names)

    for i = 1:numel(names)

        name_i = names{i};

        try
            sig = evalin('base', ['out.' name_i]);
            t = sig.time;
            return;
        catch
        end
    end

    error('Nenhum tempo encontrado.');

end

function [tc, yref_i, y_i] = align_by_time(tref, yref, t, y)

    tref = tref(:);
    t = t(:);
    yref = yref(:);
    y = y(:);

    t0 = max(tref(1), t(1));
    tf = min(tref(end), t(end));

    N = min([length(tref), length(t), 5000]);

    tc = linspace(t0, tf, N).';

    yref_i = interp1(tref, yref, tc, 'linear');
    y_i    = interp1(t, y, tc, 'linear');

end

function labels = build_legend(series, WPs_local)

    labels = cell(1,numel(series));

    for k = 1:numel(series)
        labels{k} = series(k).name;
    end

    if ~isempty(WPs_local)
        labels{end+1} = 'Waypoints';
    end

end

function labels = build_vt_legend(series)

    labels = {};

    for k = 1:numel(series)

        if ~isempty(series(k).VT)

            if strcmp(series(k).key, 'xplane')
                labels{end+1} = 'XPlane true airspeed'; %#ok<AGROW>
            else
                labels{end+1} = series(k).name; %#ok<AGROW>
            end
        end
    end

end

function labels = build_error_legend(series, idx_ref)

    labels = {};

    for k = 1:numel(series)

        if k ~= idx_ref
            labels{end+1} = series(k).name; %#ok<AGROW>
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

            for j = 1:size(WPs_local,1)
                plot(WPs_local(j,2) + R_accept_local*cos(th), ...
                     WPs_local(j,1) + R_accept_local*sin(th), ...
                     'k--', 'LineWidth', 0.5);
            end
        end
    end

end

function plot_altitude_waypoints(WPs_local)

    if ~isempty(WPs_local)

        alt_wps = unique(WPs_local(:,3));

        for j = 1:length(alt_wps)
            yline(alt_wps(j), 'k--', sprintf('%.0f m', alt_wps(j)), ...
                'LineWidth', 0.6, 'LabelHorizontalAlignment', 'left');
        end
    end

end