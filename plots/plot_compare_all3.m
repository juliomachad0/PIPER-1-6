%% plot_compare_all3.m
% Comparação compacta:
%   Altitude
%   Vista superior
%   MSE 3D acumulado
%   MSE altitude acumulado
%   MSE horizontal acumulado

fprintf('\n========== COMPARAÇÃO ALL3: ALTITUDE, TRAJETÓRIA E MSE ==========\n');

%% ===================== ESCOLHA DOS MODELOS =====================

use_xplane_ref  = true;

use_ekf_di      = true;
use_ekf_di_em   = true;

use_ekf_indi    = true;
use_ekf_indi_em = true;

use_dbn         = false;
use_dbn_em      = false;

%% ===================== CORES =====================

colors.xplane      = [0.000 0.500 0.000];  % verde escuro

colors.dbn         = [1.000 0.000 1.000];  % magenta
colors.dbn_em      = [0.500 0.500 0.500];  % cinza

colors.ekf_di      = [0.000 0.000 1.000];  % azul
colors.ekf_di_em   = [0.500 0.000 0.500];  % roxo

colors.ekf_indi    = [1.000 0.000 0.000];  % vermelho
colors.ekf_indi_em = [0.500 0.250 0.000];  % marrom

lineWidth = 1.4;

%% ===================== INTERVALOS SEM CORREÇÃO =====================

try
    range_time_without_correction_local = evalin('base','range_time_without_correction');
catch
    range_time_without_correction_local = [0 0];
end

if isempty(range_time_without_correction_local) || size(range_time_without_correction_local,2) ~= 2
    range_time_without_correction_local = [0 0];
end

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

%% ===================== VERIFICAR OUT =====================

if ~evalin('base','exist(''out'',''var'')')
    error('Base workspace variable ''out'' not found. Load simulation data before running this script.');
end

out_local = evalin('base','out');

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
    'euler', {}, ...
    'color', {} ...
);

%% XPlane referência
if use_xplane_ref
    try
        [data, t] = get_out_signal(out_local, {'XplaneSimulationData'});

        s = new_series('XPlane', 'xplane', colors.xplane);

        s.t = t;
        s.VT = data(:,1);
        s.alt = data(:,4);
        s.N = data(:,9);
        s.E = data(:,10);

        s.euler = [data(:,5), data(:,2), data(:,7)];

        series(end+1) = s;
    catch ME
        warning('XPlane out.XplaneSimulationData não encontrado. Erro: %s', ME.message);
    end
end

%% DBN sem erro
if use_dbn
    try
        % teste nomes diferentes da mesma variavel
        [data, t] = get_out_signal(out_local, {'DBN_Data','DBN_data','BDN_data'});

        s = read_dbn_format(data, t, 'DBN', 'dbn', colors.dbn);

        series(end+1) = s;
    catch ME
        warning('DBN não encontrado. Erro: %s', ME.message);
    end
end

%% DBN com erro
if use_dbn_em
    try
        [data, t] = get_out_signal(out_local, {'DBN_data_with_error','DBN_Data_with_error','BDN_data_with_error'});

        s = read_dbn_format(data, t, 'DBN EM', 'dbn_em', colors.dbn_em);

        series(end+1) = s;
    catch ME
        warning('DBN com erro não encontrado. Erro: %s', ME.message);
    end
end

%% EKF direto sem erro
if use_ekf_di
    try
        [data, t] = get_out_signal(out_local, {'EKF_DI_data','EKF_DI'});

        s = read_ekf_format(data, t, 'EKF DI', 'ekf_di', colors.ekf_di);

        series(end+1) = s;
    catch ME
        warning('EKF_DI não encontrado. Erro: %s', ME.message);
    end
end

%% EKF direto com erro
if use_ekf_di_em
    try
        [data, t] = get_out_signal(out_local, {'EKF_DI_EM_data','EKF_DI_EM'});

        s = read_ekf_format(data, t, 'EKF DI EM', 'ekf_di_em', colors.ekf_di_em);

        series(end+1) = s;
    catch ME
        warning('EKF_DI_EM não encontrado. Erro: %s', ME.message);
    end
end

%% EKF indireto sem erro
if use_ekf_indi
    try
        [data, t] = get_out_signal(out_local, {'EKF_INDI_data','EKF_INDI'});

        s = read_ekf_format(data, t, 'EKF INDI', 'ekf_indi', colors.ekf_indi);

        series(end+1) = s;
    catch ME
        warning('EKF_INDI não encontrado. Erro: %s', ME.message);
    end
end

%% EKF indireto com erro
if use_ekf_indi_em
    try
        [data, t] = get_out_signal(out_local, {'EKF_INDI_EM_data','EKF_INDI_EM'});

        s = read_ekf_format(data, t, 'EKF INDI EM', 'ekf_indi_em', colors.ekf_indi_em);

        series(end+1) = s;
    catch ME
        warning('EKF_INDI_EM não encontrado. Erro: %s', ME.message);
    end
end

%% Verificação

if isempty(series)
    error('Nenhuma série carregada.');
end

idx_ref = find(strcmp({series.key}, 'xplane'), 1);

if isempty(idx_ref)
    error('XPlane precisa estar carregado como referência.');
end

ref = series(idx_ref);

%% ===================== FIGURA =====================

figure('Name','Comparação ALL3 - Altitude, Vista Superior e MSE', ...
       'Position',[80 40 1650 950]);

%% 1 - Altitude

subplot(3,2,1)
hold on

for k = 1:numel(series)
    plot(series(k).t, series(k).alt, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_altitude_waypoints(WPs_local);
plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('Altitude [m]')
title('Altitude')
legend({series.name}, 'Location','best')
hold off

%% 2 - MSE 3D acumulado

subplot(3,2,2)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, Nref, Nk]     = align_by_time(ref.t, ref.N,   series(k).t, series(k).N);
    [~,  Eref, Ek]     = align_by_time(ref.t, ref.E,   series(k).t, series(k).E);
    [~,  altref, altk] = align_by_time(ref.t, ref.alt, series(k).t, series(k).alt);

    e_3d2 = (Nref - Nk).^2 + (Eref - Ek).^2 + (altref - altk).^2;
    mse_3d = cumulative_mean(e_3d2);

    plot(tc, mse_3d, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end
plot_correction_windows(gca, range_time_without_correction_local, true);
grid on
xlabel('Tempo [s]')
ylabel('MSE 3D [m^2]')
title('MSE 3D Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 3 - Vista superior em subplot(3,2,[3 5])

subplot(3,2,[3 5])
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

%% 4 - MSE altitude acumulado

subplot(3,2,4)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, alt_ref, alt_k] = align_by_time(ref.t, ref.alt, series(k).t, series(k).alt);

    e_alt2 = (alt_ref - alt_k).^2;
    mse_alt = cumulative_mean(e_alt2);

    plot(tc, mse_alt, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);
grid on
xlabel('Tempo [s]')
ylabel('MSE altitude [m^2]')
title('MSE de Altitude Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 5 - MSE horizontal acumulado

subplot(3,2,6)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, Nref, Nk] = align_by_time(ref.t, ref.N, series(k).t, series(k).N);
    [~,  Eref, Ek] = align_by_time(ref.t, ref.E, series(k).t, series(k).E);

    e_h2 = (Nref - Nk).^2 + (Eref - Ek).^2;
    mse_h = cumulative_mean(e_h2);

    plot(tc, mse_h, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);
grid on
xlabel('Tempo [s]')
ylabel('MSE horizontal [m^2]')
title('MSE Horizontal Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% Título geral da figura

annotation('textbox', [0 0.955 1 0.035], ...
    'String', 'Altitude, Vista Superior e MSE - XPlane x Estimadores', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'FontWeight', 'bold', ...
    'FontSize', 14, ...
    'EdgeColor', 'none');

%% ===================== ESTATÍSTICAS =====================

fprintf('\n--- Métricas finais vs XPlane ---\n');

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [~, Nref, Nk]     = align_by_time(ref.t, ref.N,   series(k).t, series(k).N);
    [~, Eref, Ek]     = align_by_time(ref.t, ref.E,   series(k).t, series(k).E);
    [~, altref, altk] = align_by_time(ref.t, ref.alt, series(k).t, series(k).alt);

    e_h2   = (Nref - Nk).^2 + (Eref - Ek).^2;
    e_alt2 = (altref - altk).^2;
    e_3d2  = e_h2 + e_alt2;

    mse_h_final   = mean(e_h2);
    mse_alt_final = mean(e_alt2);
    mse_3d_final  = mean(e_3d2);

    rmse_h_final   = sqrt(mse_h_final);
    rmse_alt_final = sqrt(mse_alt_final);
    rmse_3d_final  = sqrt(mse_3d_final);

    fprintf('%-12s | MSE H=%.3f m^2 | RMSE H=%.3f m | MSE Alt=%.3f m^2 | RMSE Alt=%.3f m | MSE 3D=%.3f m^2 | RMSE 3D=%.3f m\n', ...
        series(k).name, ...
        mse_h_final, rmse_h_final, ...
        mse_alt_final, rmse_alt_final, ...
        mse_3d_final, rmse_3d_final);
end

fprintf('\n========== FIM DO PLOT_COMPARE_ALL3 ==========\n');

%% ========================================================================
% FUNÇÕES LOCAIS
%% ========================================================================
%% new_series
function s = new_series(name, key, color)

    s.name = name;
    s.key = key;
    s.t = [];
    s.N = [];
    s.E = [];
    s.alt = [];
    s.vel = [];
    s.VT = [];
    s.euler = [];
    s.color = color;

end
%% read_dbn_format
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
%% read_ekf_format
function s = read_ekf_format(data, t, name, key, color)

    s = new_series(name, key, color);

    s.t = t;

    % EKF:
    % 1:3    pos_out   = [N E altitude]
    % 4:6    euler_out = [phi theta psi]
    % 7:9    vel_out
    % 10:12  acc_n_out
    % 13:33  xhat_out

    pos = data(:,1:3);

    s.N = pos(:,1);
    s.E = pos(:,2);
    s.alt = pos(:,3);

    s.euler = data(:,4:6);
    s.vel   = data(:,7:9);

    s.VT = sqrt(s.vel(:,1).^2 + s.vel(:,2).^2 + s.vel(:,3).^2);

end
%% get_out_signal
function [data, t] = get_out_signal(out_local, names)

    for i = 1:numel(names)

        name_i = names{i};

        try
            sig = out_local.(name_i);
        catch
            try
                sig = out_local.get(name_i);
            catch
                continue;
            end
        end

        try
            if isstruct(sig) && isfield(sig,'signals') && isfield(sig,'time')
                data = sig.signals.values;
                t = sig.time;

                data = squeeze(data);

                if size(data,1) ~= numel(t) && size(data,2) == numel(t)
                    data = data.';
                end

                return;
            end

            if isa(sig,'timeseries')
                data = sig.Data;
                t = sig.Time;

                data = squeeze(data);

                if size(data,1) ~= numel(t) && size(data,2) == numel(t)
                    data = data.';
                end

                return;
            end

            if isa(sig,'Simulink.SimulationData.Signal')
                data = sig.Values.Data;
                t = sig.Values.Time;

                data = squeeze(data);

                if size(data,1) ~= numel(t) && size(data,2) == numel(t)
                    data = data.';
                end

                return;
            end

        catch
        end
    end

    error('Nenhum dos sinais solicitados foi encontrado em out.');

end
%% align_by_time
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
%% cumulative_mean
function mse = cumulative_mean(x)

    n = (1:length(x)).';
    mse = cumsum(x) ./ n;

end
%% build_legend
function labels = build_legend(series, WPs_local)

    labels = cell(1,numel(series));

    for k = 1:numel(series)
        labels{k} = series(k).name;
    end

    if ~isempty(WPs_local)
        labels{end+1} = 'Waypoints';
    end

end
%% build_error_legend
function labels = build_error_legend(series, idx_ref)

    labels = {};

    for k = 1:numel(series)

        if k ~= idx_ref
            labels{end+1} = series(k).name; %#ok<AGROW>
        end
    end

end
%% plot_waypoints_2d
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
%% plot_altitude_waypoints
function plot_altitude_waypoints(WPs_local)

    if ~isempty(WPs_local)

        alt_wps = unique(WPs_local(:,3));

        for j = 1:length(alt_wps)
            yline(alt_wps(j), 'k--', sprintf('%.0f m', alt_wps(j)), ...
                'LineWidth', 0.6, 'LabelHorizontalAlignment', 'left');
        end
    end

end
%% plot_correction_windows
function plot_correction_windows(ax, range_no_corr, show_labels, label_y_value)

if nargin < 4
    label_y_value = [];
end

if isempty(range_no_corr) || size(range_no_corr,2) ~= 2
    return;
end

axes(ax);

yl = ylim(ax);
y_span = yl(2) - yl(1);

if y_span <= 0
    y_span = 1;
end

%% Calcula posição vertical do texto
if ~isempty(label_y_value)
    label_y = label_y_value;
else
    line_handles = findobj(ax, 'Type', 'line');

    y_all = [];

    for ii = 1:numel(line_handles)
        try
            y_i = line_handles(ii).YData;
            y_i = y_i(:);
            y_i = y_i(isfinite(y_i));

            if ~isempty(y_i)
                y_all = [y_all; y_i]; %#ok<AGROW>
            end
        catch
        end
    end

    if ~isempty(y_all)
        label_y = 0.5 * (min(y_all) + max(y_all));
    else
        label_y = 0.5 * (yl(1) + yl(2));
    end
end

%% Plota janelas sem correção
for kk = 1:size(range_no_corr,1)

    ti = range_no_corr(kk,1);
    tf = range_no_corr(kk,2);

    if ti == 0 && tf == 0
        continue;
    end

    if tf <= ti
        continue;
    end

    hp = patch(ax, ...
        [ti tf tf ti], ...
        [yl(1) yl(1) yl(2) yl(2)], ...
        [0.85 0.85 0.85], ...
        'FaceAlpha', 0.22, ...
        'EdgeColor', 'none', ...
        'HandleVisibility', 'off');

    try
        uistack(hp, 'bottom');
    catch
    end

    xline(ax, ti, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(ax, tf, 'k-',  'LineWidth', 1.0, 'HandleVisibility', 'off');

    if show_labels
        text(ax, ...
            (ti + tf)/2, label_y, ...
            sprintf('sem correção %d', kk), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'FontSize', 8, ...
            'Color', [0.15 0.15 0.15], ...
            'BackgroundColor', [1 1 1], ...
            'Margin', 1, ...
            'HandleVisibility', 'off');
    end
end

ylim(ax, yl);

end