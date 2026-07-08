%% plot_compare_all5.m
% Comparação de velocidades:
%   V_N, V_E, V_D
%   Velocidade em Relacao ao Solo 3D - NED
%   MSE acumulado de cada velocidade
%   Erro comparativo de V_3D vs XPlane

fprintf('\n========== COMPARAÇÃO ALL5: VELOCIDADES E MSE ==========\n');

%% ===================== ESCOLHA DOS MODELOS =====================

use_xplane_ref  = true;

use_dbn         = false;
use_dbn_em      = false;

use_ekf_di      = true;
use_ekf_di_em   = false;

use_ekf_indi    = true;
use_ekf_indi_em = false;

%% ===================== CORES =====================

colors.xplane      = [0.000 0.500 0.000];  % verde escuro

colors.dbn         = [1.000 0.000 1.000];  % magenta
colors.dbn_em      = [0.500 0.500 0.500];  % cinza

colors.ekf_di      = [0.000 0.000 1.000];  % azul
colors.ekf_di_em   = [0.500 0.000 0.500];  % roxo

colors.ekf_indi    = [1.000 0.000 0.000];  % vermelho
colors.ekf_indi_em = [0.500 0.250 0.000];  % marrom

colors.xplane_vt   = [0.000 0.000 0.000];  % preto para VT XPlane

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
    'vN', {}, ...
    'vE', {}, ...
    'vD', {}, ...
    'V3D', {}, ...
    'VT_xplane', {}, ...
    'color', {} ...
);

%% XPlane referência
if use_xplane_ref
    try
        [data, t] = get_out_signal(out_local, {'XplaneSimulationData'});

        s = new_series('XPlane', 'xplane', colors.xplane);

        s.t = t;

        % XPlane:
        % 14: vN
        % 15: vE
        % 16: vD
        s.vN = data(:,14);
        s.vE = data(:,15);
        s.vD = data(:,16);

        s.V3D = sqrt(s.vN.^2 + s.vE.^2 + s.vD.^2);

        % VT do XPlane:
        % 1: true airspeed
        s.VT_xplane = data(:,1);

        series(end+1) = s;
    catch ME
        warning('XPlane out.XplaneSimulationData não encontrado. Erro: %s', ME.message);
    end
end

%% DBN sem erro
if use_dbn
    try
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

figure('Name','Comparação ALL5 - Velocidades e MSE', ...
       'Position',[80 40 1650 1050]);

%% 1 - Velocidade Norte

subplot(5,2,1)
hold on

for k = 1:numel(series)
    plot(series(k).t, series(k).vN, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('V_N [m/s]')
title('Velocidade Norte - V_N')
legend({series.name}, 'Location','best')
hold off

%% 2 - MSE V_N

subplot(5,2,2)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, ref_i, y_i] = align_by_time(ref.t, ref.vN, series(k).t, series(k).vN);

    e2 = (ref_i - y_i).^2;
    mse_vN = cumulative_mean(e2);

    plot(tc, mse_vN, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, true);

grid on
xlabel('Tempo [s]')
ylabel('MSE V_N [(m/s)^2]')
title('MSE V_N Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 3 - Velocidade Leste

subplot(5,2,3)
hold on

for k = 1:numel(series)
    plot(series(k).t, series(k).vE, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('V_E [m/s]')
title('Velocidade Leste - V_E')
legend({series.name}, 'Location','best')
hold off

%% 4 - MSE V_E

subplot(5,2,4)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, ref_i, y_i] = align_by_time(ref.t, ref.vE, series(k).t, series(k).vE);

    e2 = (ref_i - y_i).^2;
    mse_vE = cumulative_mean(e2);

    plot(tc, mse_vE, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('MSE V_E [(m/s)^2]')
title('MSE V_E Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 5 - Velocidade Down

subplot(5,2,5)
hold on

for k = 1:numel(series)
    plot(series(k).t, series(k).vD, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('V_D [m/s]')
title('Velocidade Down - V_D')
legend({series.name}, 'Location','best')
hold off

%% 6 - MSE V_D

subplot(5,2,6)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, ref_i, y_i] = align_by_time(ref.t, ref.vD, series(k).t, series(k).vD);

    e2 = (ref_i - y_i).^2;
    mse_vD = cumulative_mean(e2);

    plot(tc, mse_vD, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('MSE V_D [(m/s)^2]')
title('MSE V_D Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 7 - Velocidade em Relação ao Solo 3D - NED

subplot(5,2,7)
hold on

for k = 1:numel(series)
    plot(series(k).t, series(k).V3D, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

if ~isempty(ref.VT_xplane)
    plot(ref.t, ref.VT_xplane, '--', ...
        'Color', colors.xplane_vt, ...
        'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('V_{3D} [m/s]')
title('Velocidade em Relação ao Solo 3D - NED')

legend_labels = {series.name};
legend_labels{end+1} = 'VT XPlane - true airspeed';

legend(legend_labels, 'Location','best')
hold off

%% 8 - MSE V_3D

subplot(5,2,8)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, ref_i, y_i] = align_by_time(ref.t, ref.V3D, series(k).t, series(k).V3D);

    e2 = (ref_i - y_i).^2;
    mse_v3d = cumulative_mean(e2);

    plot(tc, mse_v3d, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('MSE V_{3D} [(m/s)^2]')
title('MSE V_{3D} Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 9/10 - Erro comparativo V_3D

subplot(5,2,[9 10])
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, ref_i, y_i] = align_by_time(ref.t, ref.V3D, series(k).t, series(k).V3D);

    erro_v3d = ref_i - y_i;

    plot(tc, erro_v3d, ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

yline(0, 'k--', 'XPlane ref');
plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('Erro V_{3D} [m/s]')
title('Erro de Velocidade em Relação ao Solo 3D - NED vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

annotation('textbox', [0 0.955 1 0.035], ...
    'String', 'Velocidades NED, V_{3D} e MSE - XPlane x Estimadores', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'FontWeight', 'bold', ...
    'FontSize', 14, ...
    'EdgeColor', 'none');

%% ===================== ESTATÍSTICAS =====================

fprintf('\n--- Métricas finais de velocidade vs XPlane ---\n');

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [~, vN_ref, vN_k]   = align_by_time(ref.t, ref.vN,  series(k).t, series(k).vN);
    [~, vE_ref, vE_k]   = align_by_time(ref.t, ref.vE,  series(k).t, series(k).vE);
    [~, vD_ref, vD_k]   = align_by_time(ref.t, ref.vD,  series(k).t, series(k).vD);
    [~, V3D_ref, V3D_k] = align_by_time(ref.t, ref.V3D, series(k).t, series(k).V3D);

    rmse_vN  = sqrt(mean((vN_ref - vN_k).^2));
    rmse_vE  = sqrt(mean((vE_ref - vE_k).^2));
    rmse_vD  = sqrt(mean((vD_ref - vD_k).^2));
    rmse_V3D = sqrt(mean((V3D_ref - V3D_k).^2));

    fprintf('%-12s | RMSE V_N=%.3f m/s | RMSE V_E=%.3f m/s | RMSE V_D=%.3f m/s | RMSE V3D=%.3f m/s\n', ...
        series(k).name, rmse_vN, rmse_vE, rmse_vD, rmse_V3D);
end

fprintf('\n========== FIM DO PLOT_COMPARE_ALL5 ==========\n');

%% ========================================================================
% FUNÇÕES LOCAIS
%% ========================================================================

function s = new_series(name, key, color)

    s.name = name;
    s.key = key;
    s.t = [];
    s.vN = [];
    s.vE = [];
    s.vD = [];
    s.V3D = [];
    s.VT_xplane = [];
    s.color = color;

end

function s = read_dbn_format(data, t, name, key, color)

    s = new_series(name, key, color);

    s.t = t;

    % DBN:
    % 4:6 = [vN vE vD]
    s.vN = data(:,4);
    s.vE = data(:,5);
    s.vD = data(:,6);

    s.V3D = sqrt(s.vN.^2 + s.vE.^2 + s.vD.^2);

end

function s = read_ekf_format(data, t, name, key, color)

    s = new_series(name, key, color);

    s.t = t;

    % EKF:
    % 7:9 = [vN vE vD]
    s.vN = data(:,7);
    s.vE = data(:,8);
    s.vD = data(:,9);

    s.V3D = sqrt(s.vN.^2 + s.vE.^2 + s.vD.^2);

end

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

function mse = cumulative_mean(x)

    n = (1:length(x)).';
    mse = cumsum(x) ./ n;

end

function labels = build_error_legend(series, idx_ref)

    labels = {};

    for k = 1:numel(series)

        if k ~= idx_ref
            labels{end+1} = series(k).name; %#ok<AGROW>
        end
    end

end

function plot_correction_windows(ax, range_no_corr, show_labels, label_y_value)

    if nargin < 4
        label_y_value = [];
    end

    if isempty(range_no_corr) || size(range_no_corr,2) ~= 2
        return;
    end

    axes(ax); %#ok<LAXES>

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