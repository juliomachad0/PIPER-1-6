%% plot_compare_all4.m
% Comparação de angulos de Euler e MSE angular:
%   Roll, Pitch, Yaw
%   MSE Roll, MSE Pitch, MSE Yaw

fprintf('\n========== COMPARAÇÃO ALL4: EULER E MSE ANGULAR ==========\n');

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
    'phi', {}, ...
    'theta', {}, ...
    'psi', {}, ...
    'color', {} ...
);

%% XPlane referência
if use_xplane_ref
    try
        [data, t] = get_out_signal(out_local, {'XplaneSimulationData'});

        s = new_series('XPlane', 'xplane', colors.xplane);

        s.t = t;

        % XPlane:
        % phi   -> coluna 5
        % theta -> coluna 2
        % psi   -> coluna 7
        s.phi   = data(:,5);
        s.theta = data(:,2);
        s.psi   = data(:,7);

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

figure('Name','Comparação ALL4 - Euler e MSE Angular', ...
       'Position',[80 40 1650 950]);

%% 1 - Roll

subplot(3,2,1)
hold on

for k = 1:numel(series)
    plot(series(k).t, rad2deg(series(k).phi), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('\phi roll [deg]')
title('Roll')
legend({series.name}, 'Location','best')
hold off

%% 2 - MSE Roll

subplot(3,2,2)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, ref_i, y_i] = align_by_time(ref.t, ref.phi, series(k).t, series(k).phi);

    err = wrapToPi_local(ref_i - y_i);
    mse_roll = cumulative_mean(err.^2);

    plot(tc, rad2deg(sqrt(mse_roll)), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, true);

grid on
xlabel('Tempo [s]')
ylabel('RMSE roll [deg]')
title('RMSE Roll Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 3 - Pitch

subplot(3,2,3)
hold on

for k = 1:numel(series)
    plot(series(k).t, rad2deg(series(k).theta), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('\theta pitch [deg]')
title('Pitch')
legend({series.name}, 'Location','best')
hold off

%% 4 - MSE Pitch

subplot(3,2,4)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, ref_i, y_i] = align_by_time(ref.t, ref.theta, series(k).t, series(k).theta);

    err = wrapToPi_local(ref_i - y_i);
    mse_pitch = cumulative_mean(err.^2);

    plot(tc, rad2deg(sqrt(mse_pitch)), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('RMSE pitch [deg]')
title('RMSE Pitch Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 5 - Yaw

subplot(3,2,5)
hold on

for k = 1:numel(series)
    plot(series(k).t, rad2deg(series(k).psi), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('\psi yaw [deg]')
title('Yaw')
legend({series.name}, 'Location','best')
hold off

%% 6 - MSE Yaw

subplot(3,2,6)
hold on

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [tc, ref_i, y_i] = align_by_time(ref.t, ref.psi, series(k).t, series(k).psi);

    err = wrapToPi_local(ref_i - y_i);
    mse_yaw = cumulative_mean(err.^2);

    plot(tc, rad2deg(sqrt(mse_yaw)), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

plot_correction_windows(gca, range_time_without_correction_local, false);

grid on
xlabel('Tempo [s]')
ylabel('RMSE yaw [deg]')
title('RMSE Yaw Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

annotation('textbox', [0 0.955 1 0.035], ...
    'String', 'Euler e RMSE Angular - XPlane x Estimadores', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'FontWeight', 'bold', ...
    'FontSize', 14, ...
    'EdgeColor', 'none');

%% ===================== ESTATÍSTICAS =====================

fprintf('\n--- Métricas finais angulares vs XPlane ---\n');

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [~, phi_ref, phi_k]     = align_by_time(ref.t, ref.phi,   series(k).t, series(k).phi);
    [~, theta_ref, theta_k] = align_by_time(ref.t, ref.theta, series(k).t, series(k).theta);
    [~, psi_ref, psi_k]     = align_by_time(ref.t, ref.psi,   series(k).t, series(k).psi);

    err_phi   = wrapToPi_local(phi_ref - phi_k);
    err_theta = wrapToPi_local(theta_ref - theta_k);
    err_psi   = wrapToPi_local(psi_ref - psi_k);

    rmse_phi   = rad2deg(sqrt(mean(err_phi.^2)));
    rmse_theta = rad2deg(sqrt(mean(err_theta.^2)));
    rmse_psi   = rad2deg(sqrt(mean(err_psi.^2)));

    fprintf('%-12s | RMSE roll=%.3f deg | RMSE pitch=%.3f deg | RMSE yaw=%.3f deg\n', ...
        series(k).name, rmse_phi, rmse_theta, rmse_psi);
end

fprintf('\n========== FIM DO PLOT_COMPARE_ALL4 ==========\n');

%% ========================================================================
% FUNÇÕES LOCAIS
%% ========================================================================

function s = new_series(name, key, color)

    s.name = name;
    s.key = key;
    s.t = [];
    s.phi = [];
    s.theta = [];
    s.psi = [];
    s.color = color;

end

function s = read_dbn_format(data, t, name, key, color)

    s = new_series(name, key, color);

    s.t = t;

    % DBN:
    % 1:3 = [phi theta psi]
    s.phi   = data(:,1);
    s.theta = data(:,2);
    s.psi   = data(:,3);

end

function s = read_ekf_format(data, t, name, key, color)

    s = new_series(name, key, color);

    s.t = t;

    % EKF:
    % 4:6 = [phi theta psi]
    s.phi   = data(:,4);
    s.theta = data(:,5);
    s.psi   = data(:,6);

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

function ang = wrapToPi_local(ang)

    ang = mod(ang + pi, 2*pi) - pi;

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