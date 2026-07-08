%% plot_compare_all2.m
% Comparação de Euler e MSE de trajetória:
%   XPlane referência
%   EKF Direto sem erro
%   EKF Direto com erro
%   EKF Indireto sem erro
%   EKF Indireto com erro

fprintf('\n========== COMPARAÇÃO EULER + MSE DE TRAJETÓRIA ==========\n');

%% ===================== ESCOLHA DOS MODELOS =====================

use_xplane_ref  = true;

use_ekf_di      = true;
use_ekf_di_em   = false;

use_ekf_indi    = true;
use_ekf_indi_em = false;

%% ===================== CORES =====================

colors.xplane      = [0.000 0.500 0.000];  % verde escuro
colors.ekf_di      = [0.000 0.000 1.000];  % azul
colors.ekf_di_em   = [0.500 0.000 0.500];  % roxo
colors.ekf_indi    = [1.000 0.000 0.000];  % vermelho
colors.ekf_indi_em = [0.500 0.250 0.000];  % marrom

lineWidth = 1.4;

%% ===================== CARREGAR SÉRIES =====================

series = struct( ...
    'name', {}, ...
    'key', {}, ...
    't', {}, ...
    'N', {}, ...
    'E', {}, ...
    'alt', {}, ...
    'phi', {}, ...
    'theta', {}, ...
    'psi', {}, ...
    'color', {} ...
);

%% XPlane referência
if use_xplane_ref
    try
        data = out.XplaneSimulationData.signals.values;
        t = out.XplaneSimulationData.time;

        s = new_series('XPlane', 'xplane', colors.xplane);

        s.t = t;

        % XPlane:
        % phi   -> coluna 5
        % theta -> coluna 2
        % psi   -> coluna 7
        s.phi   = data(:,5);
        s.theta = data(:,2);
        s.psi   = data(:,7);

        % posição:
        % altitude -> coluna 4
        % Norte    -> coluna 9
        % Leste    -> coluna 10
        s.alt = data(:,4);
        s.N   = data(:,9);
        s.E   = data(:,10);

        series(end+1) = s;
    catch
        warning('XPlane out.XplaneSimulationData não encontrado.');
    end
end

%% EKF Direto sem erro
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

%% EKF Direto com erro
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

%% EKF Indireto sem erro
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

%% EKF Indireto com erro
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
    error('XPlane precisa estar carregado como referência.');
end

ref = series(idx_ref);

%% ===================== FIGURA =====================

figure('Name','Euler e MSE de Trajetória', 'Position',[80 40 1650 950]);

%% 1 - Roll

subplot(3,2,1)
hold on

for k = 1:numel(series)
    plot(series(k).t, rad2deg(series(k).phi), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

grid on
xlabel('Tempo [s]')
ylabel('\phi roll [deg]')
title('Roll')
legend({series.name}, 'Location','best')
hold off

%% 2 - Pitch

subplot(3,2,3)
hold on

for k = 1:numel(series)
    plot(series(k).t, rad2deg(series(k).theta), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

grid on
xlabel('Tempo [s]')
ylabel('\theta pitch [deg]')
title('Pitch')
legend({series.name}, 'Location','best')
hold off

%% 3 - Yaw

subplot(3,2,5)
hold on

for k = 1:numel(series)
    plot(series(k).t, rad2deg(series(k).psi), ...
        'Color', series(k).color, 'LineWidth', lineWidth);
end

grid on
xlabel('Tempo [s]')
ylabel('\psi yaw [deg]')
title('Yaw')
legend({series.name}, 'Location','best')
hold off

%% 4 - MSE horizontal acumulado

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

grid on
xlabel('Tempo [s]')
ylabel('MSE horizontal [m²]')
title('MSE Horizontal Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 5 - MSE altitude acumulado

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

grid on
xlabel('Tempo [s]')
ylabel('MSE altitude [m²]')
title('MSE de Altitude Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

%% 6 - MSE 3D acumulado

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

grid on
xlabel('Tempo [s]')
ylabel('MSE 3D [m²]')
title('MSE 3D Acumulado vs XPlane')
legend(build_error_legend(series, idx_ref), 'Location','best')
hold off

sgtitle('Euler e Erro Médio Quadrático - XPlane x EKFs')

%% ===================== ESTATÍSTICAS =====================

fprintf('\n--- Métricas finais vs XPlane ---\n');

for k = 1:numel(series)

    if k == idx_ref
        continue;
    end

    [~, Nref, Nk]       = align_by_time(ref.t, ref.N,   series(k).t, series(k).N);
    [~, Eref, Ek]       = align_by_time(ref.t, ref.E,   series(k).t, series(k).E);
    [~, altref, altk]   = align_by_time(ref.t, ref.alt, series(k).t, series(k).alt);

    e_h2   = (Nref - Nk).^2 + (Eref - Ek).^2;
    e_alt2 = (altref - altk).^2;
    e_3d2  = e_h2 + e_alt2;

    mse_h_final   = mean(e_h2);
    mse_alt_final = mean(e_alt2);
    mse_3d_final  = mean(e_3d2);

    rmse_h_final   = sqrt(mse_h_final);
    rmse_alt_final = sqrt(mse_alt_final);
    rmse_3d_final  = sqrt(mse_3d_final);

    fprintf('%-12s | MSE H=%.3f m² | RMSE H=%.3f m | MSE Alt=%.3f m² | RMSE Alt=%.3f m | MSE 3D=%.3f m² | RMSE 3D=%.3f m\n', ...
        series(k).name, ...
        mse_h_final, rmse_h_final, ...
        mse_alt_final, rmse_alt_final, ...
        mse_3d_final, rmse_3d_final);
end

fprintf('\n========== FIM DO PLOT_COMPARE_ALL2 ==========\n');

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
    s.phi = [];
    s.theta = [];
    s.psi = [];
    s.color = color;

end

function s = read_ekf_format(data, t, name, key, color)

    s = new_series(name, key, color);

    s.t = t;

    % EKF:
    % 1:3    pos_out   = [N E altitude]
    % 4:6    euler_out = [phi theta psi]
    % 7:9    vel_out
    % 10:12  acc_n_out
    % 13:33  xhat_out

    s.N   = data(:,1);
    s.E   = data(:,2);
    s.alt = data(:,3);

    s.phi   = data(:,4);
    s.theta = data(:,5);
    s.psi   = data(:,6);

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
