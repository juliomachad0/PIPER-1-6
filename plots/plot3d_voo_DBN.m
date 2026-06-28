%% plot3d_voo_dbn.m - Plota trajetória 3D calculada pelo DBN
% Rodar APÓS a simulação terminar.
% Requer variável 'out' no workspace, contendo out.BDN_data.
%
% Formato esperado de out.BDN_data.signals.values:
%   colunas  1:3   -> Euler DBN [phi, theta, psi] [rad]
%   colunas  4:6   -> Velocidade DBN em NED [vN, vE, vD] [m/s]
%   colunas  7:9   -> Posição DBN em NED [N, E, D] [m]
%   colunas 10:12  -> Aceleração DBN em NED [aN, aE, aD] [m/s²]
%   colunas 13:16  -> Quaternion Farrel [b1, b2, b3, b4]
%
% Uso:
%   >> plot3d_voo_dbn

fprintf('\n========== PLOT 3D DO VOO A PARTIR DE DADOS DO DBN ==========' );
fprintf('\n');

%% ========== Localizar dados do DBN ==========
try
    dbn_data = out.BDN_data.signals.values;
    t_dbn = out.BDN_data.time;
catch
    error('Erro: dados out.BDN_data não encontrados para plotagem do DBN.');
end

%% ========== Separar sinais ==========
euler_dbn = dbn_data(:,1:3);        % [phi theta psi] [rad]
vel_dbn   = dbn_data(:,4:6);        % [vN vE vD] [m/s]
pos_dbn   = dbn_data(:,7:9);        % [N E D] [m]
acc_dbn   = dbn_data(:,10:12);      % [aN aE aD] [m/s²]

if size(dbn_data,2) >= 16
    qb_dbn = dbn_data(:,13:16);     % % [b1 b2 b3 b4]
end

xN = pos_dbn(:,1);
xE = pos_dbn(:,2);
D  = pos_dbn(:,3);                  % NED: Down positivo para baixo

%% ========== Converter Down NED para altitude positiva para cima ==========
% Preferência: usar DBN_params, pois contém a condição inicial aplicada após
% reposicionar o X-Plane.

if exist('DBN_params', 'var') && isfield(DBN_params, 'altitude0') && isfield(DBN_params, 'pos0_ned')
    altitude0 = DBN_params.altitude0;
    D0 = DBN_params.pos0_ned(3);
    alt = altitude0 - (D - D0);
else
    % Se a posição DBN já estiver em NED absoluto, altitude = -D.
    alt = -D;
end

%% ========== Figura 1: Trajetória 3D ==========
figure('Name', 'Trajetória 3D DBN', 'Position', [50 100 800 600]);
plot3(xE, xN, alt, 'b-', 'LineWidth', 1.5);
hold on;

% Waypoints
if exist('WPs', 'var')
    plot3(WPs(:,2), WPs(:,1), WPs(:,3), 'rs', 'MarkerSize', 10, ...
          'MarkerFaceColor', 'r');
    for i = 1:size(WPs, 1)
        text(WPs(i,2)+5, WPs(i,1)+5, WPs(i,3)+2, sprintf('WP%d', i), ...
             'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
    end
end

% Início e fim
plot3(xE(1), xN(1), alt(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot3(xE(end), xN(end), alt(end), 'kx', 'MarkerSize', 12, 'LineWidth', 2);

xlabel('Leste (m)');
ylabel('Norte (m)');
zlabel('Altitude (m)');
title('Trajetória 3D da Aeronave - DBN');
legend('Trajetória DBN', 'Waypoints', 'Início', 'Fim', 'Location', 'best');
grid on;
axis equal;
view(30, 25);
hold off;

%% ========== Figura 2: Vista Superior ==========
figure('Name', 'Vista Superior DBN', 'Position', [900 100 700 600]);
plot(xE, xN, 'b-', 'LineWidth', 1.5);
hold on;

if exist('WPs', 'var')
    plot(WPs(:,2), WPs(:,1), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    for i = 1:size(WPs, 1)
        text(WPs(i,2)+5, WPs(i,1)+5, sprintf('WP%d', i), ...
             'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
    end

    if exist('R_accept', 'var')
        th = linspace(0, 2*pi, 100);
        for i = 1:size(WPs, 1)
            plot(WPs(i,2) + R_accept*cos(th), WPs(i,1) + R_accept*sin(th), ...
                 'r--', 'LineWidth', 0.5);
        end
    end
end

plot(xE(1), xN(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(xE(end), xN(end), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('Leste (m)');
ylabel('Norte (m)');
title('Vista Superior - DBN');
grid on;
axis equal;
hold off;

%% ========== Figura 3: Altitude vs Tempo ==========
figure('Name', 'Altitude - DBN', 'Position', [50 750 800 400]);
if ~isempty(t_dbn)
    plot(t_dbn, alt, 'b-', 'LineWidth', 1.5);
    xlabel('Tempo (s)');
else
    plot(alt, 'b-', 'LineWidth', 1.5);
    xlabel('Amostra');
end
hold on;

if exist('WPs', 'var')
    alt_wps = unique(WPs(:,3));
    for i = 1:length(alt_wps)
        yline(alt_wps(i), 'r--', sprintf('%.0f m', alt_wps(i)), ...
               'LineWidth', 0.8, 'LabelHorizontalAlignment', 'left');
    end
end

ylabel('Altitude (m)');
title('Altitude ao Longo do Voo - DBN');
grid on;
hold off;

%% ========== Figura 4: Ângulos de Euler ==========
figure('Name', 'Euler - DBN', 'Position', [900 750 800 400]);
if ~isempty(t_dbn)
    plot(t_dbn, rad2deg(euler_dbn(:,1)), 'r-', 'LineWidth', 1.2); hold on;
    plot(t_dbn, rad2deg(euler_dbn(:,2)), 'g-', 'LineWidth', 1.2);
    plot(t_dbn, rad2deg(euler_dbn(:,3)), 'b-', 'LineWidth', 1.2);
    xlabel('Tempo (s)');
else
    plot(rad2deg(euler_dbn(:,1)), 'r-', 'LineWidth', 1.2); hold on;
    plot(rad2deg(euler_dbn(:,2)), 'g-', 'LineWidth', 1.2);
    plot(rad2deg(euler_dbn(:,3)), 'b-', 'LineWidth', 1.2);
    xlabel('Amostra');
end
grid on;
ylabel('Euler (graus)');
legend('\phi roll', '\theta pitch', '\psi yaw', 'Location', 'best');
title('Ângulos de Euler - DBN');
hold off;

%% ========== Figura 5: Velocidades NED ==========
figure('Name', 'Velocidades NED - DBN', 'Position', [50 1200 800 400]);
if ~isempty(t_dbn)
    plot(t_dbn, vel_dbn(:,1), 'r-', 'LineWidth', 1.2); hold on;
    plot(t_dbn, vel_dbn(:,2), 'g-', 'LineWidth', 1.2);
    plot(t_dbn, vel_dbn(:,3), 'b-', 'LineWidth', 1.2);
    xlabel('Tempo (s)');
else
    plot(vel_dbn(:,1), 'r-', 'LineWidth', 1.2); hold on;
    plot(vel_dbn(:,2), 'g-', 'LineWidth', 1.2);
    plot(vel_dbn(:,3), 'b-', 'LineWidth', 1.2);
    xlabel('Amostra');
end
grid on;
ylabel('Velocidade NED (m/s)');
legend('v_N', 'v_E', 'v_D', 'Location', 'best');
title('Velocidades em NED - DBN');
hold off;

%% ========== Figura 6: Acelerações NED ==========
figure('Name', 'Acelerações NED - DBN', 'Position', [900 1200 800 400]);
if ~isempty(t_dbn)
    plot(t_dbn, acc_dbn(:,1), 'r-', 'LineWidth', 1.2); hold on;
    plot(t_dbn, acc_dbn(:,2), 'g-', 'LineWidth', 1.2);
    plot(t_dbn, acc_dbn(:,3), 'b-', 'LineWidth', 1.2);
    xlabel('Tempo (s)');
else
    plot(acc_dbn(:,1), 'r-', 'LineWidth', 1.2); hold on;
    plot(acc_dbn(:,2), 'g-', 'LineWidth', 1.2);
    plot(acc_dbn(:,3), 'b-', 'LineWidth', 1.2);
    xlabel('Amostra');
end
grid on;
ylabel('Aceleração NED (m/s²)');
legend('a_N', 'a_E', 'a_D', 'Location', 'best');
title('Acelerações em NED - DBN');
hold off;

%% ========== Estatísticas ==========
fprintf('\n--- Estatísticas do Voo - DADOS DO DBN ---\n');
fprintf('  Posição inicial: N=%.1f  E=%.1f  Alt=%.1f m\n', xN(1), xE(1), alt(1));
fprintf('  Posição final:   N=%.1f  E=%.1f  Alt=%.1f m\n', xN(end), xE(end), alt(end));
fprintf('  Altitude: min=%.2f  max=%.2f  média=%.2f m\n', min(alt), max(alt), mean(alt));
fprintf('  Distância total percorrida: %.1f m\n', ...
    sum(sqrt(diff(xN).^2 + diff(xE).^2 + diff(alt).^2)));

if exist('WPs', 'var')
    d_final = sqrt((xN(end)-WPs(end,1))^2 + (xE(end)-WPs(end,2))^2);
    fprintf('  Distância ao WP final: %.1f m\n', d_final);
end

fprintf('\n========== FIM DO PLOT 3D DBN ==========' );
fprintf('\n');
