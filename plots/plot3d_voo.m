%% plot3d_voo.m - Plota trajetória 3D da aeronave
% Rodar APÓS a simulação terminar.
% Requer variável POS no workspace (To Workspace com [xN, xE, alt]).
%
% Se POS não existir, tenta extrair de 'out' ou dos estados.
%
% Uso: >> plot3d_voo

fprintf('\n========== PLOT 3D DO VOO ==========\n');
%% ========== Localizar dados de posição ==========
try
    pos_data = out.Y.signals.values;
    t_pos = out.Y.time;
    xN = pos_data(:,10);
    xE = pos_data(:,11);
    alt = pos_data(:,12);
catch
   disp("Error: data not founded to plot")
end
%% ========== Figura 1: Trajetória 3D ==========
figure('Name', 'Trajetória 3D', 'Position', [50 100 800 600]);
plot3(xE, xN, alt, 'b-', 'LineWidth', 1.5);
hold on;

% Waypoints (apenas markers)
if exist('WPs', 'var')
    plot3(WPs(:,2), WPs(:,1), WPs(:,3), 'rs', 'MarkerSize', 10, ...
          'MarkerFaceColor', 'r');
    for i = 1:size(WPs, 1)
        text(WPs(i,2)+5, WPs(i,1)+5, WPs(i,3)+2, sprintf('WP%d', i), ...
             'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
    end
end

% Marcar início e fim
plot3(xE(1), xN(1), alt(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot3(xE(end), xN(end), alt(end), 'kx', 'MarkerSize', 12, 'LineWidth', 2);

xlabel('Leste (m)'); ylabel('Norte (m)'); zlabel('Altitude (m)');
title('Trajetória 3D da Aeronave');
legend('Trajetória', 'Waypoints', 'Início', 'Fim', 'Location', 'best');
grid on; axis equal;
view(30, 25);
hold off;

%% ========== Figura 2: Vista Superior (Ground Track) ==========
figure('Name', 'Vista Superior', 'Position', [900 100 700 600]);
plot(xE, xN, 'b-', 'LineWidth', 1.5);
hold on;
if exist('WPs', 'var')
    plot(WPs(:,2), WPs(:,1), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    for i = 1:size(WPs, 1)
        text(WPs(i,2)+5, WPs(i,1)+5, sprintf('WP%d', i), ...
             'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
    end
    % Círculo de aceitação
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
xlabel('Leste (m)'); ylabel('Norte (m)');
title('Vista Superior (Ground Track)');
grid on; axis equal;
hold off;

%% ========== Figura 3: Altitude vs Tempo ==========
figure('Name', 'Altitude', 'Position', [50 750 800 400]);
if ~isempty(t_pos)
    plot(t_pos, alt, 'b-', 'LineWidth', 1.5);
    xlabel('Tempo (s)');
else
    plot(alt, 'b-', 'LineWidth', 1.5);
    xlabel('Amostra');
end
hold on;
% Linhas de referência dos waypoints
if exist('WPs', 'var')
    alt_wps = unique(WPs(:,3));
    for i = 1:length(alt_wps)
        yline(alt_wps(i), 'r--', sprintf('%.0f m', alt_wps(i)), ...
               'LineWidth', 0.8, 'LabelHorizontalAlignment', 'left');
    end
end
ylabel('Altitude (m)');
title('Altitude ao Longo do Voo');
grid on;
hold off;

%% ========== Estatísticas ==========
fprintf('\n--- Estatísticas do Voo ---\n');
fprintf('  Posição inicial: N=%.1f  E=%.1f  Alt=%.1f m\n', xN(1), xE(1), alt(1));
fprintf('  Posição final:   N=%.1f  E=%.1f  Alt=%.1f m\n', xN(end), xE(end), alt(end));
fprintf('  Altitude: min=%.2f  max=%.2f  média=%.2f m\n', min(alt), max(alt), mean(alt));
fprintf('  Distância total percorrida: %.1f m\n', ...
    sum(sqrt(diff(xN).^2 + diff(xE).^2 + diff(alt).^2)));

if exist('WPs', 'var')
    % Distância ao último WP
    d_final = sqrt((xN(end)-WPs(end,1))^2 + (xE(end)-WPs(end,2))^2);
    fprintf('  Distância ao WP final: %.1f m\n', d_final);
end

fprintf('\n========== FIM DO PLOT 3D ==========\n');
