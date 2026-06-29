%% plot_compare_all.m
% Comparação:
%   Modelo Matemático
%   XPlane
%   DBN

fprintf('\n========== COMPARAÇÃO MODELO x XPLANE x DBN ==========\n');

%% ============================================================
% MODELO
%% ============================================================

xN_model = [];
xE_model = [];
alt_model = [];
t_model = [];

try
    model_data = out.Y.signals.values;
    t_model = out.Y.time;

    xN_model = model_data(:,10);
    xE_model = model_data(:,11);
    alt_model = model_data(:,12);
catch
    warning('Dados do modelo não encontrados.');
end

%% ============================================================
% XPLANE
%% ============================================================

xN_xp = [];
xE_xp = [];
alt_xp = [];
t_xp = [];

try
    xp_data = out.XplaneSimulationData.signals.values;
    t_xp = out.XplaneSimulationData.time;

    xN_xp = xp_data(:,9);
    xE_xp = xp_data(:,10);
    alt_xp = xp_data(:,4);
catch
    warning('Dados do XPlane não encontrados.');
end

%% ============================================================
% DBN
%% ============================================================

xN_dbn = [];
xE_dbn = [];
alt_dbn = [];
vel_dbn = [];
t_dbn = [];

try

    dbn_data = out.BDN_data.signals.values;
    t_dbn = out.BDN_data.time;

    pos_dbn = dbn_data(:,7:9);
    vel_dbn = dbn_data(:,4:6);

    xN_dbn = pos_dbn(:,1);
    xE_dbn = pos_dbn(:,2);
    D_dbn  = pos_dbn(:,3);

    if exist('DBN_params','var')

        altitude0 = DBN_params.altitude0;
        D0 = DBN_params.pos0_ned(3);

        alt_dbn = altitude0 - (D_dbn - D0);

    else

        alt_dbn = -D_dbn;

    end

catch
    warning('Dados do DBN não encontrados.');
end

%% ============================================================
% FIGURA PRINCIPAL
%% ============================================================

figure( ...
    'Name','Comparação Geral', ...
    'Position',[100 50 1500 900]);

%% ============================================================
% 1 - TRAJETÓRIA 3D
%% ============================================================

subplot(3,2,1)

hold on

if ~isempty(xN_model)
    plot3(xE_model,xN_model,alt_model,'b','LineWidth',1.5)
end

if ~isempty(xN_xp)
    plot3(xE_xp,xN_xp,alt_xp,'r','LineWidth',1.5)
end

if ~isempty(xN_dbn)
    plot3(xE_dbn,xN_dbn,alt_dbn,'g','LineWidth',1.5)
end

if exist('WPs','var')
    plot3(WPs(:,2),WPs(:,1),WPs(:,3), ...
        'ks', ...
        'MarkerFaceColor','y');
end

grid on
axis equal

xlabel('Leste [m]')
ylabel('Norte [m]')
zlabel('Altitude [m]')

title('Trajetória 3D')

legend('Modelo','XPlane','DBN','Waypoints')

view(30,25)

%% ============================================================
% 2 - VISTA SUPERIOR
%% ============================================================

subplot(3,2,2)

hold on

if ~isempty(xN_model)
    plot(xE_model,xN_model,'b','LineWidth',1.5)
end

if ~isempty(xN_xp)
    plot(xE_xp,xN_xp,'r','LineWidth',1.5)
end

if ~isempty(xN_dbn)
    plot(xE_dbn,xN_dbn,'g','LineWidth',1.5)
end

if exist('WPs','var')
    plot(WPs(:,2),WPs(:,1), ...
        'ks', ...
        'MarkerFaceColor','y');
end

grid on
axis equal

xlabel('Leste [m]')
ylabel('Norte [m]')

title('Vista Superior')

legend('Modelo','XPlane','DBN')

%% ============================================================
% 3 - ALTITUDE
%% ============================================================

subplot(3,2,3)

hold on

if ~isempty(t_model)
    plot(t_model,alt_model,'b','LineWidth',1.5)
end

if ~isempty(t_xp)
    plot(t_xp,alt_xp,'r','LineWidth',1.5)
end

if ~isempty(t_dbn)
    plot(t_dbn,alt_dbn,'g','LineWidth',1.5)
end

grid on

xlabel('Tempo [s]')
ylabel('Altitude [m]')

title('Altitude')

legend('Modelo','XPlane','DBN')

%% ============================================================
% 4 - VELOCIDADE
%% ============================================================

subplot(3,2,4)

hold on

if ~isempty(t_dbn)

    VT_dbn = sqrt( ...
        vel_dbn(:,1).^2 + ...
        vel_dbn(:,2).^2 + ...
        vel_dbn(:,3).^2 );

    plot(t_dbn,VT_dbn,'g','LineWidth',1.5)

end

try

    VT_xp = xp_data(:,1);

    plot(t_xp,VT_xp,'r','LineWidth',1.5)

catch
end

grid on

xlabel('Tempo [s]')
ylabel('VT [m/s]')

title('Velocidade Total')

legend('DBN','XPlane')

%% ============================================================
% 5 - ERRO DE POSIÇÃO
%% ============================================================

subplot(3,2,5)

if ~isempty(xN_xp) && ~isempty(xN_dbn)

    N = min(length(xN_xp),length(xN_dbn));

    erro_pos = sqrt( ...
        (xN_xp(1:N)-xN_dbn(1:N)).^2 + ...
        (xE_xp(1:N)-xE_dbn(1:N)).^2 );

    plot(t_dbn(1:N),erro_pos,'k','LineWidth',1.5)

    grid on

    xlabel('Tempo [s]')
    ylabel('Erro [m]')

    title('Erro Horizontal DBN vs XPlane')

end

%% ============================================================
% 6 - ERRO DE ALTITUDE
%% ============================================================

subplot(3,2,6)

if ~isempty(alt_xp) && ~isempty(alt_dbn)

    N = min(length(alt_xp),length(alt_dbn));

    erro_alt = alt_xp(1:N) - alt_dbn(1:N);

    plot(t_dbn(1:N),erro_alt,'m','LineWidth',1.5)

    grid on

    xlabel('Tempo [s]')
    ylabel('Erro [m]')

    title('Erro de Altitude DBN vs XPlane')

end

sgtitle('Comparação Modelo Matemático x XPlane x DBN')

fprintf('\n========== FIM DA COMPARAÇÃO ==========\n');