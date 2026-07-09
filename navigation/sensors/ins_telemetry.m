function ins_telemetry(time, h, Vt, V3D, V3D_EKF, cmds, euler)
%INS_TELEMETRY Telemetria textual limitada por intervalo de tempo.
%
% Entradas:
%   time     : tempo usado para telemetria, preferencialmente t_nav ou t_xplane_rel [s]
%   h        : altitude [m]
%   Vt       : velocidade aerodinamica [m/s]
%   V3D      : velocidade X-Plane/NED [vN; vE; vD] ou equivalente
%   V3D_EKF  : velocidade estimada EKF [vN; vE; vD]
%   cmds     : comandos [thr; elev; ail; rud]
%   euler    : [phi_x; theta_x; psi_x; phi_ekf; theta_ekf; psi_ekf]

persistent last_print_time

if isempty(last_print_time)
    last_print_time = -inf;
end

%% Configuracoes do workspace

try
    activate_telemetry_f = evalin('base', 'activate_telemetry');
catch
    activate_telemetry_f = false;
end

try
    consider_FK_in_telemetry_f = evalin('base', 'consider_FK_in_telemetry');
catch
    consider_FK_in_telemetry_f = false;
end

try
    ins_telemetry_time_interval_f = evalin('base', 'ins_telemetry_time_interval');
catch
    ins_telemetry_time_interval_f = 1.0;
end

%% Validacoes basicas

if ~activate_telemetry_f
    return;
end

if ~isfinite(time)
    return;
end

if ins_telemetry_time_interval_f <= 0
    ins_telemetry_time_interval_f = 1.0;
end

%% Controle de intervalo de impressao

if time < last_print_time
    % Caso a simulacao reinicie
    last_print_time = -inf;
end

if (time - last_print_time) < ins_telemetry_time_interval_f
    return;
end

last_print_time = time;

%% Garantir vetores coluna

V3D = V3D(:);
V3D_EKF = V3D_EKF(:);
cmds = cmds(:);
euler = euler(:);

%% Protecao contra tamanhos inesperados

if numel(V3D) < 3
    V3D = [V3D; zeros(3 - numel(V3D), 1)];
end

if numel(V3D_EKF) < 3
    V3D_EKF = [V3D_EKF; zeros(3 - numel(V3D_EKF), 1)];
end

if numel(cmds) < 4
    cmds = [cmds; zeros(4 - numel(cmds), 1)];
end

if numel(euler) < 6
    euler = [euler; zeros(6 - numel(euler), 1)];
end

%% Velocidades

Vx = V3D(1);
Vy = V3D(2);
Vz = V3D(3);

Vmod = sqrt(Vx^2 + Vy^2 + Vz^2);

Vxfk = V3D_EKF(1);
Vyfk = V3D_EKF(2);
Vzfk = V3D_EKF(3);

Vmod_fk = sqrt(Vxfk^2 + Vyfk^2 + Vzfk^2);

%% Comandos

thr  = cmds(1);
elev = cmds(2);
ail  = cmds(3);
rud  = cmds(4);

%% Euler

phix   = euler(1);
thetax = euler(2);
psix   = euler(3);

phie   = euler(4);
thetae = euler(5);
psie   = euler(6);

%% Impressao

if consider_FK_in_telemetry_f

    fprintf(['\nt: %.2f s | h: %.2f m | VT: %.2f m/s | VXP: %.2f m/s | ' ...
             'VEKF: %.2f m/s | ' ...
             'Thr: %.2f | Ele: %.2f | Ail: %.2f | Rud: %.2f | ' ...
             'phiX: %.2f deg | thetaX: %.2f deg | psiX: %.2f deg | ' ...
             'phiEKF: %.2f deg | thetaEKF: %.2f deg | psiEKF: %.2f deg\n'], ...
             time, h, Vt, Vmod, Vmod_fk, ...
             thr, elev, ail, rud, ...
             rad2deg(phix), rad2deg(thetax), rad2deg(psix), ...
             rad2deg(phie), rad2deg(thetae), rad2deg(psie));

else

    fprintf(['\nt: %.2f s | h: %.2f m | VT: %.2f m/s | VXP_3D: %.2f m/s | ' ...
             'Thr: %.2f | Ele: %.2f | Ail: %.2f | Rud: %.2f | ' ...
             'phiX: %.2f deg | thetaX: %.2f deg | psiX: %.2f deg\n'], ...
             time, h, Vt, Vmod, ...
             thr, elev, ail, rud, ...
             rad2deg(phix), rad2deg(thetax), rad2deg(psix));

end

end