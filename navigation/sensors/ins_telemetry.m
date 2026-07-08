function ins_telemetry(time, h, Vt, V3D, V3D_EKF, cmds, euler)

persistent last_print_time

if isempty(last_print_time)
    last_print_time = -inf;
end

%% Lê configurações do workspace

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
    ins_telemetry_time_interval_f = 0;
end

%% Se telemetria estiver desativada, retorna

if ~activate_telemetry_f
    return;
end

%% Controle de intervalo de impressão

if time < last_print_time
    % caso a simulação reinicie
    last_print_time = -inf;
end

if ins_telemetry_time_interval_f > 0
    if (time - last_print_time) < ins_telemetry_time_interval_f
        return;
    end
end

last_print_time = time;

%% Velocidades

Vx = V3D(1);
Vy = V3D(2);
Vz = V3D(3);

Vmod = sqrt(Vx^2 + Vy^2 + Vz^2); % módulo da velocidade XPlane/NED

Vxfk = V3D_EKF(1);
Vyfk = V3D_EKF(2);
Vzfk = V3D_EKF(3);

Vmod_fk = sqrt(Vxfk^2 + Vyfk^2 + Vzfk^2); % módulo da velocidade FK/EKF

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

%% Impressão

if consider_FK_in_telemetry_f

    fprintf(['\nt: %.2f s | h: %.2f m | VT: %.2f m/s | VXP: %.2f m/s | ' ...
             'VFK: %.2f m/s | ' ...
             'Thr: %.2f | Ele: %.2f | Ail: %.2f | Rud: %.2f | ' ...
             'φX(deg): %.2f | θX: %.2f | ψX: %.2f | ' ...
             'φFK: %.2f | θFK: %.2f | ψFK: %.2f '], ...
             time, h, Vt, Vmod, Vmod_fk, ...
             thr, elev, ail, rud, ...
             rad2deg(phix), rad2deg(thetax), rad2deg(psix), ...
             rad2deg(phie), rad2deg(thetae), rad2deg(psie));

else

    fprintf(['\nt: %.2f s | h: %.2f m | VT: %.2f m/s | VXP_3D: %.2f m/s | ' ...
             'Thr: %.2f | Ele: %.2f | Ail: %.2f | Rud: %.2f | ' ...
             'φX: %.2f deg | θX: %.2f deg | ψX: %.2f deg'], ...
             time, h, Vt, Vmod, ...
             thr, elev, ail, rud, ...
             rad2deg(phix), rad2deg(thetax), rad2deg(psix));
end

end