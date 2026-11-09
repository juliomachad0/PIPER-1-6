function U = hil_serial_step_guiagem(sensors15)
% hil_serial_step_guiagem - Ponte serial PC<->Arduino para HIL com
% guiagem embarcada. Idem hil_serial_step.m, mas na 1a chamada faz
% upload da matriz WPs do base workspace via mensagem WP_UPLOAD.
%
% Protocolo:
%   PC -> MCU  WP_UPLOAD : [0xBB 0x66] uint8 N + N*(4 floats LE)
%   MCU -> PC  WP_ACK    : [0x66 0xBB] uint8 N
%   PC -> MCU  SENSORS   : [0xAA 0x55] + 15 floats LE
%   MCU -> PC  ACTUATORS : [0x55 0xAA] + 4 floats LE
%
% sensors15 inclui xN em s(13) e xE em s(14) (antes zerados).

persistent sp fail_count last_wp R_stop pause_x_plane_simul
global GlobalSocket;
import XPlaneConnect.*;

if isempty(fail_count), fail_count = 0; end
if isempty(pause_x_plane_simul), pause_x_plane_simul = false; end
MAX_FAILS = 5;

if isempty(sp) || ~isvalid(sp)
    try
        try
            hil_port_guiagem_ref = evalin('base', 'HIL_MCUport');
        catch
            error('hil_serial_step_guiagem:noPortVar', ...
                'Variavel HIL_MCUport nao encontrada no base workspace.');
        end
        sp = serialport(hil_port_guiagem_ref, 115200, 'Timeout', 1);
    catch ME
        error('hil_serial_step_guiagem:noPort', ...
            'Microcontrolador nao encontrado em %s (%s).', hil_port_guiagem_ref, ME.message);
    end
    pause(3);           % DTR reset
    flush(sp);
    fail_count = 0;

    % --- Upload de waypoints ---
    try
        WPs_local = evalin('base', 'WPs');
    catch
        error('hil_serial_step_guiagem:noWPs', ...
            'Variavel WPs nao encontrada no base workspace.');
    end
    N = size(WPs_local, 1);
    if N < 1 || N > 32
        error('hil_serial_step_guiagem:badN', ...
            'WPs deve ter entre 1 e 32 linhas (tem %d).', N);
    end
    write(sp, uint8([187 102]), 'uint8');     % 0xBB 0x66
    write(sp, uint8(N), 'uint8');
    write(sp, single(reshape(WPs_local(:,1:4)', 1, [])), 'single');
    ack = read(sp, 3, 'uint8');
    if numel(ack) ~= 3 || ack(1) ~= 102 || ack(2) ~= 187 || ack(3) ~= N
        try, delete(sp); catch, end
        sp = [];
        error('hil_serial_step_guiagem:wpAck', ...
            'Falha no ACK do upload de WPs (recebido: %s).', mat2str(ack));
    end
    fprintf('hil_serial_step_guiagem: %d waypoints enviados ao MCU.\n', N);
    last_wp = WPs_local(end, 1:2);   % [N, E] do ultimo WP
    R_stop  = 80;                     % mesmo R_accept do firmware
    pause_x_plane_simul = false;
end

% --- Se o ultimo WP ja foi alcancado antes, mantem pausado e sai imediatamente ---
if pause_x_plane_simul
    U = [0.491387; 0.015456; 0; 0];
    return;
end

% --- Checa se chegou na zona do ultimo WP (executa apenas 1 vez) ---
xN_now = sensors15(13);
xE_now = sensors15(14);
if ~isempty(last_wp)
    dN = last_wp(1) - xN_now;
    dE = last_wp(2) - xE_now;
    if sqrt(dN*dN + dE*dE) <= R_stop
        pause_x_plane_simul = true;
        fprintf('hil_serial_step_guiagem: ultimo WP alcancado, pausando XPlane simulation.\n');
        try
            if isempty(GlobalSocket)
                GlobalSocket = openUDP('127.0.0.1', 49009);
            end
            pauseSim(1, GlobalSocket);
            fprintf('Xplane: Simulacao Pausada.\n');
        catch ME
            disp(['pausar xplane: falha ao conectar - ' ME.message]);
        end
        U = [0.491387; 0.015456; 0; 0];
        return;
    end
end

ok = false;
try
    write(sp, uint8([170 85]), 'uint8');
    write(sp, single(sensors15(:)'), 'single');
    buf = read(sp, 18, 'uint8');
    if numel(buf) == 18 && buf(1) == 85 && buf(2) == 170
        U = double(typecast(uint8(buf(3:18)), 'single')');
        ok = true;
    end
catch
    ok = false;
end

if ~ok
    fail_count = fail_count + 1;
    warning('hil_serial_step_guiagem: sem resposta do Microcontrolador (%d/%d)', fail_count, MAX_FAILS);
    if fail_count >= MAX_FAILS
        try, delete(sp); catch, end
        sp = [];
        error('hil_serial_step_guiagem:lostLink', ...
            'Microcontrolador perdeu conexao (%d passos sem resposta). Abortando.', MAX_FAILS);
    end
    U = [0.491387; 0.015456; 0; 0];
else
    fail_count = 0;
end
end
