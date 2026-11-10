function hil_serial_udp(t)
% HIL_SERIAL_UDP - Gerenciador HIL via Conversor Python Serial <-> UDP
%
% Este script e chamado pelo bloco MATLAB Function 'hil_io(t)' no modelo
% HIL_XPlaneMCU.slx.
%
% Como o conversor Python troca dados diretamente entre X-Plane e MCU
% (sensores via UDP->Serial e atuadores via Serial->UDP), o MATLAB/Simulink
% atua exclusivamente como GERENCIADOR da simulação:
%
% 1) No inicio (t == 0 ou 1a chamada):
%    - Pausa a simulacao no X-Plane
%    - Abre a porta serial do MCU, faz upload da matriz WPs e fecha a porta
%    - Atualiza config_converter.json com a porta e o tempo de duracao
%    - Dispara o conversor Python em segundo plano
%    - Despausa a simulacao no X-Plane
% 2) Durante o voo (t > 0):
%    - Monitora periodicamente o status do conversor Python via JSON
%    - Quando o conversor terminar ou o tempo t atingir a duracao configurada,
%      pausa o X-Plane e para a simulacao do Simulink para abrir os plots.

    global GlobalSocket;
    import XPlaneConnect.*;

    persistent is_initialized t_start_wall HIL_dur status_file_path json_config_path t_last_status_check

    %% ====================================================================
    %% 1) PRIMEIRA EXECUCAO (INICIALIZACAO)
    %% ====================================================================
    if isempty(is_initialized) || t <= 0.001
        fprintf('\n======================================================\n');
        fprintf('[HIL_PythonBridge] Inicializando ponte HIL (Gerenciador Simulink)...\n');
        fprintf('======================================================\n');

        % Localizar caminhos
        thisDir = fileparts(mfilename('fullpath'));
        convDir = fullfile(thisDir, 'SerialUDPConversor');
        json_config_path = fullfile(convDir, 'config_converter.json');
        status_file_path = fullfile(convDir, 'converter_status.json');
        py_script_path   = fullfile(convDir, 'serial_udp_converter.py');

        % Remover arquivo de status antigo se houver
        if exist(status_file_path, 'file')
            try, delete(status_file_path); catch, end
        end

        % A) Conectar ao X-Plane e pausar a simulacao
        if isempty(GlobalSocket)
            try
                GlobalSocket = openUDP('127.0.0.1', 49009);
            catch ME
                warning('hil_serial_udp:xpConn', 'Falha ao abrir socket X-Plane: %s', ME.message);
            end
        end

        try
            pauseSim(1, GlobalSocket);
            fprintf('[HIL_PythonBridge] Simulacao no X-Plane pausada para preparacao.\n');
        catch ME
            fprintf('[HIL_PythonBridge] Aviso ao pausar X-Plane: %s\n', ME.message);
        end

        % B) Obter variaveis do workspace base
        try
            port_mcu = evalin('base', 'HIL_MCUport');
        catch
            port_mcu = 'COM9';
        end

        try
            HIL_dur = evalin('base', 'HIL_duration');
        catch
            HIL_dur = 80.0;
        end

        try
            WPs_local = evalin('base', 'WPs');
        catch
            error('hil_serial_udp:noWPs', 'Variavel WPs nao encontrada no workspace base.');
        end

        % C) Upload de Waypoints para o MCU via serial e fechamento imediato da porta
        fprintf('[HIL_PythonBridge] Abrindo porta %s para upload de %d Waypoints...\n', port_mcu, size(WPs_local,1));
        try
            sp = serialport(port_mcu, 115200, 'Timeout', 1);
            pause(2.5); % Tempo para reset do DTR inicial
            flush(sp);

            N = size(WPs_local, 1);
            if N < 1 || N > 32
                error('hil_serial_udp:badN', 'WPs deve ter entre 1 e 32 pontos (tem %d).', N);
            end

            % Enviar cabecalho [0xBB 0x66] + N + WPs
            write(sp, uint8([187 102]), 'uint8');
            write(sp, uint8(N), 'uint8');
            write(sp, single(reshape(WPs_local(:,1:4)', 1, [])), 'single');

            % Aguardar ACK [0x66 0xBB N]
            ack = read(sp, 3, 'uint8');
            if numel(ack) ~= 3 || ack(1) ~= 102 || ack(2) ~= 187 || ack(3) ~= N
                delete(sp);
                error('hil_serial_udp:wpAck', 'Falha no ACK de Waypoints do MCU (recebido: %s).', mat2str(ack));
            end
            fprintf('[HIL_PythonBridge] %d Waypoints gravados com sucesso no MCU!\n', N);

            % Fecha a porta serial para libera-la para o Conversor Python
            delete(sp);
            clear sp;
            pause(0.5);
        catch ME
            if exist('sp', 'var') && isvalid(sp)
                delete(sp);
            end
            error('hil_serial_udp:serialUpload', 'Erro durante upload de WPs na porta %s: %s', port_mcu, ME.message);
        end

        % D) Atualizar arquivo de configuracao .json com porta e duracao
        try
            if exist(json_config_path, 'file')
                txt = fileread(json_config_path);
                cfg_data = jsondecode(txt);
                cfg_data.config.duration_sec = HIL_dur;
                if isfield(cfg_data.config, 'serial_ports') && ~isempty(cfg_data.config.serial_ports)
                    cfg_data.config.serial_ports(1).name = port_mcu;
                end
                if isfield(cfg_data, 'xplane_to_mcu') && ~isempty(cfg_data.xplane_to_mcu)
                    cfg_data.xplane_to_mcu(1).port = port_mcu;
                end
                if isfield(cfg_data, 'mcu_to_xplane') && ~isempty(cfg_data.mcu_to_xplane)
                    cfg_data.mcu_to_xplane(1).port = port_mcu;
                end
                fid = fopen(json_config_path, 'w');
                if fid > 0
                    fwrite(fid, jsonencode(cfg_data, 'PrettyPrint', true));
                    fclose(fid);
                end
            end
        catch ME
            fprintf('[HIL_PythonBridge] Aviso ao atualizar config .json: %s\n', ME.message);
        end

        % E) Disparar o Conversor Python em segundo plano (assincrono)
        rootDir = fileparts(fileparts(thisDir));
        venv_py = fullfile(rootDir, '.venv', 'Scripts', 'python.exe');
        if exist(venv_py, 'file')
            py_exe = venv_py;
        else
            py_exe = 'python';
        end

        cmd_run = sprintf('start "" /B "%s" "%s" --config "%s"', py_exe, py_script_path, json_config_path);
        fprintf('[HIL_PythonBridge] Disparando conversor Python em segundo plano...\n');
        system(cmd_run);

        % Aguarda 1s para o Python conectar e despausa simulacao no X-Plane
        pause(1);
        try
            pauseSim(0, GlobalSocket);
            fprintf('[HIL_PythonBridge] Simulacao no X-Plane despausada! Voo HIL em andamento.\n');
        catch ME
            fprintf('[HIL_PythonBridge] Aviso ao despausar X-Plane: %s\n', ME.message);
        end

        t_start_wall = tic;
        t_last_status_check = 0;
        is_initialized = true;
        return;
    end

    %% ====================================================================
    %% 2) EXECUCOES SEGUINTES (MONITORAMENTO DO TEMPO E STATUS DO CONVERSOR)
    %% ====================================================================
    % Checagem periodica a cada 0.5s de simulacao (evita sobrecarga de I/O em disco)
    if isempty(t_last_status_check) || (t - t_last_status_check >= 0.5)
        t_last_status_check = t;

        % A) Checar se o conversor Python finalizou via status JSON
        if exist(status_file_path, 'file')
            try
                txt = fileread(status_file_path);
                st_data = jsondecode(txt);
                if isfield(st_data, 'running') && ~st_data.running
                    if isfield(st_data, 'status') && st_data.status == 1
                        fprintf('\n[HIL_PythonBridge] Conversor Python concluiu com sucesso (%s)!\n', st_data.message);
                    else
                        fprintf('\n[HIL_PythonBridge] Conversor Python encerrou com status %d: %s\n', st_data.status, st_data.message);
                    end

                    % Pausa o X-Plane e encerra a simulacao Simulink
                    try, pauseSim(1, GlobalSocket); catch, end
                    fprintf('[HIL_PythonBridge] Encerrando simulacao no Simulink (t = %.2f s) para gerar graficos.\n', t);
                    is_initialized = [];
                    try, set_param(bdroot, 'SimulationCommand', 'stop'); catch, end
                    return;
                end
            catch
                % Arquivo pode estar sendo gravado naquele instante
            end
        end

        % B) Checar se o tempo de simulacao ja alcancou a duracao configurada
        if ~isempty(HIL_dur) && t >= HIL_dur
            fprintf('\n[HIL_PythonBridge] Tempo de simulacao atingido (t = %.2f s >= %.1f s).\n', t, HIL_dur);
            try, pauseSim(1, GlobalSocket); catch, end
            is_initialized = [];
            try, set_param(bdroot, 'SimulationCommand', 'stop'); catch, end
            return;
        end
    end

    % C) Watchdog de relogio de parede (caso Python trave sem atualizar status)
    if ~isempty(t_start_wall) && ~isempty(HIL_dur)
        elapsed_wall = toc(t_start_wall);
        if elapsed_wall > (2.0 * HIL_dur)
            warning('hil_serial_udp:timeout', ...
                'Tempo real (%.1f s) excedeu 2x o tempo configurado (%.1f s) sem resposta do conversor. Abortando.', ...
                elapsed_wall, HIL_dur);
            try, pauseSim(1, GlobalSocket); catch, end
            is_initialized = [];
            try, set_param(bdroot, 'SimulationCommand', 'stop'); catch, end
        end
    end
end
