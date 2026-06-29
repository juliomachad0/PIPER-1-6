function xplane_sensors = ins_read_xplane(~)
%READ_XPLANE Le o estado da aeronave no X-Plane via XPlaneConnect.
%
%   sensors = read_xplane(dummy)
%
%   Output (1x13):
%     sensors(1)  = VT    - velocidade aerodinamica (m/s)
%     sensors(2)  = theta - arfagem (rad)
%     sensors(3)  = q     - taxa de arfagem (rad/s)
%     sensors(4)  = h     - altitude MSL (m)
%     sensors(5)  = phi   - rolamento (rad)
%     sensors(6)  = p     - taxa de rolamento (rad/s)
%     sensors(7)  = psi   - proa (rad)
%     sensors(8)  = r     - taxa de guinada (rad/s)
%     sensors(9)  = xN    - posicao Norte relativa ao inicio (m)
%     sensors(10) = xE    - posicao Leste relativa ao inicio (m)
%     sensors(11) = abx - aceleração no eixo x do corpo (m/s²)
%     sensors(12) = aby - aceleração no eixo y do corpo (m/s²)
%     sensors(13) = abz - aceleração no eixo z do corpo (m/s²)
%     sensors(14) = vx - velociade no eixo x - East
%     sensors(15) = vy - velociade no eixo y - Up
%     sensors(16) = vz - velociade no eixo z - South
%   Usa variavel global GlobalSocket (compartilhada com send_xplane).
%   Posicao e relativa ao ponto onde a simulacao iniciou.

    global GlobalSocket;
    import XPlaneConnect.*;

    persistent xN0 xE0 initialized;

    xplane_sensors = zeros(1, 13);
%% CONEXÃO COM XPLANE
    % --- Abrir conexao se necessario ---
    if isempty(GlobalSocket)
        try
            GlobalSocket = openUDP('127.0.0.1', 49009);
            disp('read_xplane: Conexao X-Plane aberta.');
        catch ME
            disp(['read_xplane: Falha ao conectar - ' ME.message]);
            return;
        end
    end
    % --- Verificar socket valido ---
    if ~isa(GlobalSocket, 'gov.nasa.xpc.XPlaneConnect')
        return;
    end
%% --- Ler DataRefs --- DADOS
    try
        drefs = {
            'sim/flightmodel/position/true_airspeed',  % 1: VT (m/s)
            'sim/flightmodel/position/theta',          % 2: pitch (deg)
            'sim/flightmodel/position/Qrad',           % 3: pitch rate (rad/s)
            'sim/flightmodel/position/elevation',      % 4: altitude MSL (m)
            'sim/flightmodel/position/phi',            % 5: roll (deg)
            'sim/flightmodel/position/Prad',           % 6: roll rate (rad/s)
            'sim/flightmodel/position/psi',            % 7: heading (deg)
            'sim/flightmodel/position/Rrad',           % 8: yaw rate (rad/s)
            'sim/flightmodel/position/local_x',        % 9: posicao X OpenGL (East)
            'sim/flightmodel/position/local_z'         % 10: posicao Z OpenGL (South)
            'sim/flightmodel/position/local_ax',       % 11: acceleration x
            'sim/flightmodel/position/local_ay',       % 12: acceleration y
            'sim/flightmodel/position/local_az',       % 13: acceleration z
            'sim/flightmodel/position/local_vx',       % 14: local_vx = East
            'sim/flightmodel/position/local_vy',       % 15: local_vy = Up
            'sim/flightmodel/position/local_vz',       % 16: local_vz = South
        };
        % getDREFs retorna single array (nao cell) — usar indexacao ()
        result = double(getDREFs(drefs, GlobalSocket));
        d2r = pi / 180; % degrees to rad
        VT    = result(1);
        theta = result(2) * d2r;
        q     = result(3);
        h     = result(4);
        phi   = result(5) * d2r;
        p     = result(6);
        psi   = result(7) * d2r;
        psi   = atan2(sin(psi), cos(psi));  % wrap p/ [-pi,pi]
        r     = result(8);
        %% Posicao OpenGL (EUS) -> NED (verificado empiricamente)
        % local_x = East, local_y = Up, local_z = South
        % (diminui voando Norte)
        xE_abs = result(9);
        xN_abs = -result(10);
        %% Acquiring local accelerations - EUS
        aE_local = result(11);  % local_ax = East
        aU_local = result(12);  % local_ay = Up
        aS_local = result(13);  % local_az = South
        % EUS to NED
        aN = -aS_local;
        aE =  aE_local;
        aD = -aU_local;
        %% Euler angles
        roll = phi; pit = theta; yaw = psi;
        %% Acceleration from NED to BODY
        % R_n2b
        R_roll = [1 0 0; 0 cos(roll) sin(roll);0 -sin(roll) cos(roll)];
        R_pitch = [cos(pit) 0 -sin(pit); 0 1 0; sin(pit) 0 cos(pit)];
        R_yaw = [ cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0;0 0 1];
        R_n2b = R_roll*R_pitch*R_yaw;
        % transformation
        acc_b = R_n2b*[aN; aE; aD]; %acc no body
        abx = acc_b(1); aby = acc_b(2); abz = acc_b(3); % saídas
        %% velocidades (para os EKFs) EUS
        vE_local = result(14);   % local_vx = East
        vU_local = result(15);   % local_vy = Up
        vS_local = result(16);   % local_vz = South
        % conversão EUS to NED
        vN = -vS_local;
        vE =  vE_local;
        vD = -vU_local;
        %% Passando xN e xE como posicões relativas ao ponto inicial
        if isempty(initialized)
            xN0 = xN_abs;
            xE0 = xE_abs;
            initialized = true;
            disp(['ins_read_xplane: Posicao inicial capturada (N=' ...
                num2str(xN0, '%.1f') ', E=' num2str(xE0, '%.1f') ')']);
        end
        xN = xN_abs - xN0;
        xE = xE_abs - xE0;
        %% VETOR DE SAÍDA
        xplane_sensors = [VT, theta, q, h, phi, p, psi, r, ...
                          xN, xE, abx, aby, abz, vN, vE, vD];
    catch ME
        disp(['ins_read_xplane: Erro na leitura: ' ME.message]);
    end
end
