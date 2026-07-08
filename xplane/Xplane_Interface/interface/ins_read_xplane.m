function xplane_sensors = ins_read_xplane(~)
%READ_XPLANE Le o estado da aeronave no X-Plane via XPlaneConnect.
%
% Output:
%   sensors(1)  = VT    - velocidade aerodinamica (m/s)
%   sensors(2)  = theta - arfagem (rad)
%   sensors(3)  = q     - taxa de arfagem (rad/s)
%   sensors(4)  = h     - altitude AGL (m)
%   sensors(5)  = phi   - rolamento (rad)
%   sensors(6)  = p     - taxa de rolamento (rad/s)
%   sensors(7)  = psi   - proa/heading atual (rad)
%   sensors(8)  = r     - taxa de guinada (rad/s)
%   sensors(9)  = xN    - posicao Norte relativa ao inicio (m)
%   sensors(10) = xE    - posicao Leste relativa ao inicio (m)
%   sensors(11) = abx   - aceleracao no eixo x do corpo (m/s^2)
%   sensors(12) = aby   - aceleracao no eixo y do corpo (m/s^2)
%   sensors(13) = abz   - aceleracao no eixo z do corpo (m/s^2)
%   sensors(14) = vN    - velocidade Norte (m/s)
%   sensors(15) = vE    - velocidade Leste (m/s)
%   sensors(16) = vD    - velocidade Down (m/s)
%   sensors(17) = magnetic_variation - variacao magnetica local (rad)

    global GlobalSocket;
    import XPlaneConnect.*;

    persistent xN0 xE0 initialized;

    xplane_sensors = zeros(1, 17);

    %% CONEXAO COM XPLANE
    if isempty(GlobalSocket)
        try
            GlobalSocket = openUDP('127.0.0.1', 49009);
            disp('ins_read_xplane: Conexao X-Plane aberta.');
        catch ME
            disp(['ins_read_xplane: Falha ao conectar - ' ME.message]);
            return;
        end
    end

    if ~isa(GlobalSocket, 'gov.nasa.xpc.XPlaneConnect')
        return;
    end

    %% Ler DataRefs
    try
        drefs = {
            'sim/flightmodel/position/true_airspeed',       % 1: VT (m/s)
            'sim/flightmodel/position/theta',               % 2: pitch (deg)
            'sim/flightmodel/position/Qrad',                % 3: pitch rate (rad/s)
            'sim/flightmodel/position/y_agl',               % 4: altitude AGL (m)
            'sim/flightmodel/position/phi',                 % 5: roll (deg)
            'sim/flightmodel/position/Prad',                % 6: roll rate (rad/s)
            'sim/flightmodel/position/psi',                 % 7: heading (deg)
            'sim/flightmodel/position/Rrad',                % 8: yaw rate (rad/s)
            'sim/flightmodel/position/local_x',             % 9: local_x = East
            'sim/flightmodel/position/local_z',             % 10: local_z = South
            'sim/flightmodel/position/local_ax',            % 11: acceleration x
            'sim/flightmodel/position/local_ay',            % 12: acceleration y
            'sim/flightmodel/position/local_az',            % 13: acceleration z
            'sim/flightmodel/position/local_vx',            % 14: local_vx = East
            'sim/flightmodel/position/local_vy',            % 15: local_vy = Up
            'sim/flightmodel/position/local_vz',            % 16: local_vz = South
            'sim/flightmodel/position/magnetic_variation'   % 17: magnetic variation (deg)
        };

        result = double(getDREFs(drefs, GlobalSocket));

        d2r = pi / 180;

        %% Sinais originais - indices 1 a 16 preservados
        VT    = result(1);
        theta = result(2) * d2r;
        q     = result(3);
        h     = result(4);
        phi   = result(5) * d2r;
        p     = result(6);
        psi   = result(7) * d2r;
        psi   = wrapToPi_local(psi);
        r     = result(8);

        %% Posicao OpenGL EUS -> NED
        xE_abs = result(9);
        xN_abs = -result(10);

        %% Aceleracoes locais EUS -> NED
        aE_local = result(11);
        aU_local = result(12);
        aS_local = result(13);

        aN = -aS_local;
        aE =  aE_local;
        aD = -aU_local;

        %% Euler para rotacao NED -> BODY
        roll = phi;
        pit  = theta;
        yaw  = psi;

        R_roll = [ ...
            1,          0,           0; ...
            0,  cos(roll),   sin(roll); ...
            0, -sin(roll),   cos(roll)];

        R_pitch = [ ...
             cos(pit), 0, -sin(pit); ...
                    0, 1,         0; ...
             sin(pit), 0,  cos(pit)];

        R_yaw = [ ...
             cos(yaw),  sin(yaw), 0; ...
            -sin(yaw),  cos(yaw), 0; ...
                    0,         0, 1];

        R_n2b = R_roll * R_pitch * R_yaw;

        acc_b = R_n2b * [aN; aE; aD];

        abx = acc_b(1);
        aby = acc_b(2);
        abz = acc_b(3);

        %% Velocidades EUS -> NED
        vE_local = result(14);
        vU_local = result(15);
        vS_local = result(16);

        vN = -vS_local;
        vE =  vE_local;
        vD = -vU_local;

        %% Posicao relativa
        if isempty(initialized)
            xN0 = xN_abs;
            xE0 = xE_abs;
            initialized = true;

            disp(['ins_read_xplane: Posicao inicial capturada (N=' ...
                num2str(xN0, '%.1f') ', E=' num2str(xE0, '%.1f') ')']);
        end

        xN = xN_abs - xN0;
        xE = xE_abs - xE0;

        %% Nova variavel - indice 17
        magnetic_variation = result(17) * d2r;

        %% Vetor de saida
        xplane_sensors = [ ...
            VT, theta, q, h, phi, p, psi, r, ...
            xN, xE, abx, aby, abz, vN, vE, vD, ...
            magnetic_variation];

        % fprintf(['VT: %.3f, theta: %.3f, q: %.3f, h: %.3f, phi: %.3f, p: %.3f, psi: %.3f, r: %.3f, ' ...
        %          'xN: %.3f, xE: %.3f, abx: %.3f, aby: %.3f, abz: %.3f, vN: %.3f, vE: %.3f, vD: %.3f, ' ...
        %          'mavar: %.3f\n'], ...
        %          VT, theta, q, h, phi, p, psi, r, ...
        %          xN, xE, abx, aby, abz, vN, vE, vD, ...
        %          magnetic_variation);

    catch ME
        disp(['ins_read_xplane: Erro na leitura: ' ME.message]);
    end
end

function ang = wrapToPi_local(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end