function ins_initial_state_xplane_test()
%Teleporta a aeronave para a condicao inicial de voo
    global GlobalSocket;
    import XPlaneConnect.*;

    if isempty(GlobalSocket)
        try
            GlobalSocket = openUDP('127.0.0.1', 49009);
        catch ME
            disp(['posicionar_xplane: falha ao conectar - ' ME.message]);
            return;
        end
    else
        disp('X-Plane Communication: ins_initial_state_xplane: UDP conection alread exists.')
    end
    WPs = evalin('base','WPs'); %getting initial state (position, velocity)
    try
        drefs_lla = {'sim/flightmodel/position/latitude', ...
                    'sim/flightmodel/position/longitude', ...
                    'sim/flightmodel/position/elevation', ...
                    'sim/flightmodel/position/y_agl'};
        lla = double(getDREFs(drefs_lla, GlobalSocket)); %ll: lat - long - MSL
        pauseSim(1, GlobalSocket);
        pause(0.5);
        psi0 = 0;  % Norte
        % Set the initial position and orientation of the aircraft
        % lla(1) - latitude
        % lla(2) - longitude
        % lla(3) - elevation-Mean Sea Level - MSL
        % lla(4) - above ground level - AGL
        h0 = 500; %WPs(1,3);
        v0 = 30; %WPs(1,4);
        %
        elev_msl = lla(3); % Mean Sea Level elevation
        y_agl = lla(4); %
        ground_msl = elev_msl - y_agl;
        target_msl = ground_msl + h0; % final height = WPs(1,3) above ground
        sendPOSI([lla(1), lla(2), target_msl, 0, 0, psi0, 0], 0, GlobalSocket);
        pause(0.5);
        abs_V0 = v0;
        hdg_rad = psi0 * pi/180;
        sendDREF('sim/flightmodel/position/local_vx',  abs_V0*sin(hdg_rad), GlobalSocket);
        sendDREF('sim/flightmodel/position/local_vy',  0,                GlobalSocket);
        sendDREF('sim/flightmodel/position/local_vz', -abs_V0*cos(hdg_rad), GlobalSocket);
        pause(0.2);
        sendCTRL([0, 0, 0, 0.49, -998, -998], 0, GlobalSocket);
        pause(0.2);
        pauseSim(0, GlobalSocket);

        % Limpar persistent vars (xN0/xE0) para reiniciar refs de posicao
        clear ins_read_xplane;

        fprintf('posicionar_xplane: aeronave em %.1f m, VT=%.1f m/s, hdg=%.1f deg.\n',WPs(1,3), WPs(1,4), psi0);
    catch ME
        disp(['posicionar_xplane: erro - ' ME.message]);
    end
end