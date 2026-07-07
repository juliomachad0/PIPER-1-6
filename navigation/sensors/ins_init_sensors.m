%% Funcao Principal
function sensors = ins_init_sensors()
    disp("---------- Sensors ----------")
    %INIT_SENSORS Inicializa e consolida parâmetros do sistema e sensores.
    % Publica UMA ÚNICA variável no Base Workspace: 'sensors'.

    %% (0) Configuração local
    Ts = 1/200;          % sample time do PA/IMU
    Fs = 1/Ts;
    seed_master = 1;     % seed mestre do experimento

    %% (1) Estrutura do sistema
    sensors = struct();
    sensors.sys = struct();
    sensors.sys.Ts = Ts;
    sensors.sys.Fs = Fs;
    sensors.sys.seed_master = seed_master;
    sensors.seeds = struct();

    %% seeds específicos para cada sensor
    sensors = icm20689_seeds_generator(sensors); % ICM20689
    sensors = gps_seeds_generator(sensors);      % GPS / GNSS
    sensors = ist8310_seeds_generator(sensors);  % IST8310 / Magnetometro

    %% Inicializando sensores
    sensors.icm20689 = ins_init_icm20689(sensors.seeds.icm20689); % ICM20689 - ICM
    sensors.gps      = ins_init_gps(sensors.seeds.gps);           % u-blox NEO-M8
    sensors.ist8310  = ins_init_ist8310(sensors.seeds.ist8310);   % magnetometro

    %% Publica no workspace
    assignin('base', 'sensors', sensors);
    clear all;
    disp("-----------------------------")
end

%% SEEDS ICM20689
function sensors = icm20689_seeds_generator(sensors)
    % =========================
    % (2) Seeds globais (separa parâmetros de ruído)
    % =========================
    seed_master = sensors.sys.seed_master;
    seed_params = seed_master + 1000;  % para sorteio de parâmetros (Cg, b0, k3, etc.)
    seed_noise  = seed_master + 2000;  % para ruído temporal (Random Number no Simulink)
    sensors.sys.seed_params = seed_params;
    sensors.sys.seed_noise = seed_noise;
    % =========================
    % (3) Offsets (padrão simples e escalável)
    % =========================
    OFF.icm20689 = 100;
    OFF.gyro     = 10;
    OFF.acc      = 20;
    
    % gerador de seeds específicos para cada sensor
     % para evitar poluir função principal conforme o número de sensores
    % aumentar
    % =========================
    % (4) Seeds por sensor/componente
    % =========================
    sensors.seeds.icm20689 = struct();

    % ---- ICM20689 / GYRO ----
    sensors.seeds.icm20689.gyro = struct();
    sensors.seeds.icm20689.gyro.params = seed_params + OFF.icm20689 + OFF.gyro;
    sensors.seeds.icm20689.gyro.noise  = [ ...
        seed_noise + OFF.icm20689 + OFF.gyro + 1, ... % X
        seed_noise + OFF.icm20689 + OFF.gyro + 2, ... % Y
        seed_noise + OFF.icm20689 + OFF.gyro + 3  ... % Z
    ];

    % ---- ICM20689 / ACC ----
    sensors.seeds.icm20689.acc = struct();
    sensors.seeds.icm20689.acc.params = seed_params + OFF.icm20689 + OFF.acc;
    sensors.seeds.icm20689.acc.noise  = [ ...
        seed_noise + OFF.icm20689 + OFF.acc + 1, ... % X
        seed_noise + OFF.icm20689 + OFF.acc + 2, ... % Y
        seed_noise + OFF.icm20689 + OFF.acc + 3  ... % Z
    ];
end

%% SEEDS GPS/GNSS - u-blox NEO-M8
function sensors = gps_seeds_generator(sensors)

seed_master = sensors.sys.seed_master;
seed_params = seed_master + 1000;
seed_noise  = seed_master + 2000;

OFF.gps = 300;

sensors.seeds.gps = struct();

sensors.seeds.gps.params = seed_params + OFF.gps;

% Ruído branco de posição NED
sensors.seeds.gps.noise_pos = [ ...
    seed_noise + OFF.gps + 1, ... % N
    seed_noise + OFF.gps + 2, ... % E
    seed_noise + OFF.gps + 3  ... % D
    ];

% Ruído branco de velocidade NED
sensors.seeds.gps.noise_vel = [ ...
    seed_noise + OFF.gps + 4, ... % vN
    seed_noise + OFF.gps + 5, ... % vE
    seed_noise + OFF.gps + 6  ... % vD
    ];

% Ruído unitário para o bias Gauss-Markov
sensors.seeds.gps.noise_bias = [ ...
    seed_noise + OFF.gps + 7, ... % bias N
    seed_noise + OFF.gps + 8, ... % bias E
    seed_noise + OFF.gps + 9  ... % bias D
    ];

end
%% SEEDS IST8310
function sensors = ist8310_seeds_generator(sensors)

    seed_master = sensors.sys.seed_master;
    seed_params = seed_master + 1000;
    seed_noise  = seed_master + 2000;

    OFF.ist8310 = 400;
    OFF.mag     = 10;

    sensors.seeds.ist8310 = struct();
    sensors.seeds.ist8310.mag = struct();

    sensors.seeds.ist8310.mag.params = seed_params + OFF.ist8310 + OFF.mag;

    sensors.seeds.ist8310.mag.noise = [ ...
        seed_noise + OFF.ist8310 + OFF.mag + 1, ... % X
        seed_noise + OFF.ist8310 + OFF.mag + 2, ... % Y
        seed_noise + OFF.ist8310 + OFF.mag + 3  ... % Z
    ];
end
