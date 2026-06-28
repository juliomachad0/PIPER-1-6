function icm20689 = ins_init_icm20689(seeds_icm20689)
%INIT_ICM20689 Inicializa parâmetros do ICM20689 (por enquanto: gyro).
% Esta função é chamada por init_sensors() e NÃO publica nada no Base Workspace.
% Ela apenas retorna a struct icm20689 com subestruturas (ex.: icm20689.gyro).
%
% Entrada:
%   seeds_icm20689: struct com seeds do ICM20689, por exemplo:
%       seeds_icm20689.gyro.params
%       seeds_icm20689.gyro.noise (opcional, usado no Simulink nos blocos Random Number)
%       seeds_icm20689.acc.params (futuro)
%
% Saída:
%   icm20689: struct com parâmetros do sensor (ex.: icm20689.gyro.*)
    %% (0) Reprodutibilidade (parâmetros sorteados)
    % Usamos o seed específico do GYRO e ACC para sortear os parâmetros fixos do sensor
    % (cross-axis, bias inicial, k3, etc.). Isso garante que o "mesmo sensor"
    % seja reproduzido sempre que a simulação rodar com o mesmo seed.
    if nargin >= 1 && isfield(seeds_icm20689,'gyro') && isfield(seeds_icm20689.gyro,'params')
        rng(seeds_icm20689.gyro.params);
    else
        % fallback (não recomendado): mantém comportamento previsível
        rng(1);
    end
        % ACC - Reprodutibilidade
    if nargin >= 1 && isfield(seeds_icm20689,'acc') && isfield(seeds_icm20689.acc,'params')
        rng(seeds_icm20689.acc.params);
    else
        rng(2);
    end
    %% (1) Girometros - Parâmetros basicos 
    gyro = struct();
    gyro.Ts = 1/200;                 % [s] taxa do sensor/PA (200 Hz)
    gyro.Fs = 1/gyro.Ts;             % [Hz] correção do bug (antes estava 1/Ts)
    gyro.FS_dps = 1000;              % [°/s] fundo de escala escolhido
    gyro.bits = 16;                  % [bits] resolução
    gyro.sens_LSB_per_dps = 32.8;    % [LSB/(°/s)] típico para ±1000 °/s
    gyro.sf_tol = 0.02;              % [-] erro de escala (±2% típico)
    gyro.sf_temp_dev_typ = 0.015;    % [-] variação típica vs temperatura (±1.5%)
    gyro.bias_temp = 0.05;           % [°/s/°C] drift térmico do bias (±0.05)
    gyro.noise_density = 0.006;      % [°/s/√Hz] noise density
    gyro.noise_BW = 59;              % [Hz] Noise BW (DLPF_CFG=3)
    gyro.sigma_noise_dps = gyro.noise_density * sqrt(gyro.noise_BW);
    gyro.cross_axis_max = 0.05;      % [-] cross-axis (pior caso; típico seria 0.02)
    gyro.nonlin_max = 0.001;         % [-] não-linearidade (±0.1%)
    gyro.deadzone_LSB = 1;           % [LSB] zona morta pós-quantização
    gyro.T_ref = 25;                 % [°C] referência
    gyro.yaw_mount_deg = 0;          % [deg] montagem nominal (0 => implícito)
    gyro.pitch_mount_deg = 0;        % [deg] montagem nominal (0 => implícito)
    gyro.roll_mount_deg = 0;         % [deg] montagem nominal (0 => implícito)
    % Sorteios por eixo (fixos por simulação)
    % distribuicoes tipicas via 3 sigma
    gyro.sf_fixed = 1 + (gyro.sf_tol/3)*randn(1,3);
    gyro.k_sf     = (gyro.sf_temp_dev_typ/3)/65 * randn(1,3);    % slope por eixo
    gyro.b0       = (5/3)*randn(1,3);                            % ZRO inicial típico
    gyro.k_zro    = (2*rand(1,3)-1)*gyro.bias_temp;              % coef térmico do bias
    gyro.k3       = (2*rand(1,3)-1)*gyro.nonlin_max;             % não-linearidade por eixo
    % Matriz cross-axis 3x3 (Cg)
    E = zeros(3);
    for i = 1:3
        for j = 1:3
            if i ~= j
                E(i,j) = (2*rand-1)*gyro.cross_axis_max;
            end
        end
    end
    gyro.Cg = eye(3) + E;
    %% (2) Acelerometros - Parâmetros basicos
    acc = struct();
    acc.Ts = 1/200;
    acc.Fs = 1/acc.Ts;
    acc.gravity = 9.807;
    acc.FS_g = 8;                       % ±8g
    acc.bits = 16;
    acc.sens_LSB_per_g = 4096;          % LSB/g para ±8g
    acc.fundo_escala = acc.FS_g * acc.gravity; % m/s^2
    acc.zero_g_max_mg = 80;
    acc.zero_g_sigma_mg = acc.zero_g_max_mg/3;
    acc.zero_g_temp_mg_per_C = 0.75;
    acc.noise_density_ug_sqrtHz = 150;
    acc.noise_density_g_sqrtHz = acc.noise_density_ug_sqrtHz * 1e-6;
    acc.noise_BW_Hz = 61.5;
    acc.sigma_noise_g = acc.noise_density_g_sqrtHz * sqrt(acc.noise_BW_Hz);
    acc.sigma_noise_ms2 = acc.sigma_noise_g * acc.gravity;
    acc.sf_tol_typ = 0.02;
    acc.sf_temp_dev_typ = 0.01;
    acc.nonlin_max = 0.005;
    acc.cross_axis_max = 0.05;
    acc.deadzone_LSB = 1;
    acc.T_ref = 25;
    % desalinhamento do eixo
    acc.roll_mount_deg = 0;
    acc.pitch_mount_deg = 0;
    acc.yaw_mount_deg = 0;
    % ACC - Bias e drift térmico
    acc.bias0_g = (acc.zero_g_sigma_mg*1e-3) * randn(1,3);
    acc.kT_bias_g_per_C = ...
        (acc.zero_g_temp_mg_per_C*1e-3) * (2*rand(1,3)-1);
    % ACC - Misalignment
    % O datasheet não fornece desalinhamento angular diretamente.
    % Este bloco fica como parâmetro controlável do modelo.
    % Se quiser desligar, usar acc.misalign_deg_max = 0.
    acc.misalign_deg_max = 0.0;
    acc.misalign_sigma_rad = deg2rad(acc.misalign_deg_max)/3;
    alpha = acc.misalign_sigma_rad * randn;
    beta  = acc.misalign_sigma_rad * randn;
    gamma = acc.misalign_sigma_rad * randn;
    acc.R_mis = eye(3) + [ ...
          0     -gamma    beta;
        gamma      0     -alpha;
       -beta     alpha      0 ];
    % ACC - Cross-axis
    E = zeros(3);
    for i = 1:3
        for j = 1:3
            if i ~= j
                E(i,j) = (2*rand-1)*acc.cross_axis_max;
            end
        end
    end
    acc.Ca = eye(3) + E;
    % ACC - Scale factor
    sf_sigma = acc.sf_tol_typ/3;
    acc.sf_fixed = 1 + sf_sigma*randn(1,3);
    dT_max = 65;
    k_sf_sigma = (acc.sf_temp_dev_typ/3)/dT_max;
    acc.k_sf = k_sf_sigma*randn(1,3);
    % ACC - Não-linearidade
    acc.k3 = (2*rand(1,3)-1) * acc.nonlin_max;
    %% (3) Monta a struct final do ICM20689
    icm20689 = struct();
    icm20689.gyro = gyro;
    icm20689.acc = acc;
    disp("icm20689 sensor successfully creadted")
end