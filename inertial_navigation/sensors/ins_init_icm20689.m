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

    %% -----------------------------
    %% (0) Reprodutibilidade (parâmetros sorteados)
    %% -----------------------------
    % Usamos o seed específico do GYRO para sortear os parâmetros fixos do sensor
    % (cross-axis, bias inicial, k3, etc.). Isso garante que o "mesmo sensor"
    % seja reproduzido sempre que a simulação rodar com o mesmo seed.
    if nargin >= 1 && isfield(seeds_icm20689,'gyro') && isfield(seeds_icm20689.gyro,'params')
        rng(seeds_icm20689.gyro.params);
    else
        % fallback (não recomendado): mantém comportamento previsível
        rng(1);
    end

    %% -----------------------------
    %% (1) Parâmetros básicos (datasheet / trabalho)
    %% -----------------------------
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

    %% -----------------------------
    %% (2) Sorteios por eixo (fixos por simulação)
    %% -----------------------------
    % Obs.: assumimos distribuições típicas via 3σ ~ limite, como no seu modelo.
    gyro.sf_fixed = 1 + (gyro.sf_tol/3)*randn(1,3);
    gyro.k_sf     = (gyro.sf_temp_dev_typ/3)/65 * randn(1,3);    % slope por eixo
    gyro.b0       = (5/3)*randn(1,3);                            % ZRO inicial típico
    gyro.k_zro    = (2*rand(1,3)-1)*gyro.bias_temp;              % coef térmico do bias
    gyro.k3       = (2*rand(1,3)-1)*gyro.nonlin_max;             % não-linearidade por eixo

    %% -----------------------------
    %% (3) Matriz cross-axis 3x3 (Cg)
    %% -----------------------------
    E = zeros(3);
    for i = 1:3
        for j = 1:3
            if i ~= j
                E(i,j) = (2*rand-1)*gyro.cross_axis_max;
            end
        end
    end
    gyro.Cg = eye(3) + E;

    %% -----------------------------
    %% (4) Monta a struct final do ICM20689
    %% -----------------------------
    icm20689 = struct();
    icm20689.gyro = gyro;
    
    % Futuro:
    % icm20689.acc = ... (usando seeds_icm20689.acc.params)
    disp("icm20689 sensor successfully creadted")
end