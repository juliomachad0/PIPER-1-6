function gps = ins_init_gps(seeds_gps, model_name)
%INS_INIT_GPS Inicializa parâmetros fixos de erro do GNSS u-blox NEO-M8.
%
% Modelo:
%   pos_meas = pos_ned_true + bias_pos + noise_pos_m
%   vel_meas = vel_ned_true + noise_vel_ms
%
% Os ruídos variantes no tempo entram pelo Simulink.
% Os parâmetros fixos ficam em sensors.gps.

    %% (0) Modelo padrão
    if nargin < 2 || isempty(model_name)
        model_name = "NEO-M8N";
    end

    %% (1) Reprodutibilidade dos parâmetros fixos
    if nargin >= 1 && isfield(seeds_gps, 'params')
        rng(seeds_gps.params);
    else
        rng(3);
    end

    %% (2) Identificação básica
    gps = struct();

    gps.manufacturer = "u-blox";
    gps.family       = "NEO-M8";
    gps.model        = string(model_name);
    gps.frame        = "NED";

    %% (3) Frequência nominal do GNSS
    switch upper(char(gps.model))
        case 'NEO-M8N'
            gps.Fs = 5;
        case 'NEO-M8Q'
            gps.Fs = 10;
        case 'NEO-M8J'
            gps.Fs = 5;
        case 'NEO-M8M'
            gps.Fs = 10;
        otherwise
            gps.model = "NEO-M8N";
            gps.Fs = 5;
    end

    gps.Ts = 1/gps.Fs;

    %% (4) Erro de posição
    gps.horizontal_accuracy_cep50_m = 2.0;

    gps.sigma_pos_NE_total = gps.horizontal_accuracy_cep50_m / sqrt(2*log(2));

    gps.vertical_sigma_ratio = 1.5;

    gps.sigma_pos_N = gps.sigma_pos_NE_total;
    gps.sigma_pos_E = gps.sigma_pos_NE_total;
    gps.sigma_pos_D = gps.vertical_sigma_ratio * gps.sigma_pos_NE_total;

    gps.sigma_pos_total_NED = [ ...
        gps.sigma_pos_N; ...
        gps.sigma_pos_E; ...
        gps.sigma_pos_D];

    %% (5) Separação entre ruído branco e bias correlacionado
    gps.white_noise_variance_fraction = 0.60;
    gps.bias_variance_fraction        = 0.40;

    gps.sigma_eta_pos_NED = sqrt(gps.white_noise_variance_fraction) ...
        * gps.sigma_pos_total_NED;

    gps.sigma_bias_ss_NED = sqrt(gps.bias_variance_fraction) ...
        * gps.sigma_pos_total_NED;

    %% (6) Bias Gauss-Markov de posição
    gps.lambda_bias = 1/60;  % [1/s]

    gps.bias0_pos = gps.sigma_bias_ss_NED .* randn(3,1);

    %% (7) Erro de velocidade
    gps.velocity_accuracy_50pct_ms = 0.05;

    gps.sigma_vel_N = gps.velocity_accuracy_50pct_ms;
    gps.sigma_vel_E = gps.velocity_accuracy_50pct_ms;
    gps.sigma_vel_D = 1.5 * gps.velocity_accuracy_50pct_ms;

    gps.sigma_vel_NED = [ ...
        gps.sigma_vel_N; ...
        gps.sigma_vel_E; ...
        gps.sigma_vel_D];

    %% (8) Seeds para blocos Random Number no Simulink
    if nargin >= 1 && isfield(seeds_gps, 'noise_pos')
        gps.noise_seed_pos = seeds_gps.noise_pos;
    else
        gps.noise_seed_pos = [3001 3002 3003];
    end

    if nargin >= 1 && isfield(seeds_gps, 'noise_vel')
        gps.noise_seed_vel = seeds_gps.noise_vel;
    else
        gps.noise_seed_vel = [3004 3005 3006];
    end

    if nargin >= 1 && isfield(seeds_gps, 'noise_bias')
        gps.noise_seed_bias = seeds_gps.noise_bias;
    else
        gps.noise_seed_bias = [3007 3008 3009];
    end

    %% (9) Matrizes úteis
    gps.R_pos = diag(gps.sigma_pos_total_NED.^2);
    gps.R_vel = diag(gps.sigma_vel_NED.^2);
    gps.R     = blkdiag(gps.R_pos, gps.R_vel);

    gps.R_pos_white = diag(gps.sigma_eta_pos_NED.^2);

    disp("u-blox NEO-M8 GNSS sensor successfully created")

end