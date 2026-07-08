function EKF_INDI_params = init_EKF_INDI(xplane_init)
% init_EKF_INDI
%
% Inicializa os parametros do EKF indireto para uso no Simulink.
%
% Entrada:
%   xplane_init.euler0    = [phi0; theta0; psi0] [rad]
%   xplane_init.pos0_ned  = [N0; E0; D0] [m]
%   xplane_init.vel0_ned  = [vN0; vE0; vD0] [m/s]
%
% Saida:
%   EKF_INDI_params publicado no workspace base.

EKF_INDI_params = struct();
EKF_INDI_params.initialized = true;

%% Condicoes iniciais
EKF_INDI_params.euler0   = xplane_init.euler0(:);
EKF_INDI_params.pos0_ned = xplane_init.pos0_ned(:);
EKF_INDI_params.vel0_ned = xplane_init.vel0_ned(:);

EKF_INDI_params.g0 = 9.80665;

%% Sensores
try
    sensors = evalin('base', 'sensors');
catch
    sensors = struct();
end

EKF_INDI_params.sensors = sensors;

%% Parametros de ruido vindos da struct sensors, com fallback
if isfield(sensors, 'icm20689') && isfield(sensors.icm20689, 'acc')
    acc = sensors.icm20689.acc;
else
    acc = struct();
end

if isfield(sensors, 'icm20689') && isfield(sensors.icm20689, 'gyro')
    gyro = sensors.icm20689.gyro;
else
    gyro = struct();
end

% Acelerometro
if isfield(acc, 'sigma_noise_ms2')
    sigma_acc = acc.sigma_noise_ms2;
else
    sigma_acc = 0.02;
end

if isfield(acc, 'gravity')
    g_acc = acc.gravity;
else
    g_acc = EKF_INDI_params.g0;
end

if isfield(acc, 'zero_g_sigma_mg')
    sigma_ba0 = acc.zero_g_sigma_mg * 1e-3 * g_acc;
else
    sigma_ba0 = 0.30;
end

if isfield(acc, 'kT_bias_g_per_C')
    sigma_ba_vec = abs(acc.kT_bias_g_per_C(:)) * g_acc * 0.05;
else
    sigma_ba_vec = 0.002 * ones(3,1);
end

% Giroscopio
if isfield(gyro, 'noise_density')
    sigma_gyro = deg2rad(gyro.noise_density);
elseif isfield(gyro, 'sigma_noise_dps')
    sigma_gyro = deg2rad(gyro.sigma_noise_dps);
else
    sigma_gyro = deg2rad(0.05);
end

if isfield(gyro, 'b0')
    sigma_bg0 = max(std(deg2rad(gyro.b0(:))), deg2rad(0.5));
else
    sigma_bg0 = deg2rad(5);
end

if isfield(gyro, 'k_zro')
    sigma_bg_vec = abs(deg2rad(gyro.k_zro(:))) * 0.05;
else
    sigma_bg_vec = deg2rad(0.01) * ones(3,1);
end

%% Medidas auxiliares tipo GPS
EKF_INDI_params.gps_Fs = 1;
EKF_INDI_params.gps_period = 1/EKF_INDI_params.gps_Fs;

% Posicao: [N E D], com terceiro eixo mais ruidoso
EKF_INDI_params.R_pos = diag([1.0^2, 1.0^2, 1.5^2]);

% Velocidade 3D NED
EKF_INDI_params.R_vel = (0.05^2) * eye(3);

% Bias de posicao tipo Gauss-Markov
EKF_INDI_params.lambda_y = 1/60;
EKF_INDI_params.sigma_by = sqrt(2*EKF_INDI_params.lambda_y) * 3.0;

%% Medida auxiliar de yaw / magnetometro

if isfield(sensors, 'ist8310') && isfield(sensors.ist8310, 'mag')

    mag = sensors.ist8310.mag;

    if isfield(mag, 'Fs')
        EKF_INDI_params.mag_Fs = mag.Fs;
    elseif isfield(mag, 'Ts')
        EKF_INDI_params.mag_Fs = 1/mag.Ts;
    else
        EKF_INDI_params.mag_Fs = 50;
    end

    EKF_INDI_params.mag_period = 1/EKF_INDI_params.mag_Fs;

    if isfield(mag, 'mag_n_ref_uT')
        mag_horiz_norm = norm(mag.mag_n_ref_uT(1:2));
    else
        mag_horiz_norm = norm([23; -5]);
    end

    if isfield(mag, 'sigma_noise_uT')
        sigma_yaw_mag = mag.sigma_noise_uT / max(mag_horiz_norm, 1e-6);
        sigma_yaw_mag = max(sigma_yaw_mag, deg2rad(0.8));
    else
        sigma_yaw_mag = deg2rad(2.0);
    end

    EKF_INDI_params.R_yaw = sigma_yaw_mag^2;

else

    EKF_INDI_params.mag_Fs = 50;
    EKF_INDI_params.mag_period = 1/EKF_INDI_params.mag_Fs;
    EKF_INDI_params.R_yaw = deg2rad(2.0)^2;

end

%% Estado inicial completo
nx = 21;
x0 = zeros(nx,1);

x0(1:3)   = EKF_INDI_params.pos0_ned;
x0(4:6)   = EKF_INDI_params.vel0_ned;
x0(7:9)   = EKF_INDI_params.euler0;
x0(10:12) = zeros(3,1);  % bias acelerometro
x0(13:15) = zeros(3,1);  % bias giroscopio
x0(16:18) = zeros(3,1);  % bias da medida de posicao
x0(19:21) = zeros(3,1);  % fator de escala acelerometro

EKF_INDI_params.x0 = x0;

%% Covariancia inicial - baseada no script isolado do EKF indireto
P0 = diag([ ...
    10.0^2*ones(1,3), ...
     1.0^2*ones(1,3), ...
     deg2rad(5)^2*ones(1,3), ...
     sigma_ba0^2*ones(1,3), ...
     sigma_bg0^2*ones(1,3), ...
     3.0^2*ones(1,3), ...
     0.02^2*ones(1,3) ...
]);

EKF_INDI_params.P0 = P0;

%% Ruido de processo continuo
Qw = diag([ ...
    sigma_acc^2*ones(3,1); ...
    sigma_gyro^2*ones(3,1); ...
    sigma_ba_vec(:).^2; ...
    sigma_bg_vec(:).^2; ...
    EKF_INDI_params.sigma_by^2*ones(3,1) ...
]);

EKF_INDI_params.Qw = Qw;

%% Convencao da aceleracao
% Pela sua leitura do X-Plane, acc_b_in ja e aceleracao translacional em corpo.
% Portanto, por padrao nao se soma gravidade na propagacao.
EKF_INDI_params.acc_input_is_translational = true;

%% Token para forcar reinicializacao entre simulacoes
EKF_INDI_params.reset_token = now;

assignin('base', 'EKF_INDI_params', EKF_INDI_params);

clear EKF_INDI_solver;
clear EKF_INDI_EM_solver;

disp('--- EKF_INDI_params inicializado ---');
fprintf('Euler0 [deg] = [%.3f %.3f %.3f]\n', rad2deg(EKF_INDI_params.euler0));
fprintf('Pos0 NED [m] = [%.3f %.3f %.3f]\n', EKF_INDI_params.pos0_ned);
fprintf('Vel0 NED [m/s] = [%.3f %.3f %.3f]\n', EKF_INDI_params.vel0_ned);

end