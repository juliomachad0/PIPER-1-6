function EKF_DI_params = init_EKF_DI(xplane_init)
% init_EKF_DI
%
% Inicializa parâmetros do EKF direto.
%
% Entrada:
%   xplane_init.euler0    = [phi0; theta0; psi0] [rad]
%   xplane_init.pos0_ned  = [N0; E0; D0] [m]
%   xplane_init.vel0_ned  = [vN0; vE0; vD0] [m/s]
%
% Saída:
%   EKF_DI_params no workspace base.

EKF_DI_params = struct();
EKF_DI_params.initialized = true;

%% Condições iniciais

EKF_DI_params.euler0   = xplane_init.euler0(:);
EKF_DI_params.pos0_ned = xplane_init.pos0_ned(:);
EKF_DI_params.vel0_ned = xplane_init.vel0_ned(:);

EKF_DI_params.g0 = 9.807;

%% Carregar sensores

try
    sensors = evalin('base', 'sensors');
    EKF_DI_params.sensors = sensors;
catch
    sensors = struct();
    EKF_DI_params.sensors = sensors;
end

%% Parâmetros do sensor

if isfield(sensors, 'icm20689')

    acc = sensors.icm20689.acc;
    gyro = sensors.icm20689.gyro;

    sigma_acc = acc.sigma_noise_ms2;
    sigma_gyro = deg2rad(gyro.sigma_noise_dps);

    sigma_ba = acc.zero_g_temp_mg_per_C * 1e-3 * acc.gravity * 0.05;
    sigma_bg = deg2rad(gyro.bias_temp) * 0.05;

    sigma_ba0 = acc.zero_g_sigma_mg * 1e-3 * acc.gravity;
    sigma_bg0 = deg2rad(5/3);

else

    sigma_acc = 0.02;
    sigma_gyro = deg2rad(0.05);

    sigma_ba = 1e-4;
    sigma_bg = deg2rad(1e-3);

    sigma_ba0 = 0.1;
    sigma_bg0 = deg2rad(0.5);

end

%% Parâmetros GPS/pseudo-GPS

EKF_DI_params.gps_Fs = 1;                  % GPS a 1 Hz
EKF_DI_params.gps_period = 1/EKF_DI_params.gps_Fs;

EKF_DI_params.R_pos = diag([1.0^2, 1.0^2, 1.5^2]);
EKF_DI_params.R_vel = (0.05^2)*eye(3);

lambda_y = 1/60;
EKF_DI_params.lambda_y = lambda_y;

sigma_by = sqrt(3)/30;
EKF_DI_params.sigma_by = sigma_by;

%% Parâmetros Magnetômetro / yaw

if isfield(sensors, 'ist8310') && isfield(sensors.ist8310, 'mag')

    mag = sensors.ist8310.mag;

    if isfield(mag, 'Fs')
        EKF_DI_params.mag_Fs = mag.Fs;
    elseif isfield(mag, 'Ts')
        EKF_DI_params.mag_Fs = 1/mag.Ts;
    else
        EKF_DI_params.mag_Fs = EKF_DI_params.gps_Fs;
    end

    EKF_DI_params.mag_period = 1/EKF_DI_params.mag_Fs;

    if isfield(mag, 'mag_n_ref_uT')
        mag_horiz_norm = norm(mag.mag_n_ref_uT(1:2));
    else
        mag_horiz_norm = norm([23; -5]);
    end

    if isfield(mag, 'sigma_noise_uT')
        sigma_yaw_mag = max(mag.sigma_noise_uT / max(mag_horiz_norm, 1e-6), deg2rad(0.8));
    else
        sigma_yaw_mag = deg2rad(2.0);
    end

    EKF_DI_params.R_yaw = sigma_yaw_mag^2;

else

    EKF_DI_params.mag_Fs = 50;
    EKF_DI_params.mag_period = 1/EKF_DI_params.mag_Fs;
    EKF_DI_params.R_yaw = deg2rad(2.0)^2;

end

%% Estado inicial

nx = 21;

x0 = zeros(nx,1);

x0(1:3)   = EKF_DI_params.pos0_ned;
x0(4:6)   = EKF_DI_params.vel0_ned;
x0(7:9)   = EKF_DI_params.euler0;
x0(10:12) = zeros(3,1);   % bias acelerômetro
x0(13:15) = zeros(3,1);   % bias giroscópio
x0(16:18) = zeros(3,1);   % bias posição/GPS
x0(19:21) = zeros(3,1);   % fator de escala acelerômetro

EKF_DI_params.x0 = x0;

%% Covariância inicial

P0 = diag([ ...
    1.0^2*ones(1,3), ...
    0.5^2*ones(1,3), ...
    deg2rad(2)^2*ones(1,3), ...
    sigma_ba0^2*ones(1,3), ...
    sigma_bg0^2*ones(1,3), ...
    3.0^2*ones(1,3), ...
    0.01^2*ones(1,3) ...
]);

EKF_DI_params.P0 = P0;

%% Ruídos de processo

Qw = diag([ ...
    sigma_acc^2*ones(1,3), ...
    sigma_gyro^2*ones(1,3), ...
    sigma_ba^2*ones(1,3), ...
    sigma_bg^2*ones(1,3), ...
    sigma_by^2*ones(1,3) ...
]);

EKF_DI_params.Qw = Qw;

%% Convenção da aceleração

% acc_b_in vindo do ins_read_xplane é aceleração translacional no corpo.
% Portanto, no EKF:
%   a_n = Rb2n * acc_b_corrigida
% sem adicionar/subtrair gravidade.
EKF_DI_params.acc_input_is_translational = true;

%% Time ranges without GPS/yaw correction
try
    EKF_DI_params.range_time_without_correction = ...
        evalin('base','range_time_without_correction');
catch
    EKF_DI_params.range_time_without_correction = [0 0];
end

%% Token de reinicialização

EKF_DI_params.reset_token = now;

assignin('base', 'EKF_DI_params', EKF_DI_params);

clear EKF_DI_solver;
clear EKF_DI_EM_solver;

disp('--- EKF_DI_params inicializado ---');
fprintf('Euler0 [deg] = [%.3f %.3f %.3f]\n', rad2deg(EKF_DI_params.euler0));
fprintf('Pos0 NED [m] = [%.3f %.3f %.3f]\n', EKF_DI_params.pos0_ned);
fprintf('Vel0 NED [m/s] = [%.3f %.3f %.3f]\n', EKF_DI_params.vel0_ned);

end