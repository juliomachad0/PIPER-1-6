function acc_meas_ms2 = ins_acc_errors_icm20689(acc_true_ms2, T, noise_ms2, sensors)
%#codegen
% INS_ACC_ERROS_ICM20689 - núcleo do modelo de erros do acelerômetro ICM20689.
%
% Entradas:
%   acc_true_ms2 : [3x1] aceleração verdadeira em m/s^2
%   T            : [1x1] temperatura em °C
%   noise_ms2    : [3x1] ruído externo em m/s^2, já amostrado em Ts
%   sensors      : struct de sensores inicializada por ins_init_sensors()
%
% Saída:
%   acc_meas_ms2 : [3x1] aceleração medida em m/s^2

acc_true_ms2 = acc_true_ms2(:);
noise_ms2    = noise_ms2(:);

% Lê parâmetros fixos do workspace/modelo
a = sensors.icm20689.acc;

% -------------------------------------------------------------------------
% (1) Montagem nominal corpo -> sensor
% -------------------------------------------------------------------------
roll  = deg2rad(a.roll_mount_deg);
pitch = deg2rad(a.pitch_mount_deg);
yaw   = deg2rad(a.yaw_mount_deg);

R_roll = [ ...
    1,          0,           0; ...
    0,  cos(roll),   sin(roll); ...
    0, -sin(roll),   cos(roll)];

R_pitch = [ ...
     cos(pitch), 0, -sin(pitch); ...
              0, 1,           0; ...
     sin(pitch), 0,  cos(pitch)];

R_yaw = [ ...
     cos(yaw),  sin(yaw), 0; ...
    -sin(yaw),  cos(yaw), 0; ...
            0,         0, 1];

R_b2s = R_roll * R_pitch * R_yaw;

acc_nom = R_b2s * acc_true_ms2;

% -------------------------------------------------------------------------
% (2) Misalignment fino do acelerômetro
% -------------------------------------------------------------------------
acc_mis = a.R_mis * acc_nom;

% -------------------------------------------------------------------------
% (3) Cross-axis
% -------------------------------------------------------------------------
acc_cross = a.Ca * acc_mis;

% -------------------------------------------------------------------------
% (4) Scale factor fixo + variação térmica
% -------------------------------------------------------------------------
sf = a.sf_fixed + (T - a.T_ref) .* a.k_sf;
acc_sf = acc_cross .* sf(:);

% -------------------------------------------------------------------------
% (5) Não-linearidade cúbica
% -------------------------------------------------------------------------
FS_ms2 = a.fundo_escala;

acc_nl = acc_sf + (acc_sf.^3) .* (a.k3(:) / (FS_ms2^2));

% -------------------------------------------------------------------------
% (6) Bias zero-g + drift térmico
% -------------------------------------------------------------------------
bias_g = a.bias0_g + (T - a.T_ref) .* a.kT_bias_g_per_C;
bias_ms2 = bias_g(:) * a.gravity;

% -------------------------------------------------------------------------
% (7) Soma com ruído externo
% -------------------------------------------------------------------------
acc_ana = acc_nl + bias_ms2 + noise_ms2;

% -------------------------------------------------------------------------
% (8) Saturação em ±FS
% -------------------------------------------------------------------------
acc_sat = min(max(acc_ana, -FS_ms2), FS_ms2);

% -------------------------------------------------------------------------
% (9) Quantização em LSB
% -------------------------------------------------------------------------
acc_sat_g = acc_sat / a.gravity;

counts = round(acc_sat_g * a.sens_LSB_per_g);
counts = min(max(counts, -32768), 32767);

% -------------------------------------------------------------------------
% (10) Zona morta pós-quantização
% -------------------------------------------------------------------------
D = a.deadzone_LSB;

abs_c = abs(counts);
counts_dz = zeros(size(counts));

mask = abs_c > D;
counts_dz(mask) = sign(counts(mask)) .* (abs_c(mask) - D);

% -------------------------------------------------------------------------
% (11) Volta para m/s^2
% -------------------------------------------------------------------------
acc_meas_g = counts_dz / a.sens_LSB_per_g;
acc_meas_ms2 = acc_meas_g * a.gravity;

end