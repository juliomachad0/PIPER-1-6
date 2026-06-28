function omega_meas_dps = ins_gyro_errors_icm20689(omega_true_dps, T, noise_dps, sensors)
omega_true_dps = omega_true_dps(:);
noise_dps      = noise_dps(:);
%#codegen
% GyroSensor_ICM20689 - núcleo do modelo em °/s.
% Entradas:
%   omega_true_dps : [3x1] taxa verdadeira em °/s
%   T              : [1x1] temperatura em °C
%   noise_dps      : [3x1] ruído externo em °/s (já amostrado em Ts)
% Saída:
%   omega_meas_dps : [3x1] taxa medida em °/s
% Lê parâmetros fixos do workspace (congelados em tempo de execução)
g = sensors.icm20689.gyro;
% --- (1) Montagem nominal ---
roll = deg2rad(g.yaw_mount_deg);
pit = deg2rad(g.yaw_mount_deg);
yaw = deg2rad(g.yaw_mount_deg);
R_roll = [1 0 0; 0 cos(roll) sin(roll);0 -sin(roll) cos(roll)];
R_pitch = [cos(pit) 0 -sin(pit); 0 1 0; sin(pit) 0 cos(pit)];
R_yaw = [ cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0;0 0 1];
R_b2s = R_roll*R_pitch*R_yaw;
omega_nom = R_b2s* omega_true_dps;
% --- (2) Cross-axis ---
omega_cross = g.Cg * omega_nom;
% --- (3) Scale factor (fixo + variação com T, se você estiver usando) ---
sf = g.sf_fixed + (T - g.T_ref).*g.k_sf;   % 1x3
omega_sf = omega_cross .* sf(:);
% --- (4) Não-linearidade (opcional) ---
FS = g.FS_dps;
omega_nl = omega_sf + (omega_sf.^3) .* (g.k3(:) / (FS^2));
% --- (5) Bias (ZRO + drift térmico) ---
bias = g.b0 + (T - g.T_ref).*g.k_zro;      % 1x3
bias = bias(:);
% --- (6) Soma com ruído externo ---
omega_ana = omega_nl + bias + noise_dps;
% --- (7) Saturação em ±FS ---
omega_sat = min(max(omega_ana, -FS), FS);
% --- (8) Quantização (LSB) ---
counts = round(omega_sat * g.sens_LSB_per_dps);
counts = min(max(counts, -32768), 32767);
% --- (9) Zona morta (soft deadband) ---
D = g.deadzone_LSB;
abs_c = abs(counts);
counts_dz = zeros(size(counts));
mask = abs_c > D;
counts_dz(mask) = sign(counts(mask)) .* (abs_c(mask) - D);
% --- (10) Volta para °/s ---
omega_meas_dps = counts_dz / g.sens_LSB_per_dps;
end