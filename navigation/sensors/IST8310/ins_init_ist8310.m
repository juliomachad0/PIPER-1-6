function ist8310 = ins_init_ist8310(seeds_ist8310)
%INS_INIT_IST8310 Inicializa parâmetros do magnetômetro IST8310.

%% (0) Reprodutibilidade
if nargin >= 1 && isfield(seeds_ist8310,'mag') && isfield(seeds_ist8310.mag,'params')
    rng(seeds_ist8310.mag.params);
else
    rng(11);
end

%% (1) Parâmetros básicos
mag = struct();

mag.Ts = 1/200;
mag.Fs = 1/mag.Ts;

mag.noise_BW_Hz = 50;

mag.range_xy_uT = 1600;
mag.range_z_uT  = 2500;
mag.range_uT = [mag.range_xy_uT, mag.range_xy_uT, mag.range_z_uT];

mag.bits = 16;
mag.sens_LSB_per_uT = 3.3;
mag.resolution_uT_per_LSB = 1/mag.sens_LSB_per_uT;

%% (2) Parâmetros de erro
mag.linearity = [0.01 0.001 0.001];

mag.offset0_uT = [0.3 -0.3 0.3];

mag.sens_temp_drift = 0.00016;
mag.offset_temp_drift_uT_per_C = 0.024;

mag.hysteresis = 0.001;

mag.misalignment_rad = deg2rad(0.05);
mag.cross_axis = 0.01;

mag.noise_density_uT_sqrtHz = 0.05;
mag.sigma_noise_uT = mag.noise_density_uT_sqrtHz * sqrt(mag.noise_BW_Hz);

mag.dead_zone_uT = 0.05;
mag.T_ref = 25;

%% (3) Campo magnético local NED
mag.mag_n_ref_uT = [23; -5; -38];

mag.declination_rad = atan2(mag.mag_n_ref_uT(2), mag.mag_n_ref_uT(1));

%% (4) Matrizes de desalinhamento e cross-axis
a = mag.misalignment_rad;

mag.M_align = [ ...
    1,  a, 0; ...
    -a,  1, 0; ...
    0,  0, 1];

c = mag.cross_axis;

mag.M_cross = [ ...
    1, c, c; ...
    c, 1, c; ...
    c, c, 1];

mag.M_mag = mag.M_cross * mag.M_align;

%% (5) Coeficientes fixos sorteados
mag.bias0_uT = mag.offset0_uT;

mag.kT_bias_uT_per_C = ...
    mag.offset_temp_drift_uT_per_C * (2*rand(1,3)-1);

mag.sf_fixed = 1 + (mag.linearity/3).*randn(1,3);

mag.k_sf = (mag.sens_temp_drift/3) * randn(1,3);

mag.k3 = (2*rand(1,3)-1).*mag.linearity;

%% (6) Limites digitais
mag.raw_min_LSB = -2^(mag.bits-1);
mag.raw_max_LSB =  2^(mag.bits-1) - 1;

%% (7) Seeds para ruído no Simulink
if nargin >= 1 && isfield(seeds_ist8310,'mag') && isfield(seeds_ist8310.mag,'noise')
    mag.noise_seed = seeds_ist8310.mag.noise;
else
    mag.noise_seed = [4011 4012 4013];
end

%% (8) Struct final
ist8310 = struct();
ist8310.mag = mag;

disp("ist8310 sensor successfully created")
end