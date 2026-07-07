function yaw_mag_rad = ins_mag_meas(euler_true_rad, T, noise_mag_uT, sensors)
%#codegen
%INS_MAG_MEAS Modelo de erro do magnetômetro IST8310.
%
% Entrada:
%   euler_true_rad : [3x1] [roll; pitch; yaw] verdadeiro [rad]
%   T              : temperatura [°C]
%   noise_mag_uT   : [3x1] ruído externo do magnetômetro [uT]
%   sensors        : struct sensors
%
% Saída:
%   yaw_mag_rad    : yaw medido pelo magnetômetro [rad]

euler_true_rad = euler_true_rad(:);
noise_mag_uT   = noise_mag_uT(:);

mag = sensors.ist8310.mag;

%% (1) Campo magnético ideal no corpo
Rbn = Rb2n_321_local(euler_true_rad);

mag_true_b_uT = Rbn' * mag.mag_n_ref_uT(:);

%% (2) Desalinhamento e cross-axis
mag_mis = mag.M_mag * mag_true_b_uT;

%% (3) Fator de escala fixo + térmico
sf = mag.sf_fixed(:) + (T - mag.T_ref) .* mag.k_sf(:);

mag_scale = mag_mis .* sf;

%% (4) Não-linearidade
range_uT = mag.range_uT(:);
k3 = mag.k3(:);

mag_nl = k3 .* (mag_scale.^2 ./ range_uT) .* sign(mag_scale);

%% (5) Bias fixo + drift térmico
bias_uT = mag.bias0_uT(:) + (T - mag.T_ref) .* mag.kT_bias_uT_per_C(:);

%% (6) Histerese
mag_hyst = zeros(3,1);

%% (7) Soma dos erros
mag_ana = mag_scale + mag_nl + bias_uT + mag_hyst + noise_mag_uT;

%% (8) Zona morta
D = mag.dead_zone_uT;

mag_dz = mag_ana;
mag_dz(abs(mag_dz) < D) = 0;

%% (9) Saturação
mag_sat = min(max(mag_dz, -range_uT), range_uT);

%% (10) Quantização
raw = round(mag_sat * mag.sens_LSB_per_uT);

raw = min(max(raw, mag.raw_min_LSB), mag.raw_max_LSB);

mag_meas_b_uT = raw / mag.sens_LSB_per_uT;

%% (11) Tilt compensation usando roll e pitch verdadeiros
roll  = euler_true_rad(1);
pitch = euler_true_rad(2);

R_level = Rb2n_321_local([roll; pitch; 0]);

mag_level = R_level * mag_meas_b_uT;

%% (12) Cálculo do yaw magnético corrigido por declinação
yaw_mag_rad = mag.declination_rad - atan2(mag_level(2), mag_level(1));

yaw_mag_rad = wrapToPi_local(yaw_mag_rad);

end

%% Funções locais
function R = Rb2n_321_local(euler_rad)

    phi   = euler_rad(1);
    theta = euler_rad(2);
    psi   = euler_rad(3);

    cphi = cos(phi);
    sphi = sin(phi);
    cth  = cos(theta);
    sth  = sin(theta);
    cpsi = cos(psi);
    spsi = sin(psi);

    R = [ ...
        cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi; ...
        cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi; ...
        -sth,      sphi*cth,                    cphi*cth];
end

function ang = wrapToPi_local(ang)

    ang = mod(ang + pi, 2*pi) - pi;

end