function yaw_true_degraded_rad = ins_mag_meas(euler_true_rad, T, noise_mag_uT, mag_var, sensors)
%#codegen
%INS_MAG_MEAS Modelo de erro do magnetometro IST8310.
%
% Entradas:
%   euler_true_rad        : [3x1] [phi; theta; psi] do X-Plane [rad]
%   T                     : temperatura [deg C]
%   noise_mag_uT          : [3x1] ruido externo do magnetometro [uT]
%   mag_var               : variacao magnetica local do X-Plane [rad]
%   sensors               : struct sensors
%
% Saida:
%   yaw_true_degraded_rad : yaw verdadeiro degradado pelo modelo do magnetometro [rad]

euler_true_rad = euler_true_rad(:);
noise_mag_uT   = noise_mag_uT(:);

mag = sensors.ist8310.mag;

mag_var = wrapToPi_local(mag_var);

%% (1) Campo magnetico local NED usando magnetic_variation do X-Plane
% Usa a magnitude horizontal e vertical definidas em ins_init_ist8310,
% mas substitui a direcao horizontal pela variacao magnetica do X-Plane.

H_mag_uT = norm(mag.mag_n_ref_uT(1:2));
D_mag_uT = mag.mag_n_ref_uT(3);

mag_n_ref_uT = [ ...
    H_mag_uT * cos(mag_var); ...
    H_mag_uT * sin(mag_var); ...
    D_mag_uT];

%% (2) Campo magnetico ideal no corpo
Rbn = Rb2n_321_local(euler_true_rad);

mag_true_b_uT = Rbn' * mag_n_ref_uT;

%% (3) Desalinhamento e cross-axis
mag_mis = mag.M_mag * mag_true_b_uT;

%% (4) Fator de escala fixo + termico
sf = mag.sf_fixed(:) + (T - mag.T_ref) .* mag.k_sf(:);

mag_scale = mag_mis .* sf;

%% (5) Nao-linearidade
range_uT = mag.range_uT(:);
k3 = mag.k3(:);

mag_nl = k3 .* (mag_scale.^2 ./ range_uT) .* sign(mag_scale);

%% (6) Bias fixo + drift termico
bias_uT = mag.bias0_uT(:) + (T - mag.T_ref) .* mag.kT_bias_uT_per_C(:);

%% (7) Histerese
% Mantida zerada para evitar offset artificial constante no yaw.
mag_hyst = zeros(3,1);

%% (8) Soma dos erros
mag_ana = mag_scale + mag_nl + bias_uT + mag_hyst + noise_mag_uT;

%% (9) Zona morta
D = mag.dead_zone_uT;

mag_dz = mag_ana;
mag_dz(abs(mag_dz) < D) = 0;

%% (10) Saturacao
mag_sat = min(max(mag_dz, -range_uT), range_uT);

%% (11) Quantizacao
raw = round(mag_sat * mag.sens_LSB_per_uT);

raw = min(max(raw, mag.raw_min_LSB), mag.raw_max_LSB);

mag_meas_b_uT = raw / mag.sens_LSB_per_uT;

%% (12) Tilt compensation usando roll e pitch do X-Plane
roll  = euler_true_rad(1);
pitch = euler_true_rad(2);

R_level = Rb2n_321_local([roll; pitch; 0]);

mag_level = R_level * mag_meas_b_uT;

%% (13) Yaw verdadeiro degradado
% Modelo coerente com:
%   yaw_true = magnetic_variation - atan2(E_mag_level, N_mag_level)
%
% Sem erros, essa expressao retorna aproximadamente o psi verdadeiro
% usado para gerar mag_true_b_uT.

yaw_true_degraded_rad = mag_var - atan2(mag_level(2), mag_level(1));

yaw_true_degraded_rad = wrapToPi_local(yaw_true_degraded_rad);

end

%% Funcoes locais
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