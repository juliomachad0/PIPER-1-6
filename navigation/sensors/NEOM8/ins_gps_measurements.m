function y_gps = ins_gps_measurements(pos_ned_true, vel_ned_true, t, ...
                                      noise_pos_m, noise_vel_ms, w_bias_pos, sensors)
%INS_GPS_MEASUREMENTS Aplica modelo de erro GNSS.
%
% Entradas variantes no tempo:
%   pos_ned_true : [3x1] posição verdadeira NED [m]
%   vel_ned_true : [3x1] velocidade verdadeira NED [m/s]
%   t            : [1x1] tempo [s]
%   noise_pos_m  : [3x1] ruído branco de posição [m]
%   noise_vel_ms : [3x1] ruído branco de velocidade [m/s]
%   w_bias_pos   : [3x1] ruído normal unitário para excitar bias
%
% Parâmetros fixos:
%   gps          : sensors.gps
%
% Saída:
%   y_gps        : [6x1] [pos_NED; vel_NED] com erro aplicado

persistent bias_pos last_t initialized

gps = sensors.gps; % parametros do gps
pos_ned_true = pos_ned_true(:);
vel_ned_true = vel_ned_true(:);
noise_pos_m  = noise_pos_m(:);
noise_vel_ms = noise_vel_ms(:);
w_bias_pos   = w_bias_pos(:);

%% (0) Inicialização persistente
if isempty(initialized) || t < last_t
    bias_pos = gps.bias0_pos(:);
    last_t = t;
    initialized = true;
end

%% (1) Passo de tempo
dt = t - last_t;

if dt <= 0
    dt = gps.Ts;
end

%% (2) Bias de posição - Gauss-Markov
phi = exp(-gps.lambda_bias * dt);

qd = gps.sigma_bias_ss_NED(:) * sqrt(max(0, 1 - phi^2));

bias_pos = phi * bias_pos + qd .* w_bias_pos;

last_t = t;

%% (3) Medição de posição
pos_meas = pos_ned_true + bias_pos + noise_pos_m;

%% (4) Medição de velocidade
vel_meas = vel_ned_true + noise_vel_ms;

%% (5) Saída
y_gps = [pos_meas; vel_meas];

end