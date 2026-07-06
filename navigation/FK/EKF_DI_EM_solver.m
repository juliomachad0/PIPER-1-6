function [euler_out, vel_out, pos_out, acc_n_out, xhat_out] = EKF_DI_EM_solver( ...
    acc_b_in, gyro_b_in, pos_meas_in, vel_meas_in, reset, sample_valid, t_now, EKF_DI_params)

persistent x_hat_DEM
persistent P_DEM
persistent t_prev_DEM
persistent initialized_DEM
persistent last_reset_token_DEM
persistent next_gps_time_DEM
persistent h0_meas_DEM

nx = 21;
I3 = eye(3);
Z3 = zeros(3);

if isempty(initialized_DEM)
    initialized_DEM = false;
end

if ~isfield(EKF_DI_params, 'initialized') || ~EKF_DI_params.initialized
    euler_out = zeros(3,1);
    vel_out   = zeros(3,1);
    pos_out   = zeros(3,1);
    acc_n_out = zeros(3,1);
    xhat_out  = zeros(nx,1);
    return;
end

if isempty(last_reset_token_DEM)
    last_reset_token_DEM = -1;
end

new_params_loaded = false;

if isfield(EKF_DI_params, 'reset_token')
    if EKF_DI_params.reset_token ~= last_reset_token_DEM
        new_params_loaded = true;
        last_reset_token_DEM = EKF_DI_params.reset_token;
    end
end

%% Inicialização

if reset || ~initialized_DEM || new_params_loaded

    x_hat_DEM = EKF_DI_params.x0;
    P_DEM = EKF_DI_params.P0;

    t_prev_DEM = t_now;
    initialized_DEM = true;

    next_gps_time_DEM = t_now + EKF_DI_params.gps_period;

    % pos_meas_in = [N; E; h]
    h0_meas_DEM = pos_meas_in(3);

    euler_out = x_hat_DEM(7:9);
    vel_out   = x_hat_DEM(4:6);
    pos_out = ekf_pos_output(x_hat_DEM(1:3));
    acc_n_out = zeros(3,1);
    xhat_out  = x_hat_DEM;

    return;
end

%% Se não houver amostra válida, mantém estados

if sample_valid == 0

    euler_out = x_hat_DEM(7:9);
    vel_out   = x_hat_DEM(4:6);
    pos_out = ekf_pos_output(x_hat_DEM(1:3));
    acc_n_out = zeros(3,1);
    xhat_out  = x_hat_DEM;

    return;
end

%% Tempo

dt = t_now - t_prev_DEM;

if dt <= 0
    dt = 0;
end

%% Parâmetros

Qw = EKF_DI_params.Qw;
R_pos = EKF_DI_params.R_pos;
R_vel = EKF_DI_params.R_vel;
lambda_y = EKF_DI_params.lambda_y;

beta_y = exp(-lambda_y*dt);

%% Separar estados

p_hat     = x_hat_DEM(1:3);
v_hat     = x_hat_DEM(4:6);
eta_hat   = x_hat_DEM(7:9);
ba_hat    = x_hat_DEM(10:12);
bg_hat    = x_hat_DEM(13:15);
by_hat    = x_hat_DEM(16:18);
alpha_hat = x_hat_DEM(19:21);

%% Corrigir IMU

den = 1 - alpha_hat;

for k = 1:3
    if abs(den(k)) < 1e-6
        den(k) = sign(den(k))*1e-6;
        if den(k) == 0
            den(k) = 1e-6;
        end
    end
end

M_alpha = diag(1./den);

acc_corr_b = M_alpha*(acc_b_in + ba_hat);
gyro_corr_b = gyro_b_in + bg_hat;

%% Propagação nominal

Rbn = Rb2n_321(eta_hat);
Teta = T_321(eta_hat);

% O acc_b_in atual é aceleração translacional expressa no corpo.
a_hat_n = Rbn * acc_corr_b;

x_pred = x_hat_DEM;

x_pred(1:3)   = p_hat + v_hat*dt + 0.5*a_hat_n*dt^2;
x_pred(4:6)   = v_hat + a_hat_n*dt;
x_pred(7:9)   = eta_hat + Teta*gyro_corr_b*dt;
x_pred(10:12) = ba_hat;
x_pred(13:15) = bg_hat;
x_pred(16:18) = beta_y*by_hat;
x_pred(19:21) = alpha_hat;

x_pred(9) = atan2(sin(x_pred(9)), cos(x_pred(9)));

%% Linearização

F = zeros(nx,nx);

F(1:3,4:6) = I3;

[Rphi,Rtheta,Rpsi] = dRb2n_321(eta_hat);
Fv_eta = [Rphi*acc_corr_b, Rtheta*acc_corr_b, Rpsi*acc_corr_b];

Fv_ba = Rbn*M_alpha;

Fv_alpha = Rbn*diag((acc_b_in + ba_hat)./((1 - alpha_hat).^2));

[Tphi,Ttheta,Tpsi] = dT_321(eta_hat);
Feta_eta = [Tphi*gyro_corr_b, Ttheta*gyro_corr_b, Tpsi*gyro_corr_b];
Feta_bg = Teta;

F(4:6,7:9)     = Fv_eta;
F(4:6,10:12)   = Fv_ba;
F(4:6,19:21)   = Fv_alpha;

F(7:9,7:9)     = Feta_eta;
F(7:9,13:15)   = Feta_bg;

F(16:18,16:18) = -lambda_y*I3;

G = zeros(nx,15);

G(4:6,1:3)     = Rbn*M_alpha;
G(7:9,4:6)     = Teta;
G(10:12,7:9)   = I3;
G(13:15,10:12) = I3;
G(16:18,13:15) = I3;

Phi = eye(nx) + F*dt;
Qd = G*Qw*G'*dt;

P_DEM = Phi*P_DEM*Phi' + Qd;
P_DEM = 0.5*(P_DEM + P_DEM');

x_hat_DEM = x_pred;

%% Atualização GPS/Pseudo-GPS a 1 Hz

do_gps_update = false;

if t_now >= next_gps_time_DEM
    do_gps_update = true;

    while next_gps_time_DEM <= t_now
        next_gps_time_DEM = next_gps_time_DEM + EKF_DI_params.gps_period;
    end
end

if do_gps_update

    %% Medição de posição
    % pos_meas_in = [N; E; h]
    % Converter h para D relativo ao D0 do filtro.
    N_meas = pos_meas_in(1);
    E_meas = pos_meas_in(2);
    h_meas = pos_meas_in(3);

    D0 = EKF_DI_params.pos0_ned(3);
    D_meas = D0 - (h_meas - h0_meas_DEM);

    z_pos = [N_meas; E_meas; D_meas];

    H_pos = [I3 Z3 Z3 Z3 Z3 I3 Z3];

    y_hat_pos = x_hat_DEM(1:3) + x_hat_DEM(16:18);

    innov_pos = z_pos - y_hat_pos;

    S_pos = H_pos*P_DEM*H_pos' + R_pos;
    K_pos = P_DEM*H_pos'/S_pos;

    x_hat_DEM = x_hat_DEM + K_pos*innov_pos;

    P_DEM = (eye(nx) - K_pos*H_pos)*P_DEM*(eye(nx) - K_pos*H_pos)' + K_pos*R_pos*K_pos';
    P_DEM = 0.5*(P_DEM + P_DEM');

    %% Medição de velocidade
    z_vel = vel_meas_in;

    H_vel = [Z3 I3 Z3 Z3 Z3 Z3 Z3];

    innov_vel = z_vel - x_hat_DEM(4:6);

    S_vel = H_vel*P_DEM*H_vel' + R_vel;
    K_vel = P_DEM*H_vel'/S_vel;

    x_hat_DEM = x_hat_DEM + K_vel*innov_vel;

    P_DEM = (eye(nx) - K_vel*H_vel)*P_DEM*(eye(nx) - K_vel*H_vel)' + K_vel*R_vel*K_vel';
    P_DEM = 0.5*(P_DEM + P_DEM');

end

%% Atualização do tempo

t_prev_DEM = t_now;

%% Saídas

euler_out = x_hat_DEM(7:9);
vel_out   = x_hat_DEM(4:6);
pos_out   = ekf_pos_output(x_hat_DEM(1:3));
acc_n_out = a_hat_n;
xhat_out  = x_hat_DEM;

end

%-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
%-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
%-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
%% HELPERS
%-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
function R = Rb2n_321(eta)

phi = eta(1);
theta = eta(2);
psi = eta(3);

cphi = cos(phi);
sphi = sin(phi);
cth = cos(theta);
sth = sin(theta);
cpsi = cos(psi);
spsi = sin(psi);

R = [ ...
    cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
    cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
   -sth,       sphi*cth,                    cphi*cth ...
];

end

function T = T_321(eta)

phi = eta(1);
theta = eta(2);

cphi = cos(phi);
sphi = sin(phi);
cth = cos(theta);

if abs(cth) < 1e-6
    cth = sign(cth)*1e-6;
    if cth == 0
        cth = 1e-6;
    end
end

T = [ ...
    1, sphi*tan(theta), cphi*tan(theta);
    0, cphi,           -sphi;
    0, sphi/cth,        cphi/cth ...
];

end

function [Rphi,Rtheta,Rpsi] = dRb2n_321(eta)

phi = eta(1);
theta = eta(2);
psi = eta(3);

cphi = cos(phi);
sphi = sin(phi);
cth = cos(theta);
sth = sin(theta);
cpsi = cos(psi);
spsi = sin(psi);

Rphi = [ ...
    0,  cphi*sth*cpsi + sphi*spsi, -sphi*sth*cpsi + cphi*spsi;
    0,  cphi*sth*spsi - sphi*cpsi, -sphi*sth*spsi - cphi*cpsi;
    0,  cphi*cth,                  -sphi*cth ...
];

Rtheta = [ ...
   -sth*cpsi,  sphi*cth*cpsi,  cphi*cth*cpsi;
   -sth*spsi,  sphi*cth*spsi,  cphi*cth*spsi;
   -cth,      -sphi*sth,      -cphi*sth ...
];

Rpsi = [ ...
   -cth*spsi, -sphi*sth*spsi - cphi*cpsi, -cphi*sth*spsi + sphi*cpsi;
    cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
    0,         0,                          0 ...
];

end

function [Tphi,Ttheta,Tpsi] = dT_321(eta)

phi = eta(1);
theta = eta(2);

cphi = cos(phi);
sphi = sin(phi);
cth = cos(theta);

if abs(cth) < 1e-6
    cth = sign(cth)*1e-6;
    if cth == 0
        cth = 1e-6;
    end
end

sec_th = 1/cth;
tan_th = tan(theta);

Tphi = [ ...
    0,  cphi*tan_th,        -sphi*tan_th;
    0, -sphi,               -cphi;
    0,  cphi*sec_th,        -sphi*sec_th ...
];

Ttheta = [ ...
    0, sphi*(sec_th^2),          cphi*(sec_th^2);
    0, 0,                        0;
    0, sphi*sec_th*tan_th,       cphi*sec_th*tan_th ...
];

Tpsi = zeros(3);

end
function pos_out = ekf_pos_output(pos_ned)

N = pos_ned(1);
E = pos_ned(2);
D = pos_ned(3);

altitude = -D;

pos_out = [N; E; altitude];

end