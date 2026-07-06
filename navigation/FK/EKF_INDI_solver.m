function [euler_out, vel_out, pos_out, acc_n_out, xhat_out] = EKF_INDI_solver( ...
    acc_b_in, gyro_b_in, pos_meas_in, vel_meas_in, reset, sample_valid, t_now, EKF_INDI_params)
% EKF_INDI_solver
%
% Adaptacao para Simulink do EKF indireto 3D isolado.
%
% Estado mantido:
%   x_hat = [p^n; v^n; eta; ba; bg; by; alpha_a]
%
% Entradas:
%   acc_b_in     = aceleracao no corpo [m/s^2]
%   gyro_b_in    = [p; q; r] [rad/s]
%   pos_meas_in  = [N; E; h] ou [N; E; D], conforme params.pos_meas_mode
%   vel_meas_in  = [vN; vE; vD] [m/s]
%   reset
%   sample_valid
%   t_now
%   EKF_INDI_params
%
% Saidas:
%   euler_out = [phi; theta; psi]
%   vel_out   = [vN; vE; vD]
%   pos_out   = [N; E; D]
%   acc_n_out = aceleracao estimada em NED
%   xhat_out  = estado completo 21x1

persistent x_hat
persistent P
persistent t_prev
persistent initialized
persistent last_reset_token
persistent next_gps_time
persistent h0_meas

nx = 21;
I3 = eye(3);
Z3 = zeros(3);

euler_out = zeros(3,1);
vel_out   = zeros(3,1);
pos_out   = zeros(3,1);
acc_n_out = zeros(3,1);
xhat_out  = zeros(nx,1);

if isempty(initialized)
    initialized = false;
end

if ~isfield(EKF_INDI_params, 'initialized') || ~EKF_INDI_params.initialized
    return;
end

if isempty(last_reset_token)
    last_reset_token = -1;
end

new_params_loaded = false;
if isfield(EKF_INDI_params, 'reset_token')
    if EKF_INDI_params.reset_token ~= last_reset_token
        new_params_loaded = true;
        last_reset_token = EKF_INDI_params.reset_token;
    end
end

%% Inicializacao
if reset || ~initialized || new_params_loaded

    x_hat = EKF_INDI_params.x0;
    P = EKF_INDI_params.P0;

    t_prev = t_now;
    initialized = true;

    next_gps_time = t_now + EKF_INDI_params.gps_period;

    % pos_meas_in tipicamente vem como [N; E; h] do X-Plane.
    h0_meas = pos_meas_in(3);

    euler_out = x_hat(7:9);
    vel_out   = x_hat(4:6);
    pos_out   = x_hat(1:3);
    acc_n_out = zeros(3,1);
    xhat_out  = x_hat;

    return;
end

%% Sem amostra valida: mantem estados
if sample_valid == 0
    euler_out = x_hat(7:9);
    vel_out   = x_hat(4:6);
    pos_out   = x_hat(1:3);
    acc_n_out = zeros(3,1);
    xhat_out  = x_hat;
    return;
end

%% Tempo
dt = t_now - t_prev;
if dt <= 0
    dt = 0;
end

%% Parametros
Qw = EKF_INDI_params.Qw;
R_pos = EKF_INDI_params.R_pos;
R_vel = EKF_INDI_params.R_vel;
lambda_y = EKF_INDI_params.lambda_y;
beta_y = exp(-lambda_y*dt);

if isfield(EKF_INDI_params, 'acc_input_is_translational')
    acc_input_is_translational = EKF_INDI_params.acc_input_is_translational;
else
    acc_input_is_translational = true;
end

gn = [0; 0; EKF_INDI_params.g0];

%% Separar estados
p_hat     = x_hat(1:3);
v_hat     = x_hat(4:6);
eta_hat   = x_hat(7:9);
ba_hat    = x_hat(10:12);
bg_hat    = x_hat(13:15);
by_hat    = x_hat(16:18);
alpha_hat = x_hat(19:21);

%% Corrigir IMU
den = 1 - alpha_hat;
for k = 1:3
    if abs(den(k)) < 1e-6
        if den(k) >= 0
            den(k) = 1e-6;
        else
            den(k) = -1e-6;
        end
    end
end

M_alpha = diag(1./den);

f_hat_b = M_alpha*(acc_b_in + ba_hat);
omega_hat_b = gyro_b_in + bg_hat;

%% Propagacao nominal
Rbn = Rb2n_321(eta_hat);
Teta = T_321(eta_hat);

if acc_input_is_translational
    % acc_b_in ja e aceleracao translacional em corpo.
    a_hat_n = Rbn*f_hat_b;
else
    % acc_b_in e forca especifica em corpo.
    a_hat_n = Rbn*f_hat_b + gn;
end

x_pred = x_hat;
x_pred(1:3)   = p_hat + v_hat*dt + 0.5*a_hat_n*dt^2;
x_pred(4:6)   = v_hat + a_hat_n*dt;
x_pred(7:9)   = eta_hat + Teta*omega_hat_b*dt;
x_pred(10:12) = ba_hat;
x_pred(13:15) = bg_hat;
x_pred(16:18) = beta_y*by_hat;
x_pred(19:21) = alpha_hat;

x_pred(7:9) = wrapToPi_local(x_pred(7:9));

%% Linearizacao
F = zeros(nx,nx);
F(1:3,4:6) = I3;

[Rphi,Rtheta,Rpsi] = dRb2n_321(eta_hat);
Fv_eta = [Rphi*f_hat_b, Rtheta*f_hat_b, Rpsi*f_hat_b];
Fv_ba = Rbn*M_alpha;
Fv_alpha = Rbn*diag((acc_b_in + ba_hat)./((1 - alpha_hat).^2));

[Tphi,Ttheta,Tpsi] = dT_321(eta_hat);
Feta_eta = [Tphi*omega_hat_b, Ttheta*omega_hat_b, Tpsi*omega_hat_b];
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

P = Phi*P*Phi' + Qd;
P = 0.5*(P + P');

x_hat = x_pred;

%% Atualizacao auxiliar a 1 Hz
do_gps_update = false;
if t_now >= next_gps_time
    do_gps_update = true;
    while next_gps_time <= t_now
        next_gps_time = next_gps_time + EKF_INDI_params.gps_period;
    end
end

if do_gps_update

    %% Posicao auxiliar
    N_meas = pos_meas_in(1);
    E_meas = pos_meas_in(2);
    z3_meas = pos_meas_in(3);

    if isfield(EKF_INDI_params, 'pos_meas_mode') && strcmpi(EKF_INDI_params.pos_meas_mode, 'NED')
        D_meas = z3_meas;
    else
        % Padrao: pos_meas_in = [N; E; h]
        D0 = EKF_INDI_params.pos0_ned(3);
        h_meas = z3_meas;
        D_meas = D0 - (h_meas - h0_meas);
    end

    z_pos = [N_meas; E_meas; D_meas];

    H_pos = [I3 Z3 Z3 Z3 Z3 I3 Z3];
    y_hat_pos = x_hat(1:3) + x_hat(16:18);
    innov_pos = z_pos - y_hat_pos;

    S_pos = H_pos*P*H_pos' + R_pos;
    K_pos = P*H_pos'/S_pos;

    dx_hat = K_pos*innov_pos;
    x_hat = x_hat + dx_hat;
    x_hat(7:9) = wrapToPi_local(x_hat(7:9));

    P = (eye(nx) - K_pos*H_pos)*P*(eye(nx) - K_pos*H_pos)' + K_pos*R_pos*K_pos';
    P = 0.5*(P + P');

    %% Velocidade auxiliar
    z_vel = vel_meas_in;

    H_vel = [Z3 I3 Z3 Z3 Z3 Z3 Z3];
    innov_vel = z_vel - x_hat(4:6);

    S_vel = H_vel*P*H_vel' + R_vel;
    K_vel = P*H_vel'/S_vel;

    dx_hat_v = K_vel*innov_vel;
    x_hat = x_hat + dx_hat_v;
    x_hat(7:9) = wrapToPi_local(x_hat(7:9));

    P = (eye(nx) - K_vel*H_vel)*P*(eye(nx) - K_vel*H_vel)' + K_vel*R_vel*K_vel';
    P = 0.5*(P + P');

end

%% Atualizar tempo
t_prev = t_now;

%% Saidas
euler_out = x_hat(7:9);
vel_out   = x_hat(4:6);
pos_out   = x_hat(1:3);      % [N; E; D]
acc_n_out = a_hat_n;
xhat_out  = x_hat;

end

%% HELPERS
function R = Rb2n_321(eta)
phi = eta(1); theta = eta(2); psi = eta(3);
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);

R = [ cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
      cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
     -sth,       sphi*cth,                    cphi*cth ];
end

function T = T_321(eta)
phi = eta(1); theta = eta(2);
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta);

if abs(cth) < 1e-6
    if cth >= 0
        cth = 1e-6;
    else
        cth = -1e-6;
    end
end

T = [1, sphi*tan(theta), cphi*tan(theta);
     0, cphi,           -sphi;
     0, sphi/cth,        cphi/cth];
end

function [Rphi,Rtheta,Rpsi] = dRb2n_321(eta)
phi = eta(1); theta = eta(2); psi = eta(3);
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);

Rphi = [0, cphi*sth*cpsi + sphi*spsi, -sphi*sth*cpsi + cphi*spsi;
        0, cphi*sth*spsi - sphi*cpsi, -sphi*sth*spsi - cphi*cpsi;
        0, cphi*cth,                  -sphi*cth];

Rtheta = [-sth*cpsi, sphi*cth*cpsi, cphi*cth*cpsi;
          -sth*spsi, sphi*cth*spsi, cphi*cth*spsi;
          -cth,     -sphi*sth,     -cphi*sth];

Rpsi = [-cth*spsi, -sphi*sth*spsi - cphi*cpsi, -cphi*sth*spsi + sphi*cpsi;
         cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
         0,         0,                            0];
end

function [Tphi,Ttheta,Tpsi] = dT_321(eta)
phi = eta(1); theta = eta(2);
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta);

if abs(cth) < 1e-6
    if cth >= 0
        cth = 1e-6;
    else
        cth = -1e-6;
    end
end

sec_th = 1/cth;
tan_th = tan(theta);

Tphi = [0, cphi*tan_th, -sphi*tan_th;
        0, -sphi,       -cphi;
        0, cphi*sec_th, -sphi*sec_th];

Ttheta = [0, sphi*sec_th^2,      cphi*sec_th^2;
          0, 0,                  0;
          0, sphi*sec_th*tan_th, cphi*sec_th*tan_th];

Tpsi = zeros(3);
end

function ang = wrapToPi_local(ang)
ang = mod(ang + pi, 2*pi) - pi;
end
