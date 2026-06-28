function [euler_out, vel_out, pos_out, acc_n_out, qb_out] = DBN_solver( ...
    acc_b_in, gyro_b_in, reset, sample_valid, t_now, DBN_params)

persistent qb_state
persistent vel_state
persistent pos_state
persistent acc_n_prev
persistent gyro_prev
persistent t_prev
persistent initialized
persistent last_reset_token

if isempty(initialized)
    initialized = false;
end

if ~isfield(DBN_params, 'initialized') || ~DBN_params.initialized
    euler_out = zeros(3,1);
    vel_out   = zeros(3,1);
    pos_out   = zeros(3,1);
    acc_n_out = zeros(3,1);
    qb_out    = [0;0;0;1];
    return;
end

if isempty(last_reset_token)
    last_reset_token = -1;
end

new_params_loaded = false;

if isfield(DBN_params, 'reset_token')
    if DBN_params.reset_token ~= last_reset_token
        new_params_loaded = true;
        last_reset_token = DBN_params.reset_token;
    end
end

g_ned = [0; 0; DBN_params.g0];

%% Inicialização

if reset || ~initialized || new_params_loaded

    euler0 = DBN_params.euler0;
    pos0   = DBN_params.pos0_ned;
    vel0   = DBN_params.vel0_ned;

    phi0   = euler0(1);
    theta0 = euler0(2);
    psi0   = euler0(3);

    Rn2b0 = Rn2b_from_euler(phi0, theta0, psi0);

    b4 = 0.5 * sqrt(1 + Rn2b0(1,1) + Rn2b0(2,2) + Rn2b0(3,3));

    qb_state = zeros(4,1);

    qb_state(1) = (Rn2b0(3,2) - Rn2b0(2,3)) / (4*b4);
    qb_state(2) = (Rn2b0(1,3) - Rn2b0(3,1)) / (4*b4);
    qb_state(3) = (Rn2b0(2,1) - Rn2b0(1,2)) / (4*b4);
    qb_state(4) = b4;

    qb_state = qb_state / norm(qb_state);

    vel_state = vel0;
    pos_state = pos0;

    R_acc = quatb_to_Racc(qb_state);

    acc_n_prev = R_acc * acc_b_in - g_ned;

    gyro_prev = gyro_b_in;
    t_prev = t_now;

    initialized = true;

    euler_out = quatb_to_euler_body_to_ned(qb_state);
    vel_out = vel_state;
    pos_out = pos_state;
    acc_n_out = acc_n_prev;
    qb_out = qb_state;

    return;
end

%% Se não houver amostra válida, mantém estados

if sample_valid == 0

    euler_out = quatb_to_euler_body_to_ned(qb_state);
    vel_out = vel_state;
    pos_out = pos_state;
    acc_n_out = acc_n_prev;
    qb_out = qb_state;

    return;
end

%% Tempo

dt = t_now - t_prev;

if dt <= 0
    dt = 0;
end

%% Propagação do quaternion Farrel

wx = gyro_prev(1);
wy = gyro_prev(2);
wz = gyro_prev(3);

Omega = [   0,   wz,  -wy,  wx;
          -wz,    0,   wx,  wy;
           wy,  -wx,    0,  wz;
          -wx,  -wy,  -wz,   0 ];

qbdot = 0.5 * Omega * qb_state;

qb_state = qb_state + qbdot * dt;

qb_state = qb_state / norm(qb_state);

%% Integração

vel_old = vel_state;

vel_state = vel_state + acc_n_prev * dt;

pos_state = pos_state + vel_old * dt;

%% Nova aceleração em NED

R_acc = quatb_to_Racc(qb_state);

acc_n_current = R_acc * acc_b_in - g_ned;

%% Atualização das memórias

acc_n_prev = acc_n_current;
gyro_prev = gyro_b_in;
t_prev = t_now;

%% Saídas

euler_out = quatb_to_euler_body_to_ned(qb_state);
vel_out = vel_state;
pos_out = pos_state;
acc_n_out = acc_n_current;
qb_out = qb_state;

end

%% ========================================================================
% Euler -> Rn2b
% ========================================================================

function R = Rn2b_from_euler(phi, theta, psi)

R = [ ...
    cos(psi)*cos(theta), ...
    sin(psi)*cos(theta), ...
   -sin(theta);

    cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), ...
    sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), ...
    cos(theta)*sin(phi);

    cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi), ...
    sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi), ...
    cos(theta)*cos(phi) ...
];

end

%% ========================================================================
% Matriz usada na transformação da aceleração
% ========================================================================

function R_acc = quatb_to_Racc(b)

b1 = b(1);
b2 = b(2);
b3 = b(3);
b4 = b(4);

% Mantém a mesma convenção validada no seu código final:
% acc_n = R_acc * acc_b - g_ned

R_acc = [ ...
    b1^2 + b4^2 - b2^2 - b3^2, ...
    2*(b1*b2 - b3*b4), ...
    2*(b1*b3 + b2*b4);

    2*(b1*b2 + b3*b4), ...
    b2^2 + b4^2 - b1^2 - b3^2, ...
    2*(b2*b3 - b1*b4);

    2*(b1*b3 - b2*b4), ...
    2*(b2*b3 + b1*b4), ...
    b3^2 + b4^2 - b1^2 - b2^2 ...
];

end

%% ========================================================================
% Quaternion Farrel -> Euler corpo em relação ao NED
% ========================================================================

function eul = quatb_to_euler_body_to_ned(b)

x = b(1);
y = b(2);
z = b(3);
w = b(4);

norm2 = x^2 + y^2 + z^2 + w^2;

b_inv = [-x; -y; -z; w] / norm2;

b1 = b_inv(1);
b2 = b_inv(2);
b3 = b_inv(3);
b4 = b_inv(4);

val = -2*(b2*b4 + b1*b3);

if val > 1
    val = 1;
elseif val < -1
    val = -1;
end

theta = asin(val);

phi = atan2(2*(b2*b3 - b1*b4), ...
            1 - 2*(b1^2 + b2^2));

psi = atan2(2*(b1*b2 - b3*b4), ...
            1 - 2*(b2^2 + b3^2));

eul = [phi; theta; psi];

end