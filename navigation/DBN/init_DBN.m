function DBN_params = init_DBN(xplane_init)
% init_DBN
%
% Inicializa os parâmetros do DBN a partir da condição inicial efetivamente
% aplicada no X-Plane.
%
% Entrada:
%   xplane_init.euler0    = [phi0; theta0; psi0] [rad]
%   xplane_init.pos0_ned  = [N0; E0; D0] [m]
%   xplane_init.vel0_ned  = [vN0; vE0; vD0] [m/s]
%
% Saída:
%   DBN_params no workspace base.

DBN_params = struct();

DBN_params.initialized = true;

DBN_params.euler0 = xplane_init.euler0(:);
DBN_params.pos0_ned = xplane_init.pos0_ned(:);
DBN_params.vel0_ned = xplane_init.vel0_ned(:);

DBN_params.g0 = 9.81;

% Altitude física positiva para cima.
% Em NED: D = -altitude.
DBN_params.altitude0 = -DBN_params.pos0_ned(3);

% Notação usada no DBN:
% Farrel: b = [b1; b2; b3; b4], com b4 escalar.
DBN_params.quat_notation = 'farrel_b';

% Convenção validada nos testes:
% acc_n = R_acc * acc_b - g_ned
DBN_params.subtract_gravity = true;

% Token usado para identificar nova inicialização.
% Útil caso múltiplas simulações sejam executadas na mesma sessão.
DBN_params.reset_token = now;

assignin('base', 'DBN_params', DBN_params);

% Limpa persistentes do solver entre simulações.
clear DBN_solver;

disp('--- DBN_params inicializado a partir do estado inicial do X-Plane ---');
fprintf('Euler0 [deg] = [%.3f %.3f %.3f]\n', rad2deg(DBN_params.euler0));
fprintf('Pos0 NED [m] = [%.3f %.3f %.3f]\n', DBN_params.pos0_ned);
fprintf('Vel0 NED [m/s] = [%.3f %.3f %.3f]\n', DBN_params.vel0_ned);
fprintf('Altitude0 [m] = %.3f\n', DBN_params.altitude0);

end