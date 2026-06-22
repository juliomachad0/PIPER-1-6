%% scr_waypoints.m - Configuracao de waypoints para simulacao
% Define waypoints via tuplas (N, E, h, vel) e parametros auxiliares,
% depois chama scr_aux_wp para executar a simulacao.
%
% Uso:
%   >> scr_waypoints
%
% Edite apenas este arquivo para configurar a missao. Nao e necessario
% rodar inicializar.m antes - ele e executado internamente por scr_aux_wp.
%
% Convencoes:
%   - Coordenadas NED (Norte, Leste, Down). Altitude h e positiva para cima.
%   - WP1 e fixo na origem (0, 0, h_eq, 15 m/s) e e inserido automaticamente.
%   - Os waypoints definidos em WPs sao usados como WP2 em diante.
%   - Se |Delta h| entre dois WPs consecutivos exceder max_dalt (30 m),
%     pontos intermediarios sao interpolados automaticamente.

%% ========== WAYPOINTS ==========
% Formato: [Norte (m), Leste (m), Altitude (m), Velocidade (m/s)]
% Defina apenas WP2, WP3, ... (WP1 na origem e adicionado automaticamente).
WPs = [
    500,    0,   80,  15;   % WP2
    500,  500,  100,  18;   % WP3
      0,  500,   80,  15;   % WP4
];

%% ========== PARAMETROS AUXILIARES ==========
R_accept = 80;        % Raio de aceitacao dos waypoints (m)
t_stop   = 200;       % Tempo de simulacao (s)
h_eq     = 100;       % Altitude de equilibrio / WP1 (m)

%% ========== EXECUTAR ==========
% Adiciona o diretorio guiagem ao path para garantir que scr_aux_wp
% seja encontrado, e chama a rotina auxiliar.
addpath(fileparts(mfilename('fullpath')));

scr_aux_wp(WPs, R_accept, t_stop, h_eq);
